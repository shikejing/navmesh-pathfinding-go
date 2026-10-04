package navmesh

import (
	"math"
	"sort"
)

// Build constructs a navigation mesh from scratch. The result is
// deterministic: the same scene and config always produce an identical
// mesh (identical checksums).
func Build(scene Scene, cfg Config) *NavMesh {
	cfg = cfg.withDefaults()
	zones := normalizeZones(scene.Zones)
	cols := maxInt(1, int(math.Ceil((scene.Bounds.MaxX-scene.Bounds.MinX)/cfg.CellSize)))
	rows := maxInt(1, int(math.Ceil((scene.Bounds.MaxY-scene.Bounds.MinY)/cfg.CellSize)))
	m := &NavMesh{
		Cfg:         cfg,
		Scene:       Scene{Bounds: scene.Bounds, Zones: zones},
		Version:     1,
		Cols:        cols,
		Rows:        rows,
		TileCols:    cfg.TileCells,
		TilesX:      ceilDiv(cols, cfg.TileCells),
		TilesY:      ceilDiv(rows, cfg.TileCells),
		WorldBounds: scene.Bounds,
	}
	m.Tiles = make([]*Tile, m.TilesX*m.TilesY)

	// Pass 1: rasterize blocked flags and cost classes per tile.
	raw := make([][]Cell, len(m.Tiles))
	for ty := 0; ty < m.TilesY; ty++ {
		for tx := 0; tx < m.TilesX; tx++ {
			raw[ty*m.TilesX+tx] = rasterizeTile(m, zones, tx, ty)
		}
	}
	get := func(cx, cy int) (Cell, bool) {
		if cx < 0 || cy < 0 || cx >= m.Cols || cy >= m.Rows {
			return Cell{}, false
		}
		tx, ty := cx/m.TileCols, cy/m.TileCols
		w := tileWidth(m, tx)
		return raw[ty*m.TilesX+tx][(cy-ty*m.TileCols)*w+(cx-tx*m.TileCols)], true
	}
	// Pass 2: clearance, region decomposition, checksums.
	for ty := 0; ty < m.TilesY; ty++ {
		for tx := 0; tx < m.TilesX; tx++ {
			i := ty*m.TilesX + tx
			cells := raw[i]
			fillClearance(m, get, tx, ty, cells)
			regions, _ := decompose(m, i, tx, ty, cells)
			m.Tiles[i] = &Tile{Index: i, TX: tx, TY: ty, Cells: cells, Regions: regions}
			m.Tiles[i].Checksum = tileChecksum(m.Tiles[i])
		}
	}
	m.assemble()
	return m
}

// normalizeZones deduplicates zones by ID (last wins) and sorts them by ID
// so build results do not depend on input order.
func normalizeZones(zones []Zone) []Zone {
	byID := make(map[string]Zone, len(zones))
	for _, z := range zones {
		if z.Kind == ZoneCost && z.Cost < 1 {
			z.Cost = 1
		}
		byID[z.ID] = z
	}
	out := make([]Zone, 0, len(byID))
	for _, z := range byID {
		out = append(out, z)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out
}

// rasterizeTile computes blocked/cost/forbidden/ceiling for every cell of
// tile (tx,ty). Clearance is left zero and filled later.
func rasterizeTile(m *NavMesh, zones []Zone, tx, ty int) []Cell {
	w, h := tileWidth(m, tx), tileHeight(m, ty)
	cells := make([]Cell, w*h)
	claimed := make([]bool, w*h) // a cost zone already claimed the cell
	for i := range cells {
		cells[i].Cost = 1
	}
	x0 := tx * m.TileCols
	y0 := ty * m.TileCols
	for _, z := range zones {
		for ly := 0; ly < h; ly++ {
			for lx := 0; lx < w; lx++ {
				c := m.CellCenter(x0+lx, y0+ly)
				if !z.Rect.Contains(c) {
					continue
				}
				cell := &cells[ly*w+lx]
				if z.Kind == ZoneObstacle {
					*cell = Cell{Blocked: true, Cost: 1}
					continue
				}
				if cell.Blocked {
					continue // obstacles always win
				}
				// First (lowest-ID) cost zone covering the cell wins.
				if claimed[ly*w+lx] {
					continue
				}
				claimed[ly*w+lx] = true
				cell.Cost = z.Cost
				cell.Forbidden = z.Forbidden
				cell.Ceiling = z.Ceiling
			}
		}
	}
	return cells
}

// haloCells is the clearance halo in cells around a tile.
func haloCells(m *NavMesh) int {
	return int(math.Ceil(m.Cfg.MaxAgentRadius/m.Cfg.CellSize)) + 1
}

// fillClearance computes per-cell clearance for tile (tx,ty) via a
// multi-source BFS from blocked cells inside the tile plus its halo,
// combined with the exact distance to the map edge. Values saturate at
// MaxAgentRadius + CellSize.
func fillClearance(m *NavMesh, get func(cx, cy int) (Cell, bool), tx, ty int, cells []Cell) {
	w, h := tileWidth(m, tx), tileHeight(m, ty)
	halo := haloCells(m)
	cs := m.Cfg.CellSize
	sat := m.Cfg.MaxAgentRadius + cs
	maxSteps := int(math.Ceil(sat / cs))

	gx0, gy0 := tx*m.TileCols, ty*m.TileCols
	wx0 := maxInt(0, gx0-halo)
	wy0 := maxInt(0, gy0-halo)
	wx1 := minInt(m.Cols-1, gx0+w-1+halo)
	wy1 := minInt(m.Rows-1, gy0+h-1+halo)
	ww, wh := wx1-wx0+1, wy1-wy0+1

	const inf = math.MaxInt32 / 2
	dist := make([]int, ww*wh)
	for i := range dist {
		dist[i] = inf
	}
	var queue []int
	for y := wy0; y <= wy1; y++ {
		for x := wx0; x <= wx1; x++ {
			c, _ := get(x, y)
			if c.Blocked {
				idx := (y-wy0)*ww + (x - wx0)
				dist[idx] = 0
				queue = append(queue, idx)
			}
		}
	}
	dx := [4]int{1, -1, 0, 0}
	dy := [4]int{0, 0, 1, -1}
	for head := 0; head < len(queue); head++ {
		idx := queue[head]
		d := dist[idx]
		if d >= maxSteps {
			continue
		}
		cx, cy := idx%ww, idx/ww
		for k := 0; k < 4; k++ {
			nx, ny := cx+dx[k], cy+dy[k]
			if nx < 0 || ny < 0 || nx >= ww || ny >= wh {
				continue
			}
			nidx := ny*ww + nx
			if dist[nidx] > d+1 {
				dist[nidx] = d + 1
				queue = append(queue, nidx)
			}
		}
	}
	for ly := 0; ly < h; ly++ {
		for lx := 0; lx < w; lx++ {
			i := ly*w + lx
			if cells[i].Blocked {
				cells[i].Clearance = 0
				continue
			}
			gx, gy := gx0+lx, gy0+ly
			d := dist[(gy-wy0)*ww+(gx-wx0)]
			cl := float64(d) * cs
			if d >= inf/2 {
				cl = sat
			}
			// Distance from cell centre to the map edge.
			edge := float64(minInt(minInt(gx, m.Cols-1-gx), minInt(gy, m.Rows-1-gy)))*cs + 0.5*cs
			if edge < cl {
				cl = edge
			}
			if cl > sat {
				cl = sat
			}
			cells[i].Clearance = cl
		}
	}
}

// cellClass identifies cells that may share a region.
type cellClass struct {
	cost      float64
	forbidden bool
	ceiling   float64
	band      int
}

// clearanceBand quantizes clearance into four bands relative to
// MaxAgentRadius. Banding keeps near-obstacle cells from merging into
// large open regions, so region-level minimum clearance stays useful for
// agent-radius filtering.
func clearanceBand(cl, maxR float64) int {
	switch {
	case cl < maxR*0.25:
		return 0
	case cl < maxR*0.5:
		return 1
	case cl < maxR:
		return 2
	default:
		return 3
	}
}

func (m *NavMesh) classOf(c Cell) cellClass {
	return cellClass{
		cost:      c.Cost,
		forbidden: c.Forbidden,
		ceiling:   c.Ceiling,
		band:      clearanceBand(c.Clearance, m.Cfg.MaxAgentRadius),
	}
}

// decompose splits the walkable cells of a tile into maximal rectangles
// via a deterministic scanline, and assigns each cell its local region
// index (or -1). Concave areas become multiple adjacent rectangles.
func decompose(m *NavMesh, tileIndex, tx, ty int, cells []Cell) ([]Region, []int32) {
	w, h := tileWidth(m, tx), tileHeight(m, ty)
	regionOf := make([]int32, w*h)
	for i := range regionOf {
		regionOf[i] = -1
	}
	type active struct {
		x0, x1    int
		class     cellClass
		region    int
		clearance float64
		ceiling   float64 // min of cell ceilings, 0 (unlimited) treated as +inf
	}
	var regions []Region
	var actives []active // keyed by run position; rebuilt per row
	gx0, gy0 := tx*m.TileCols, ty*m.TileCols

	finish := func(a *active, yEnd int) {
		r := &regions[a.region]
		r.MaxY = gy0 + yEnd
		r.Clearance = a.clearance
		if a.ceiling == math.Inf(1) {
			r.Ceiling = 0
		} else {
			r.Ceiling = a.ceiling
		}
	}

	for y := 0; y < h; y++ {
		var next []active
		x := 0
		for x < w {
			i := y*w + x
			if cells[i].Blocked {
				x++
				continue
			}
			cls := m.classOf(cells[i])
			x0 := x
			for x < w && !cells[y*w+x].Blocked && m.classOf(cells[y*w+x]) == cls {
				x++
			}
			x1 := x - 1
			// Find a matching active rectangle to extend.
			extended := false
			for ai := range actives {
				a := &actives[ai]
				if a.x0 == x0 && a.x1 == x1 && a.class == cls {
					for lx := x0; lx <= x1; lx++ {
						regionOf[y*w+lx] = int32(a.region)
						c := cells[y*w+lx]
						if c.Clearance < a.clearance {
							a.clearance = c.Clearance
						}
						a.ceiling = minCeiling(a.ceiling, c.Ceiling)
					}
					next = append(next, *a)
					extended = true
					break
				}
			}
			if extended {
				continue
			}
			r := Region{
				ID:        regionID(tileIndex, len(regions)),
				Tile:      tileIndex,
				MinX:      gx0 + x0,
				MinY:      gy0 + y,
				MaxX:      gx0 + x1 + 1,
				MaxY:      gy0 + y + 1,
				Cost:      cls.cost,
				Forbidden: cls.forbidden,
				Clearance: math.Inf(1),
			}
			a := active{x0: x0, x1: x1, class: cls, region: len(regions), clearance: math.Inf(1), ceiling: math.Inf(1)}
			for lx := x0; lx <= x1; lx++ {
				regionOf[y*w+lx] = int32(a.region)
				c := cells[y*w+lx]
				if c.Clearance < a.clearance {
					a.clearance = c.Clearance
				}
				a.ceiling = minCeiling(a.ceiling, c.Ceiling)
			}
			regions = append(regions, r)
			next = append(next, a)
		}
		// Close actives not extended into this row.
		for ai := range actives {
			a := actives[ai]
			still := false
			for ni := range next {
				if next[ni].region == a.region {
					still = true
					break
				}
			}
			if !still {
				finish(&a, y)
			}
		}
		actives = next
	}
	for ai := range actives {
		finish(&actives[ai], h)
	}
	for i := range cells {
		if regionOf[i] >= 0 {
			cells[i].Region = regionID(tileIndex, int(regionOf[i]))
		} else {
			cells[i].Region = -1
		}
	}
	return regions, regionOf
}

func minCeiling(a, b float64) float64 {
	if a == 0 {
		a = math.Inf(1)
	}
	if b == 0 {
		b = math.Inf(1)
	}
	return math.Min(a, b)
}

// regionID packs a tile index and a tile-local region index into a
// globally unique, version-stable region ID.
func regionID(tile, local int) int64 {
	return int64(tile)<<32 | int64(uint32(local))
}

// assemble builds the global region table, its lookup index and the
// adjacency lists. It never mutates tiles, so unchanged tiles can be
// shared freely between versions.
func (m *NavMesh) assemble() {
	var regions []Region
	for _, t := range m.Tiles {
		regions = append(regions, t.Regions...)
	}
	m.Regions = regions
	m.RegionIndex = make(map[int64]int32, len(regions))
	for i := range regions {
		m.RegionIndex[regions[i].ID] = int32(i)
	}
	// Adjacency: scan every interior grid edge; two walkable cells with
	// different region IDs make their regions adjacent.
	adj := make([]map[int32]struct{}, len(regions))
	add := func(a, b int32) {
		if adj[a] == nil {
			adj[a] = map[int32]struct{}{}
		}
		if adj[b] == nil {
			adj[b] = map[int32]struct{}{}
		}
		adj[a][b] = struct{}{}
		adj[b][a] = struct{}{}
	}
	for cy := 0; cy < m.Rows; cy++ {
		for cx := 0; cx < m.Cols; cx++ {
			c, ok := m.CellAt(cx, cy)
			if !ok || c.Blocked || c.Region < 0 {
				continue
			}
			ci, ok := m.RegionIndex[c.Region]
			if !ok {
				continue
			}
			for _, nb := range [2][2]int{{cx + 1, cy}, {cx, cy + 1}} {
				n, ok := m.CellAt(nb[0], nb[1])
				if !ok || n.Blocked || n.Region < 0 || n.Region == c.Region {
					continue
				}
				ni, ok := m.RegionIndex[n.Region]
				if !ok {
					continue
				}
				add(ci, ni)
			}
		}
	}
	m.RegionAdj = make([][]int64, len(regions))
	for i := range regions {
		set := adj[i]
		list := make([]int64, 0, len(set))
		for idx := range set {
			list = append(list, regions[idx].ID)
		}
		sort.Slice(list, func(a, b int) bool { return list[a] < list[b] })
		m.RegionAdj[i] = list
		m.Regions[i].Neighbors = list
	}
}

// Checksum returns a deterministic hash of the whole mesh, suitable for
// reproducibility checks in tests and persistence validation.
func (m *NavMesh) Checksum() uint64 {
	h := fnvOffset
	h = fnvAdd(h, m.Cfg.Hash())
	h = fnvAdd(h, uint64(m.Cols))
	h = fnvAdd(h, uint64(m.Rows))
	h = fnvAdd(h, uint64(m.TileCols))
	for _, t := range m.Tiles {
		h = fnvAdd(h, t.Checksum)
	}
	return h
}

func tileChecksum(t *Tile) uint64 {
	h := fnvOffset
	h = fnvAdd(h, uint64(t.Index))
	for _, c := range t.Cells {
		var b uint64
		if c.Blocked {
			b |= 1
		}
		if c.Forbidden {
			b |= 2
		}
		h = fnvAdd(h, b)
		h = fnvAdd(h, mathFloatBits(c.Cost))
		h = fnvAdd(h, mathFloatBits(c.Ceiling))
		h = fnvAdd(h, mathFloatBits(c.Clearance))
		h = fnvAdd(h, uint64(int64(c.Region)))
	}
	for _, r := range t.Regions {
		h = fnvAdd(h, uint64(r.MinX))
		h = fnvAdd(h, uint64(r.MinY))
		h = fnvAdd(h, uint64(r.MaxX))
		h = fnvAdd(h, uint64(r.MaxY))
		h = fnvAdd(h, mathFloatBits(r.Cost))
	}
	return h
}

func ceilDiv(a, b int) int { return (a + b - 1) / b }

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}
