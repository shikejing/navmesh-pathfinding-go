package navmesh

import (
	"fmt"
	"sort"
)

// Diff describes what changed between two mesh versions, so callers can
// explain path differences across an update.
type Diff struct {
	FromVersion, ToVersion uint64
	// ChangedTiles lists tile indices whose contents actually changed.
	ChangedTiles []int
	// RemovedRegions lists region IDs of the old version that lived in
	// changed tiles; AddedRegions lists their replacements in the new
	// version. Region IDs are version-stable, so an ID present in both
	// versions denotes the same tile-local rectangle.
	RemovedRegions []int64
	AddedRegions   []int64
}

// Update returns a new mesh version reflecting zone additions/replacements
// (upserts) and removals. Only tiles intersecting changed zones (plus the
// clearance halo around them) are rebuilt; unchanged tiles are shared with
// the receiver. The receiver is not modified.
func (m *NavMesh) Update(upserts []Zone, removeIDs []string) (*NavMesh, Diff, error) {
	remove := make(map[string]struct{}, len(removeIDs))
	for _, id := range removeIDs {
		remove[id] = struct{}{}
	}
	byID := make(map[string]Zone, len(m.Scene.Zones)+len(upserts))
	for _, z := range m.Scene.Zones {
		byID[z.ID] = z
	}
	changed := make(map[string]struct{})
	for _, id := range removeIDs {
		if _, ok := byID[id]; !ok {
			return nil, Diff{}, fmt.Errorf("navmesh: cannot remove unknown zone %q", id)
		}
		changed[id] = struct{}{}
		delete(byID, id)
	}
	for _, z := range upserts {
		if z.Kind == ZoneCost && z.Cost < 1 {
			z.Cost = 1
		}
		if old, ok := byID[z.ID]; ok && old == z {
			continue // no-op upsert
		}
		changed[z.ID] = struct{}{}
		byID[z.ID] = z
	}
	if len(changed) == 0 {
		return m, Diff{FromVersion: m.Version, ToVersion: m.Version}, nil
	}
	zones := make([]Zone, 0, len(byID))
	for _, z := range byID {
		zones = append(zones, z)
	}
	sort.Slice(zones, func(i, j int) bool { return zones[i].ID < zones[j].ID })

	// Dirty tiles: anything touched by the old or new shape of a changed
	// zone, expanded by the clearance halo.
	oldByID := make(map[string]Zone, len(m.Scene.Zones))
	for _, z := range m.Scene.Zones {
		oldByID[z.ID] = z
	}
	halo := haloCells(m)
	dirty := make(map[int]struct{})
	markRect := func(r Rect) {
		cx0, cy0 := m.WorldToCell(Vec2{X: r.MinX, Y: r.MinY})
		cx1, cy1 := m.WorldToCell(Vec2{X: r.MaxX, Y: r.MaxY})
		cx0, cy0 = cx0-halo, cy0-halo
		cx1, cy1 = cx1+halo, cy1+halo
		// Negative cell coords need floor division.
		tx0 := maxInt(0, floorDiv(cx0, m.TileCols))
		ty0 := maxInt(0, floorDiv(cy0, m.TileCols))
		tx1 := minInt(m.TilesX-1, floorDiv(cx1, m.TileCols))
		ty1 := minInt(m.TilesY-1, floorDiv(cy1, m.TileCols))
		for ty := ty0; ty <= ty1; ty++ {
			for tx := tx0; tx <= tx1; tx++ {
				dirty[ty*m.TilesX+tx] = struct{}{}
			}
		}
	}
	for id := range changed {
		if old, ok := oldByID[id]; ok {
			markRect(old.Rect)
		}
		if nz, ok := byID[id]; ok {
			markRect(nz.Rect)
		}
	}

	// Rasterize rebuilt tiles with the new zone set.
	raw := make(map[int][]Cell, len(dirty))
	for i := range dirty {
		t := m.Tiles[i]
		raw[i] = rasterizeTile(m, zones, t.TX, t.TY)
	}
	get := func(cx, cy int) (Cell, bool) {
		if cx < 0 || cy < 0 || cx >= m.Cols || cy >= m.Rows {
			return Cell{}, false
		}
		tx, ty := cx/m.TileCols, cy/m.TileCols
		i := ty*m.TilesX + tx
		if cells, ok := raw[i]; ok {
			w := tileWidth(m, tx)
			return cells[(cy-ty*m.TileCols)*w+(cx-tx*m.TileCols)], true
		}
		return m.CellAt(cx, cy)
	}

	nm := &NavMesh{
		Cfg:         m.Cfg,
		Scene:       Scene{Bounds: m.Scene.Bounds, Zones: zones},
		Version:     m.Version + 1,
		Cols:        m.Cols,
		Rows:        m.Rows,
		TileCols:    m.TileCols,
		TilesX:      m.TilesX,
		TilesY:      m.TilesY,
		WorldBounds: m.WorldBounds,
		Tiles:       make([]*Tile, len(m.Tiles)),
	}
	for i, t := range m.Tiles {
		cells, ok := raw[i]
		if !ok {
			nm.Tiles[i] = t // shared, untouched
			continue
		}
		fillClearance(m, get, t.TX, t.TY, cells)
		regions, _ := decompose(m, i, t.TX, t.TY, cells)
		nt := &Tile{Index: i, TX: t.TX, TY: t.TY, Cells: cells, Regions: regions}
		nt.Checksum = tileChecksum(nt)
		nm.Tiles[i] = nt
	}
	nm.assemble()

	diff := Diff{FromVersion: m.Version, ToVersion: nm.Version}
	for i := range dirty {
		if nm.Tiles[i].Checksum == m.Tiles[i].Checksum {
			continue
		}
		diff.ChangedTiles = append(diff.ChangedTiles, i)
		for _, r := range m.Tiles[i].Regions {
			diff.RemovedRegions = append(diff.RemovedRegions, r.ID)
		}
		for _, r := range nm.Tiles[i].Regions {
			diff.AddedRegions = append(diff.AddedRegions, r.ID)
		}
	}
	sort.Ints(diff.ChangedTiles)
	sort.Slice(diff.RemovedRegions, func(a, b int) bool { return diff.RemovedRegions[a] < diff.RemovedRegions[b] })
	sort.Slice(diff.AddedRegions, func(a, b int) bool { return diff.AddedRegions[a] < diff.AddedRegions[b] })
	return nm, diff, nil
}

func floorDiv(a, b int) int {
	q := a / b
	if a%b != 0 && a < 0 {
		q--
	}
	return q
}
