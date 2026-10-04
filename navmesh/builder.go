package navmesh

import (
	"math"
	"sort"
)

// Builder 从场景几何确定性地构建导航网格。
// 确定性来源：所有输入按 ID 排序后应用，扫描顺序固定，
// 不依赖 map 迭代顺序与 goroutine 调度。
type Builder struct {
	cfg Config
}

// NewBuilder 创建构建器。
func NewBuilder(cfg Config) *Builder {
	return &Builder{cfg: cfg.withDefaults()}
}

// worldCells 返回世界格子维度。
func (b *Builder) worldCells() (int, int) {
	cx := int(math.Ceil((b.cfg.WorldMax.X - b.cfg.WorldMin.X) / b.cfg.CellSize))
	cy := int(math.Ceil((b.cfg.WorldMax.Y - b.cfg.WorldMin.Y) / b.cfg.CellSize))
	if cx < 1 {
		cx = 1
	}
	if cy < 1 {
		cy = 1
	}
	return cx, cy
}

// tileBounds 返回 tile (tx,ty) 的格子范围与尺寸。
func (b *Builder) tileBounds(tx, ty int) (x0, y0, w, h int) {
	cx, cy := b.worldCells()
	tc := b.cfg.TileCells
	x0, y0 = tx*tc, ty*tc
	w = min(tc, cx-x0)
	h = min(tc, cy-y0)
	return
}

// cellAABB 返回世界格子 (cx,cy) 的世界空间 AABB。
func (b *Builder) cellAABB(cx, cy int) AABB {
	cs := b.cfg.CellSize
	return AABB{
		Min: Vec2{b.cfg.WorldMin.X + float64(cx)*cs, b.cfg.WorldMin.Y + float64(cy)*cs},
		Max: Vec2{b.cfg.WorldMin.X + float64(cx+1)*cs, b.cfg.WorldMin.Y + float64(cy+1)*cs},
	}
}

// overlapsStrict 判断两盒是否有正面积重叠（边界接触不算），
// 用于栅格化：格子只有与几何真正重叠才被影响。
func overlapsStrict(a, b AABB) bool {
	return a.Min.X < b.Max.X && a.Max.X > b.Min.X &&
		a.Min.Y < b.Max.Y && a.Max.Y > b.Min.Y
}

// sortGeometry 按 ID 稳定排序，保证应用顺序确定。
func sortGeometry(obstacles []Obstacle, zones []Zone) ([]Obstacle, []Zone) {
	obs := append([]Obstacle(nil), obstacles...)
	zns := append([]Zone(nil), zones...)
	sort.Slice(obs, func(i, j int) bool { return obs[i].ID < obs[j].ID })
	sort.Slice(zns, func(i, j int) bool { return zns[i].ID < zns[j].ID })
	return obs, zns
}

// BuildTile 构建单块 tile（增量更新与全量构建共用同一逻辑，保证结果一致）。
func (b *Builder) BuildTile(tx, ty int, obstacles []Obstacle, zones []Zone) *Tile {
	x0, y0, w, h := b.tileBounds(tx, ty)
	t := &Tile{X: tx, Y: ty, W: w, H: h, Cells: make([]Cell, w*h)}

	obs, zns := sortGeometry(obstacles, zones)

	for cy := 0; cy < h; cy++ {
		for cx := 0; cx < w; cx++ {
			box := b.cellAABB(x0+cx, y0+cy)
			c := Cell{Flags: flagWalkable, Cost: 1, CeilY: float32(math.Inf(1))}
			for _, o := range obs {
				if !overlapsStrict(box, o.Box) {
					continue
				}
				if o.MinY <= 0 {
					// 贴地障碍：完全阻挡。
					c.Flags &^= flagWalkable
				} else if float64(c.CeilY) > o.MinY {
					// 悬空障碍：压低净高。
					c.CeilY = float32(o.MinY)
				}
			}
			cost := 1.0
			for _, z := range zns {
				if !overlapsStrict(box, z.Box) {
					continue
				}
				if z.Forbidden {
					c.Flags |= flagForbidden
				}
				if z.Cost > 0 {
					cost *= z.Cost
				}
			}
			if cost > 255 {
				cost = 255
			}
			if cost < 1 {
				cost = 1
			}
			c.Cost = uint8(math.Round(cost))
			t.Cells[cy*w+cx] = c
		}
	}

	b.computeClearance(t)
	b.partitionRegions(t)
	return t
}

// computeClearance 用多源 BFS（8 邻域）计算每格到最近不可行走格的距离（单位：格）。
func (b *Builder) computeClearance(t *Tile) {
	const inf = math.MaxUint16
	for i := range t.Cells {
		if t.Cells[i].walkable() {
			t.Cells[i].Clearance = inf
		} else {
			t.Cells[i].Clearance = 0
		}
	}
	// 多源 BFS，队列用环形切片，顺序固定。
	queue := make([]int, 0, len(t.Cells))
	for i, c := range t.Cells {
		if !c.walkable() {
			queue = append(queue, i)
		}
	}
	dirs := [8][2]int{{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}
	for head := 0; head < len(queue); head++ {
		i := queue[head]
		x, y := i%t.W, i/t.W
		d := t.Cells[i].Clearance + 1
		for _, dd := range dirs {
			nx, ny := x+dd[0], y+dd[1]
			nc := t.cellAt(nx, ny)
			if nc == nil || !nc.walkable() {
				continue
			}
			if nc.Clearance > d {
				nc.Clearance = d
				queue = append(queue, ny*t.W+nx)
			}
		}
	}
}

// partitionRegions 将可行走格子贪心切分为轴对齐矩形（凸区域），并建立相邻关系。
// 扫描顺序固定（行优先），结果确定。
func (b *Builder) partitionRegions(t *Tile) {
	used := make([]bool, len(t.Cells))
	walk := func(x, y int) bool {
		c := t.cellAt(x, y)
		return c != nil && c.walkable() && !used[y*t.W+x]
	}
	for y := 0; y < t.H; y++ {
		for x := 0; x < t.W; x++ {
			if !walk(x, y) {
				continue
			}
			// 向右扩展宽度。
			w := 0
			for walk(x+w, y) {
				w++
			}
			// 向下扩展高度（整行都可用）。
			h := 1
			for {
				ok := true
				for dx := 0; dx < w; dx++ {
					if !walk(x+dx, y+h) {
						ok = false
						break
					}
				}
				if !ok {
					break
				}
				h++
			}
			for dy := 0; dy < h; dy++ {
				for dx := 0; dx < w; dx++ {
					used[(y+dy)*t.W+x+dx] = true
				}
			}
			t.Regions = append(t.Regions, Region{X: x, Y: y, W: w, H: h})
		}
	}
	// 相邻关系：两矩形在 x 或 y 方向上贴边且另一方向区间重叠。
	n := len(t.Regions)
	t.Adj = make([][]int32, n)
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			if regionsAdjacent(t.Regions[i], t.Regions[j]) {
				t.Adj[i] = append(t.Adj[i], int32(j))
				t.Adj[j] = append(t.Adj[j], int32(i))
			}
		}
	}
}

// regionsAdjacent 判断两个矩形区域是否共享一段边界。
func regionsAdjacent(a, b Region) bool {
	overlapX := a.X < b.X+b.W && b.X < a.X+a.W
	overlapY := a.Y < b.Y+b.H && b.Y < a.Y+a.H
	touchX := a.X+a.W == b.X || b.X+b.W == a.X
	touchY := a.Y+a.H == b.Y || b.Y+b.H == a.Y
	return (touchX && overlapY) || (touchY && overlapX)
}

// BuildAll 全量构建整个世界的导航网格。
func (b *Builder) BuildAll(obstacles []Obstacle, zones []Zone) *NavMesh {
	cx, cy := b.worldCells()
	tc := b.cfg.TileCells
	tx := (cx + tc - 1) / tc
	ty := (cy + tc - 1) / tc
	m := &NavMesh{
		Config: b.cfg,
		Tiles:  make(map[TileKey]*Tile, tx*ty),
		cellsX: cx,
		cellsY: cy,
	}
	for y := 0; y < ty; y++ {
		for x := 0; x < tx; x++ {
			m.Tiles[TileKey{x, y}] = b.BuildTile(x, y, obstacles, zones)
		}
	}
	return m
}
