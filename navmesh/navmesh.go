package navmesh

import "math"

// TileKey 为 tile 坐标键。
type TileKey struct {
	X, Y int
}

// NavMesh 为一份不可变的导航数据快照。
// 更新产生新的 NavMesh 版本，旧版本仍可被进行中的查询安全读取。
type NavMesh struct {
	Version uint64
	Config  Config
	Tiles   map[TileKey]*Tile

	// 世界格子维度（由 Config 推出）。
	cellsX, cellsY int
}

// tileCoord 将世界坐标转换为 tile 键与 tile 内格子坐标。
// 越界时 ok=false。
func (m *NavMesh) tileCoord(p Vec2) (key TileKey, cx, cy int, ok bool) {
	cs := m.Config.CellSize
	gx := int(math.Floor((p.X - m.Config.WorldMin.X) / cs))
	gy := int(math.Floor((p.Y - m.Config.WorldMin.Y) / cs))
	if gx < 0 || gy < 0 || gx >= m.cellsX || gy >= m.cellsY {
		return TileKey{}, 0, 0, false
	}
	tc := m.Config.TileCells
	return TileKey{gx / tc, gy / tc}, gx % tc, gy % tc, true
}

// TileAt 返回世界坐标所在的 tile，越界返回 nil。
func (m *NavMesh) TileAt(p Vec2) *Tile {
	key, _, _, ok := m.tileCoord(p)
	if !ok {
		return nil
	}
	return m.Tiles[key]
}

// cellAt 返回世界坐标处的格子，越界返回 nil。
func (m *NavMesh) cellAt(p Vec2) (*Cell, TileKey, int, int) {
	key, cx, cy, ok := m.tileCoord(p)
	if !ok {
		return nil, key, 0, 0
	}
	t := m.Tiles[key]
	if t == nil {
		return nil, key, 0, 0
	}
	return t.cellAt(cx, cy), key, cx, cy
}

// cellCenter 返回 tile key 内格子 (cx,cy) 的世界中心坐标。
func (m *NavMesh) cellCenter(key TileKey, cx, cy int) Vec2 {
	cs := m.Config.CellSize
	tc := m.Config.TileCells
	gx := key.X*tc + cx
	gy := key.Y*tc + cy
	return Vec2{
		m.Config.WorldMin.X + (float64(gx)+0.5)*cs,
		m.Config.WorldMin.Y + (float64(gy)+0.5)*cs,
	}
}

// globalCell 为跨 tile 的全局格子坐标。
type globalCell struct {
	X, Y int
}

// globalCoord 返回世界坐标对应的全局格子坐标，越界 ok=false。
func (m *NavMesh) globalCoord(p Vec2) (globalCell, bool) {
	cs := m.Config.CellSize
	gx := int(math.Floor((p.X - m.Config.WorldMin.X) / cs))
	gy := int(math.Floor((p.Y - m.Config.WorldMin.Y) / cs))
	if gx < 0 || gy < 0 || gx >= m.cellsX || gy >= m.cellsY {
		return globalCell{}, false
	}
	return globalCell{gx, gy}, true
}

// cellAtGlobal 返回全局格子坐标处的格子，越界返回 nil。
func (m *NavMesh) cellAtGlobal(g globalCell) *Cell {
	tc := m.Config.TileCells
	t := m.Tiles[TileKey{g.X / tc, g.Y / tc}]
	if t == nil {
		return nil
	}
	return t.cellAt(g.X%tc, g.Y%tc)
}

// globalCenter 返回全局格子的世界中心坐标。
func (m *NavMesh) globalCenter(g globalCell) Vec2 {
	cs := m.Config.CellSize
	return Vec2{
		m.Config.WorldMin.X + (float64(g.X)+0.5)*cs,
		m.Config.WorldMin.Y + (float64(g.Y)+0.5)*cs,
	}
}
