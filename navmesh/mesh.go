package navmesh

// Cell is one raster cell of the navigation grid.
type Cell struct {
	// Blocked is true when an obstacle zone covers the cell.
	Blocked bool
	// Cost is the traversal cost multiplier (>= 1) for walkable cells.
	Cost float64
	// Forbidden marks a walkable cell as never traversable.
	Forbidden bool
	// Ceiling is the passable height of the cell; 0 means unlimited.
	Ceiling float64
	// Clearance is the distance (in world units) from the cell centre to
	// the nearest blocked cell or map edge.
	Clearance float64
	// Region is the global ID of the region the cell belongs to, or -1.
	Region int64
}

// Region is a rectangular group of walkable cells. Concave walkable areas
// are split into multiple regions linked by adjacency.
type Region struct {
	// ID is globally unique and stable across versions:
	// tileIndex<<32 | localIndexWithinTile.
	ID int64
	// Tile is the index of the tile this region was built from.
	Tile int
	// Rect is the cell-space rectangle covered by the region
	// (MinX/MinY inclusive, MaxX/MaxY exclusive), in global cell coords.
	MinX, MinY, MaxX, MaxY int
	Cost                   float64
	Forbidden              bool
	Clearance              float64
	Ceiling                float64
	// Neighbors holds adjacent region IDs (may cross tile boundaries),
	// sorted ascending for determinism.
	Neighbors []int64
}

// Tile is the unit of incremental rebuild. Tiles are immutable once built
// and shared between mesh versions.
type Tile struct {
	Index    int
	TX, TY   int // tile coordinates
	Cells    []Cell
	Regions  []Region // local regions; Region.ID is global
	Checksum uint64
}

// NavMesh is an immutable, versioned navigation mesh. All slices must be
// treated as read-only; updates produce a new NavMesh sharing unchanged
// tiles.
type NavMesh struct {
	Cfg         Config
	Scene       Scene // zone snapshot used to build this version
	Version     uint64
	Cols, Rows  int // grid dimensions in cells
	TileCols    int // tile edge in cells
	TilesX      int
	TilesY      int
	Tiles       []*Tile
	Regions     []Region // global region table sorted by ID
	RegionAdj   [][]int64
	RegionIndex map[int64]int32 // region ID -> position in Regions
	CellCols    int             // == Cols, kept for clarity in indexing
	WorldBounds Rect
}

// CellAt returns the cell at global cell coordinates, and false when out
// of bounds.
func (m *NavMesh) CellAt(cx, cy int) (Cell, bool) {
	if cx < 0 || cy < 0 || cx >= m.Cols || cy >= m.Rows {
		return Cell{}, false
	}
	tx, ty := cx/m.TileCols, cy/m.TileCols
	t := m.Tiles[ty*m.TilesX+tx]
	lx, ly := cx-tx*m.TileCols, cy-ty*m.TileCols
	w := tileWidth(m, tx)
	return t.Cells[ly*w+lx], true
}

// WorldToCell converts world coordinates to global cell coordinates.
// The result may be out of range; callers must check with CellAt.
func (m *NavMesh) WorldToCell(p Vec2) (int, int) {
	cs := m.Cfg.CellSize
	return int((p.X - m.WorldBounds.MinX) / cs), int((p.Y - m.WorldBounds.MinY) / cs)
}

// CellCenter returns the world-space centre of a cell.
func (m *NavMesh) CellCenter(cx, cy int) Vec2 {
	cs := m.Cfg.CellSize
	return Vec2{
		X: m.WorldBounds.MinX + (float64(cx)+0.5)*cs,
		Y: m.WorldBounds.MinY + (float64(cy)+0.5)*cs,
	}
}

// tileWidth returns the width in cells of tile column tx (edge tiles may
// be narrower).
func tileWidth(m *NavMesh, tx int) int {
	w := m.TileCols
	if rem := m.Cols - tx*m.TileCols; rem < w {
		w = rem
	}
	return w
}

// tileHeight returns the height in cells of tile row ty.
func tileHeight(m *NavMesh, ty int) int {
	h := m.TileCols
	if rem := m.Rows - ty*m.TileCols; rem < h {
		h = rem
	}
	return h
}
