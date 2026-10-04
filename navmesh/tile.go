package navmesh

// 格子标记位。
const (
	flagWalkable  uint8 = 1 << iota // 基础几何上可行走
	flagForbidden                   // 禁行区域（Zone 标记）
)

// Cell 为导航网格的最小单元。
type Cell struct {
	Flags     uint8   // flagWalkable / flagForbidden
	Cost      uint8   // 进入代价（1..255，1 为基准）
	Clearance uint16  // 到最近障碍的水平距离，单位：格子（饱和）
	CeilY     float32 // 该格子上方最低障碍底面；+Inf 表示无遮挡
}

// walkable 报告格子是否在基础几何上可行走。
func (c Cell) walkable() bool { return c.Flags&flagWalkable != 0 }

// Region 为 tile 内的一个凸可行走区域（轴对齐矩形，格子坐标）。
type Region struct {
	X, Y, W, H int
}

// Tile 为一块固定大小的导航数据，是增量更新的最小单位。
type Tile struct {
	X, Y    int       // tile 坐标
	W, H    int       // 实际格子数（边缘 tile 可能不足 TileCells）
	Cells   []Cell    // 行优先，长度 W*H
	Regions []Region  // 贪心切分出的凸区域
	Adj     [][]int32 // 与 Regions 等长的相邻表
}

// cellIndex 返回格子 (x,y) 在 Cells 中的下标。
func (t *Tile) cellIndex(x, y int) int { return y*t.W + x }

// cellAt 返回格子指针，越界返回 nil。
func (t *Tile) cellAt(x, y int) *Cell {
	if x < 0 || y < 0 || x >= t.W || y >= t.H {
		return nil
	}
	return &t.Cells[y*t.W+x]
}
