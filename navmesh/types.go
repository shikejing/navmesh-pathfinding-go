// Package navmesh 提供确定性的导航网格构建、增量更新与多代理并发寻路。
package navmesh

// Vec2 为二维平面坐标（世界单位）。
type Vec2 struct {
	X, Y float64
}

// AABB 为轴对齐包围盒。
type AABB struct {
	Min, Max Vec2
}

// Intersects 判断两个 AABB 是否相交（含边界接触）。
func (a AABB) Intersects(b AABB) bool {
	return a.Min.X <= b.Max.X && a.Max.X >= b.Min.X &&
		a.Min.Y <= b.Max.Y && a.Max.Y >= b.Min.Y
}

// Contains 判断点是否在盒内。
func (a AABB) Contains(p Vec2) bool {
	return p.X >= a.Min.X && p.X <= a.Max.X && p.Y >= a.Min.Y && p.Y <= a.Max.Y
}

// AgentProfile 描述一个代理的物理尺寸。
// Radius 为水平半径，Height 为垂直高度，二者共同决定可通行区域。
type AgentProfile struct {
	Radius float64
	Height float64
}

// Obstacle 为阻挡几何：水平 footprint + 垂直区间 [MinY, MaxY]。
// 代理的垂直区间 [baseY, baseY+Height] 与之重叠时被阻挡。
type Obstacle struct {
	ID         uint64
	Box        AABB
	MinY, MaxY float64
}

// Zone 为区域修饰：为覆盖到的格子设置代价与禁行标记。
type Zone struct {
	ID        uint64
	Box       AABB
	Cost      float64 // 进入代价倍率，<=0 视为 1
	Forbidden bool
}

// Config 为构建与运行配置。同一份几何 + 同一份 Config 必须产生完全一致的网格。
type Config struct {
	WorldMin, WorldMax Vec2 // 世界范围
	CellSize           float64
	TileCells          int // 每块 tile 的边长（格子数）

	QueryWorkers     int // 并发查询 worker 数，<=0 取默认值
	QueryQueueSize   int // 查询排队上限，<=0 取默认值
	MaxExpandedNodes int // 单次查询 A* 扩展上限，<=0 取默认值
}

// withDefaults 填充默认值，返回规范化后的配置。
func (c Config) withDefaults() Config {
	if c.CellSize <= 0 {
		c.CellSize = 1
	}
	if c.TileCells <= 0 {
		c.TileCells = 32
	}
	if c.QueryWorkers <= 0 {
		c.QueryWorkers = 4
	}
	if c.QueryQueueSize <= 0 {
		c.QueryQueueSize = 1024
	}
	if c.MaxExpandedNodes <= 0 {
		c.MaxExpandedNodes = 200000
	}
	return c
}
