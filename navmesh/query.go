package navmesh

import (
	"container/heap"
	"context"
	"math"
)

// FailReason 区分查询失败的原因。
type FailReason int

const (
	ReasonNone         FailReason = iota // 成功
	ReasonStartOffMesh                   // 起点不在可通行区域
	ReasonGoalOffMesh                    // 终点不在可通行区域且附近无可投影点
	ReasonNoConnection                   // 起终点各自合法但不连通（完全不可达）
	ReasonCanceled                       // 查询被取消
	ReasonTimeout                        // 查询超时
	ReasonNodeLimit                      // 超出扩展节点上限
	ReasonQueueFull                      // 查询队列已满
)

// String 返回原因的可读描述。
func (r FailReason) String() string {
	switch r {
	case ReasonNone:
		return "ok"
	case ReasonStartOffMesh:
		return "start is not on any walkable area for this agent size"
	case ReasonGoalOffMesh:
		return "goal is not on any walkable area for this agent size"
	case ReasonNoConnection:
		return "start and goal are in disconnected regions"
	case ReasonCanceled:
		return "query canceled"
	case ReasonTimeout:
		return "query timed out"
	case ReasonNodeLimit:
		return "expanded node limit exceeded"
	case ReasonQueueFull:
		return "query queue is full"
	}
	return "unknown"
}

// PathRequest 为一次寻路请求。
type PathRequest struct {
	Start, Goal Vec2
	Agent       AgentProfile
}

// PathResult 为寻路结果。
// Version 记录结果基于哪个数据版本，用于解释更新前后的路径差异。
type PathResult struct {
	Version uint64
	Points  []Vec2 // 路径点（世界坐标）；失败时为空
	Partial bool   // 终点不可达时返回到最近可达点的部分路径
	Reason  FailReason
	Cost    float64
}

// agentFilter 判定格子对特定尺寸代理是否可通行。
// 共享同一份网格数据，过滤发生在查询侧，互不污染。
type agentFilter struct {
	m           *NavMesh
	radiusCells uint16  // 所需净空（格）
	height      float64 // 所需净高
}

func newAgentFilter(m *NavMesh, a AgentProfile) agentFilter {
	rc := uint16(0)
	if a.Radius > 0 {
		rc = uint16(math.Ceil(a.Radius / m.Config.CellSize))
	}
	return agentFilter{m: m, radiusCells: rc, height: a.Height}
}

// passable 报告全局格子是否可通行。禁行格永不通过。
func (f agentFilter) passable(g globalCell) bool {
	c := f.m.cellAtGlobal(g)
	if c == nil || !c.walkable() || c.Flags&flagForbidden != 0 {
		return false
	}
	if c.Clearance < f.radiusCells {
		return false
	}
	if float64(c.CeilY) < f.height {
		return false
	}
	return true
}

// cost 返回进入格子的代价。
func (f agentFilter) cost(g globalCell) float64 {
	c := f.m.cellAtGlobal(g)
	if c == nil {
		return 0
	}
	return float64(c.Cost)
}

// 优先队列元素。比较键全序确定，保证结果可复现。
type openItem struct {
	cell globalCell
	f, h float64
	g    float64
}

type openHeap []openItem

func (o openHeap) Len() int { return len(o) }
func (o openHeap) Less(i, j int) bool {
	a, b := o[i], o[j]
	if a.f != b.f {
		return a.f < b.f
	}
	if a.h != b.h {
		return a.h < b.h
	}
	if a.cell.X != b.cell.X {
		return a.cell.X < b.cell.X
	}
	return a.cell.Y < b.cell.Y
}
func (o openHeap) Swap(i, j int)       { o[i], o[j] = o[j], o[i] }
func (o *openHeap) Push(x interface{}) { *o = append(*o, x.(openItem)) }
func (o *openHeap) Pop() interface{} {
	old := *o
	n := len(old)
	it := old[n-1]
	*o = old[:n-1]
	return it
}

var diagDirs = [8][2]int{{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}

// octile 启发函数（允许对角移动）。
func octile(a, b globalCell) float64 {
	dx := math.Abs(float64(a.X - b.X))
	dy := math.Abs(float64(a.Y - b.Y))
	return math.Max(dx, dy) + (math.Sqrt2-1)*math.Min(dx, dy)
}

// projectToMesh 在 maxR 格范围内螺旋搜索离 target 最近的可通行格。
func (f agentFilter) projectToMesh(target globalCell, maxR int) (globalCell, bool) {
	if f.passable(target) {
		return target, true
	}
	best := globalCell{}
	bestD := math.Inf(1)
	found := false
	for r := 1; r <= maxR; r++ {
		for dy := -r; dy <= r; dy++ {
			for dx := -r; dx <= r; dx++ {
				if max(abs(dx), abs(dy)) != r {
					continue
				}
				g := globalCell{target.X + dx, target.Y + dy}
				if !f.passable(g) {
					continue
				}
				d := float64(dx*dx + dy*dy)
				if d < bestD {
					bestD, best, found = d, g, true
				}
			}
		}
		if found {
			return best, true
		}
	}
	return globalCell{}, false
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// FindPath 在快照 m 上为指定尺寸的代理寻路。
// 只读取 m，不写共享状态，可并发调用；ctx 取消/超时会中断搜索。
func FindPath(ctx context.Context, m *NavMesh, req PathRequest) PathResult {
	res := PathResult{Version: m.Version}
	f := newAgentFilter(m, req.Agent)

	start, ok := m.globalCoord(req.Start)
	if !ok {
		res.Reason = ReasonStartOffMesh
		return res
	}
	goal, ok := m.globalCoord(req.Goal)
	if !ok {
		res.Reason = ReasonGoalOffMesh
		return res
	}

	// 起终点投影：不在可通行格上时寻找最近可通行格。
	maxProj := 8 + int(f.radiusCells)
	start, ok = f.projectToMesh(start, maxProj)
	if !ok {
		res.Reason = ReasonStartOffMesh
		return res
	}
	realGoal := goal
	goalProjected := false
	if !f.passable(realGoal) {
		realGoal, ok = f.projectToMesh(realGoal, maxProj)
		if !ok {
			res.Reason = ReasonGoalOffMesh
			return res
		}
		goalProjected = true
	}

	const ctxCheckInterval = 2048
	gScore := map[globalCell]float64{start: 0}
	cameFrom := make(map[globalCell]globalCell)
	closed := make(map[globalCell]struct{})
	open := &openHeap{{cell: start, f: octile(start, realGoal), h: octile(start, realGoal)}}
	heap.Init(open)

	best := start // 离终点最近的已扩展节点，用于部分路径
	bestH := octile(start, realGoal)
	expanded := 0
	limit := m.Config.MaxExpandedNodes

	for open.Len() > 0 {
		if expanded%ctxCheckInterval == 0 {
			if err := ctx.Err(); err != nil {
				if err == context.DeadlineExceeded {
					res.Reason = ReasonTimeout
				} else {
					res.Reason = ReasonCanceled
				}
				return res
			}
		}
		cur := heap.Pop(open).(openItem)
		if _, done := closed[cur.cell]; done {
			continue
		}
		if g, ok := gScore[cur.cell]; ok && cur.g > g {
			continue // 过期堆元素
		}
		closed[cur.cell] = struct{}{}
		expanded++
		if expanded > limit {
			res.Reason = ReasonNodeLimit
			return res
		}
		if cur.h < bestH {
			bestH, best = cur.h, cur.cell
		}
		if cur.cell == realGoal {
			res.Points = reconstruct(m, cameFrom, cur.cell, req, goalProjected)
			res.Cost = gScore[cur.cell]
			res.Partial = goalProjected
			return res
		}
		cx, cy := cur.cell.X, cur.cell.Y
		for _, d := range diagDirs {
			ng := globalCell{cx + d[0], cy + d[1]}
			if _, done := closed[ng]; done {
				continue
			}
			if !f.passable(ng) {
				continue
			}
			step := f.cost(ng)
			if d[0] != 0 && d[1] != 0 {
				// 禁止切角：两个正交邻居都必须可通行。
				if !f.passable(globalCell{cx + d[0], cy}) || !f.passable(globalCell{cx, cy + d[1]}) {
					continue
				}
				step *= math.Sqrt2
			}
			ng2 := gScore[cur.cell] + step
			if old, ok := gScore[ng]; ok && ng2 >= old {
				continue
			}
			gScore[ng] = ng2
			cameFrom[ng] = cur.cell
			h := octile(ng, realGoal)
			heap.Push(open, openItem{cell: ng, f: ng2 + h, h: h, g: ng2})
		}
	}

	// 终点合法但不连通：给出到最近可达点的部分路径。
	res.Reason = ReasonNoConnection
	if best != start {
		res.Points = reconstruct(m, cameFrom, best, req, true)
		res.Cost = gScore[best]
		res.Partial = true
	}
	return res
}

// reconstruct 回溯路径并压缩共线点。
func reconstruct(m *NavMesh, cameFrom map[globalCell]globalCell, end globalCell, req PathRequest, partial bool) []Vec2 {
	rev := []globalCell{end}
	for cur := end; ; {
		prev, ok := cameFrom[cur]
		if !ok {
			break
		}
		rev = append(rev, prev)
		cur = prev
	}
	// 反转并压缩共线段。
	cells := make([]globalCell, 0, len(rev))
	for i := len(rev) - 1; i >= 0; i-- {
		cells = append(cells, rev[i])
	}
	pts := make([]Vec2, 0, len(cells)+2)
	pts = append(pts, req.Start)
	for i := 0; i < len(cells); i++ {
		if i+2 < len(cells) {
			d1x := cells[i+1].X - cells[i].X
			d1y := cells[i+1].Y - cells[i].Y
			d2x := cells[i+2].X - cells[i+1].X
			d2y := cells[i+2].Y - cells[i+1].Y
			if d1x == d2x && d1y == d2y {
				continue // 共线，跳过中间点
			}
		}
		if i == 0 || i == len(cells)-1 {
			continue // 首尾用精确坐标代替
		}
		pts = append(pts, m.globalCenter(cells[i]))
	}
	if partial {
		pts = append(pts, m.globalCenter(cells[len(cells)-1]))
	} else {
		pts = append(pts, req.Goal)
	}
	return pts
}
