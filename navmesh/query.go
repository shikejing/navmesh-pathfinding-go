package navmesh

import (
	"container/heap"
	"context"
	"math"
)

// AgentProfile describes the physical size of an agent. Radius and Height
// select which regions are traversable for the query; agents of different
// sizes share the same mesh without rebuilding it.
type AgentProfile struct {
	Radius float64
	Height float64 // 0 means unbounded
}

// UnreachableReason distinguishes failure causes of a query.
type UnreachableReason int

const (
	ReasonNone UnreachableReason = iota
	// ReasonOutOfBounds: start or goal lies outside the world bounds.
	ReasonOutOfBounds
	// ReasonAgentTooLarge: the agent radius exceeds MaxAgentRadius, so
	// clearance data cannot answer the query.
	ReasonAgentTooLarge
	// ReasonStartBlocked: the start cell is not traversable for this agent.
	ReasonStartBlocked
	// ReasonGoalBlocked: the goal cell is not traversable for this agent.
	ReasonGoalBlocked
	// ReasonNoPath: both endpoints are traversable but no path connects
	// them (disconnected or forbidden-only regions in between).
	ReasonNoPath
	// ReasonCancelled: the query context was cancelled or timed out.
	ReasonCancelled
	// ReasonExpansionLimit: the configured expansion budget was exhausted.
	ReasonExpansionLimit
)

func (r UnreachableReason) String() string {
	switch r {
	case ReasonNone:
		return "none"
	case ReasonOutOfBounds:
		return "out_of_bounds"
	case ReasonAgentTooLarge:
		return "agent_too_large"
	case ReasonStartBlocked:
		return "start_blocked"
	case ReasonGoalBlocked:
		return "goal_blocked"
	case ReasonNoPath:
		return "no_path"
	case ReasonCancelled:
		return "cancelled"
	case ReasonExpansionLimit:
		return "expansion_limit"
	}
	return "unknown"
}

// Query is a single pathfinding request.
type Query struct {
	Start, Goal Vec2
	Agent       AgentProfile
}

// PathResult is the outcome of a query.
type PathResult struct {
	Found bool
	// Points are world-space waypoints from start to goal when Found.
	Points []Vec2
	// Cost is the total traversal cost when Found.
	Cost float64
	// Reason explains failure when Found is false.
	Reason UnreachableReason
	// Version is the mesh version the query ran against.
	Version uint64
}

// regionCenter returns the world-space centre of a region's rectangle.
func (m *NavMesh) regionCenter(r *Region) Vec2 {
	return Vec2{
		X: m.WorldBounds.MinX + (float64(r.MinX+r.MaxX)/2)*m.Cfg.CellSize,
		Y: m.WorldBounds.MinY + (float64(r.MinY+r.MaxY)/2)*m.Cfg.CellSize,
	}
}

// traversable reports whether the agent may enter the region.
func traversable(r *Region, a AgentProfile) bool {
	if r.Forbidden {
		return false
	}
	if r.Clearance < a.Radius {
		return false
	}
	if a.Height > 0 && r.Ceiling > 0 && r.Ceiling < a.Height {
		return false
	}
	return true
}

func (m *NavMesh) regionAt(p Vec2) (int64, bool) {
	cx, cy := m.WorldToCell(p)
	c, ok := m.CellAt(cx, cy)
	if !ok {
		return -1, false
	}
	if c.Blocked || c.Region < 0 {
		return -1, true
	}
	return c.Region, true
}

// FindPath runs an A* search over the region graph, filtered by the
// agent's size. It never returns a path crossing forbidden or blocked
// regions. It honours ctx cancellation.
func (m *NavMesh) FindPath(ctx context.Context, q Query) PathResult {
	res := PathResult{Version: m.Version}
	if q.Agent.Radius > m.Cfg.MaxAgentRadius {
		res.Reason = ReasonAgentTooLarge
		return res
	}
	startID, ok := m.regionAt(q.Start)
	if !ok {
		res.Reason = ReasonOutOfBounds
		return res
	}
	goalID, ok := m.regionAt(q.Goal)
	if !ok {
		res.Reason = ReasonOutOfBounds
		return res
	}
	if startID < 0 {
		res.Reason = ReasonStartBlocked
		return res
	}
	if goalID < 0 {
		res.Reason = ReasonGoalBlocked
		return res
	}
	start := m.RegionIndex[startID]
	goal := m.RegionIndex[goalID]
	if !traversable(&m.Regions[start], q.Agent) {
		res.Reason = ReasonStartBlocked
		return res
	}
	if !traversable(&m.Regions[goal], q.Agent) {
		res.Reason = ReasonGoalBlocked
		return res
	}
	if start == goal {
		res.Found = true
		res.Points = []Vec2{q.Start, q.Goal}
		res.Cost = dist(q.Start, q.Goal) * m.Regions[start].Cost
		return res
	}

	goalPos := m.regionCenter(&m.Regions[goal])
	n := len(m.Regions)
	g := make([]float64, n)
	came := make([]int32, n)
	closed := make([]bool, n)
	for i := range g {
		g[i] = math.Inf(1)
		came[i] = -1
	}
	g[start] = 0
	open := &pq{}
	heap.Push(open, pqItem{id: start, f: dist(m.regionCenter(&m.Regions[start]), goalPos)})

	maxExp := m.Cfg.MaxExpansions
	expansions := 0
	for open.Len() > 0 {
		if expansions&63 == 0 {
			if err := ctx.Err(); err != nil {
				res.Reason = ReasonCancelled
				return res
			}
		}
		if maxExp > 0 && expansions >= maxExp {
			res.Reason = ReasonExpansionLimit
			return res
		}
		expansions++
		cur := heap.Pop(open).(pqItem)
		if closed[cur.id] {
			continue
		}
		closed[cur.id] = true
		if cur.id == goal {
			break
		}
		curPos := m.regionCenter(&m.Regions[cur.id])
		for _, nbID := range m.RegionAdj[cur.id] {
			nb := m.RegionIndex[nbID]
			if closed[nb] {
				continue
			}
			nr := &m.Regions[nb]
			if !traversable(nr, q.Agent) {
				continue
			}
			cost := dist(curPos, m.regionCenter(nr)) * nr.Cost
			ng := g[cur.id] + cost
			if ng < g[nb] {
				g[nb] = ng
				came[nb] = cur.id
				heap.Push(open, pqItem{id: nb, f: ng + dist(m.regionCenter(nr), goalPos)})
			}
		}
	}
	if !closed[goal] {
		res.Reason = ReasonNoPath
		return res
	}
	// Reconstruct region path, then waypoints.
	var rev []int32
	for id := goal; id != -1; id = came[id] {
		rev = append(rev, id)
	}
	pts := make([]Vec2, 0, len(rev)+1)
	pts = append(pts, q.Start)
	for i := len(rev) - 2; i >= 1; i-- {
		pts = append(pts, m.regionCenter(&m.Regions[rev[i]]))
	}
	pts = append(pts, q.Goal)
	res.Found = true
	res.Points = pts
	res.Cost = g[goal]
	return res
}

func dist(a, b Vec2) float64 {
	return math.Hypot(a.X-b.X, a.Y-b.Y)
}

// pqItem orders the open set by (f, id) so ties break deterministically.
type pqItem struct {
	id int32
	f  float64
}

type pq []pqItem

func (p pq) Len() int { return len(p) }
func (p pq) Less(i, j int) bool {
	if p[i].f != p[j].f {
		return p[i].f < p[j].f
	}
	return p[i].id < p[j].id
}
func (p pq) Swap(i, j int)       { p[i], p[j] = p[j], p[i] }
func (p *pq) Push(x interface{}) { *p = append(*p, x.(pqItem)) }
func (p *pq) Pop() interface{} {
	old := *p
	n := len(old)
	it := old[n-1]
	*p = old[:n-1]
	return it
}
