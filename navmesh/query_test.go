package navmesh

import (
	"context"
	"testing"
	"time"
)

// 不同尺寸代理共享同一份网格：小代理能穿过缺口，大代理不能。
func TestMultiSizeAgents(t *testing.T) {
	obs := []Obstacle{
		{ID: 1, Box: AABB{Vec2{0, 32}, Vec2{30, 34}}, MinY: 0, MaxY: 3},
		{ID: 2, Box: AABB{Vec2{34, 32}, Vec2{64, 34}}, MinY: 0, MaxY: 3},
	}
	m := NewBuilder(testConfig()).BuildAll(obs, nil)
	req := PathRequest{Start: Vec2{32, 16}, Goal: Vec2{32, 48}}

	small := FindPath(context.Background(), m, PathRequest{Start: req.Start, Goal: req.Goal, Agent: AgentProfile{Radius: 1.5, Height: 1}})
	if small.Reason != ReasonNone {
		t.Fatalf("small agent should pass the gap, got %v", small.Reason)
	}
	large := FindPath(context.Background(), m, PathRequest{Start: req.Start, Goal: req.Goal, Agent: AgentProfile{Radius: 3, Height: 1}})
	if large.Reason != ReasonNoConnection {
		t.Fatalf("large agent should not fit the gap, got %v", large.Reason)
	}
	if !large.Partial {
		t.Fatal("large agent should get a partial path to the closest reachable point")
	}
	// 小代理的路径不被大代理查询污染：再查一次结果一致。
	again := FindPath(context.Background(), m, PathRequest{Start: req.Start, Goal: req.Goal, Agent: AgentProfile{Radius: 1.5, Height: 1}})
	if again.Reason != ReasonNone || again.Cost != small.Cost {
		t.Fatal("queries of different sizes polluted each other")
	}
}

// 高度过滤：悬空障碍只挡高代理。
func TestAgentHeight(t *testing.T) {
	obs := []Obstacle{
		{ID: 1, Box: AABB{Vec2{0, 30}, Vec2{64, 34}}, MinY: 3, MaxY: 5}, // 悬空，净高 3
	}
	m := NewBuilder(testConfig()).BuildAll(obs, nil)
	short := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{32, 16}, Goal: Vec2{32, 48}, Agent: AgentProfile{Radius: 0, Height: 2},
	})
	if short.Reason != ReasonNone {
		t.Fatalf("short agent should pass under, got %v", short.Reason)
	}
	tall := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{32, 16}, Goal: Vec2{32, 48}, Agent: AgentProfile{Radius: 0, Height: 4},
	})
	if tall.Reason != ReasonNoConnection {
		t.Fatalf("tall agent should be blocked, got %v", tall.Reason)
	}
}

// 任何情况下不得返回穿越禁行区域的路径。
func TestForbiddenNeverCrossed(t *testing.T) {
	zones := []Zone{
		{ID: 1, Box: AABB{Vec2{16, 16}, Vec2{48, 48}}, Forbidden: true},
	}
	m := NewBuilder(testConfig()).BuildAll(nil, zones)
	res := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{8, 32}, Goal: Vec2{56, 32}, Agent: AgentProfile{Radius: 0.5},
	})
	if res.Reason != ReasonNone {
		t.Fatalf("expected detour around forbidden zone, got %v", res.Reason)
	}
	for _, p := range res.Points {
		c, _, _, _ := m.cellAt(p)
		if c != nil && c.Flags&flagForbidden != 0 {
			t.Fatalf("path crosses forbidden cell at %v", p)
		}
	}
}

// 不可达原因的区分。
func TestUnreachableReasons(t *testing.T) {
	obs := []Obstacle{
		{ID: 1, Box: AABB{Vec2{0, 32}, Vec2{64, 34}}, MinY: 0, MaxY: 3}, // 横断墙
		{ID: 2, Box: AABB{Vec2{10, 10}, Vec2{14, 14}}, MinY: 0, MaxY: 3},
	}
	m := NewBuilder(testConfig()).BuildAll(obs, nil)

	// 不连通：起终点合法但被墙隔开。
	r1 := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{8, 8}, Goal: Vec2{8, 56}, Agent: AgentProfile{},
	})
	if r1.Reason != ReasonNoConnection {
		t.Fatalf("expected NoConnection, got %v", r1.Reason)
	}
	if !r1.Partial || len(r1.Points) == 0 {
		t.Fatal("expected partial path toward unreachable goal")
	}

	// 起点在障碍内且附近无可投影点（被障碍完全包围不可行，这里起点在界外）。
	r2 := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{-5, 8}, Goal: Vec2{8, 8}, Agent: AgentProfile{},
	})
	if r2.Reason != ReasonStartOffMesh {
		t.Fatalf("expected StartOffMesh, got %v", r2.Reason)
	}

	// 终点在界外。
	r3 := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{8, 8}, Goal: Vec2{100, 100}, Agent: AgentProfile{},
	})
	if r3.Reason != ReasonGoalOffMesh {
		t.Fatalf("expected GoalOffMesh, got %v", r3.Reason)
	}

	// 终点在障碍内：投影到附近可通行格，返回部分路径。
	r4 := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{8, 20}, Goal: Vec2{12, 12}, Agent: AgentProfile{},
	})
	if r4.Reason != ReasonNone || !r4.Partial {
		t.Fatalf("expected partial success for goal inside obstacle, got %v partial=%v", r4.Reason, r4.Partial)
	}
}

// 取消与超时必须中断查询且不留下悬挂状态。
func TestCancelAndTimeout(t *testing.T) {
	m := NewBuilder(testConfig()).BuildAll(nil, nil)
	req := PathRequest{Start: Vec2{1, 1}, Goal: Vec2{63, 63}, Agent: AgentProfile{}}

	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	r1 := FindPath(ctx, m, req)
	if r1.Reason != ReasonCanceled {
		t.Fatalf("expected Canceled, got %v", r1.Reason)
	}

	ctx2, cancel2 := context.WithTimeout(context.Background(), -time.Second)
	defer cancel2()
	r2 := FindPath(ctx2, m, req)
	if r2.Reason != ReasonTimeout {
		t.Fatalf("expected Timeout, got %v", r2.Reason)
	}
}

// 确定性：同一快照同一请求，结果逐点一致。
func TestQueryDeterministic(t *testing.T) {
	obs, zones := testGeometry()
	m := NewBuilder(testConfig()).BuildAll(obs, zones)
	req := PathRequest{Start: Vec2{2, 2}, Goal: Vec2{62, 62}, Agent: AgentProfile{Radius: 1}}
	a := FindPath(context.Background(), m, req)
	b := FindPath(context.Background(), m, req)
	if a.Reason != b.Reason || a.Cost != b.Cost || len(a.Points) != len(b.Points) {
		t.Fatal("same query produced different results")
	}
	for i := range a.Points {
		if a.Points[i] != b.Points[i] {
			t.Fatal("path points differ")
		}
	}
}

// 扩展节点上限：超限查询失败但不影响其它查询。
func TestNodeLimit(t *testing.T) {
	cfg := testConfig()
	cfg.MaxExpandedNodes = 10
	m := NewBuilder(cfg).BuildAll(nil, nil)
	res := FindPath(context.Background(), m, PathRequest{
		Start: Vec2{1, 1}, Goal: Vec2{63, 63}, Agent: AgentProfile{},
	})
	if res.Reason != ReasonNodeLimit {
		t.Fatalf("expected NodeLimit, got %v", res.Reason)
	}
}

// 队列背压：worker 与队列都满时立即失败，不阻塞。
func TestQueueFull(t *testing.T) {
	cfg := testConfig()
	cfg.QueryWorkers = 1
	cfg.QueryQueueSize = 1
	rt := NewRuntime(cfg)
	defer rt.Close()
	rt.Rebuild(nil, nil)

	// 占住唯一的 worker。
	rt.sem <- struct{}{}
	defer func() { <-rt.sem }()
	// 占住唯一的排队位。
	rt.waiters.Add(1)
	defer rt.waiters.Add(-1)

	res, err := rt.FindPath(context.Background(), PathRequest{
		Start: Vec2{1, 1}, Goal: Vec2{63, 63}, Agent: AgentProfile{},
	})
	if err != nil {
		t.Fatal(err)
	}
	if res.Reason != ReasonQueueFull {
		t.Fatalf("expected QueueFull, got %v", res.Reason)
	}
}
