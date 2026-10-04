package navmesh

import (
	"context"
	"testing"
)

// corridorScene has a narrow vertical corridor (width 3, x in [30,33])
// connecting the bottom and top halves, plus a forbidden room.
func corridorScene() Scene {
	return Scene{
		Bounds: Rect{0, 0, 64, 64},
		Zones: []Zone{
			{ID: "wall-l", Kind: ZoneObstacle, Rect: Rect{0, 30, 30, 34}},
			{ID: "wall-r", Kind: ZoneObstacle, Rect: Rect{33, 30, 64, 34}},
			{ID: "keepout", Kind: ZoneCost, Rect: Rect{48, 48, 64, 64}, Forbidden: true},
		},
	}
}

func TestFindPathBasic(t *testing.T) {
	m := Build(corridorScene(), testConfig())
	res := m.FindPath(context.Background(), Query{
		Start: Vec2{8, 8}, Goal: Vec2{56, 8},
		Agent: AgentProfile{Radius: 0.5},
	})
	if !res.Found {
		t.Fatalf("expected path, got reason %v", res.Reason)
	}
	if res.Version != m.Version {
		t.Fatalf("result version %d != mesh version %d", res.Version, m.Version)
	}
	assertPathClean(t, m, res, AgentProfile{Radius: 0.5})
}

// assertPathClean verifies no waypoint sits in a blocked, forbidden or
// too-tight cell for the agent.
func assertPathClean(t *testing.T, m *NavMesh, res PathResult, a AgentProfile) {
	t.Helper()
	for _, p := range res.Points {
		c, ok := m.CellAt(m.WorldToCell(p))
		if !ok {
			t.Fatalf("waypoint %v out of bounds", p)
		}
		if c.Blocked || c.Forbidden {
			t.Fatalf("waypoint %v in blocked/forbidden cell", p)
		}
		if c.Region >= 0 && !traversable(&m.Regions[m.RegionIndex[c.Region]], a) {
			t.Fatalf("waypoint %v in non-traversable region for agent %+v", p, a)
		}
	}
}

func TestMultiSizeAgentsShareMesh(t *testing.T) {
	m := Build(corridorScene(), testConfig())
	small := m.FindPath(context.Background(), Query{
		Start: Vec2{31.5, 10}, Goal: Vec2{31.5, 50},
		Agent: AgentProfile{Radius: 0.5},
	})
	if !small.Found {
		t.Fatalf("small agent should fit the corridor, got %v", small.Reason)
	}
	// The corridor is 3 wide; an agent with radius 2.5 needs clearance
	// the corridor cannot provide, and there is no other way across.
	big := m.FindPath(context.Background(), Query{
		Start: Vec2{31.5, 10}, Goal: Vec2{31.5, 50},
		Agent: AgentProfile{Radius: 2.5},
	})
	if big.Found {
		t.Fatal("big agent must not fit through the 3-wide corridor")
	}
	if big.Reason != ReasonNoPath {
		t.Fatalf("expected no_path for big agent, got %v", big.Reason)
	}
	// Same mesh, no rebuild: both queries ran against one version.
	if small.Version != big.Version {
		t.Fatal("queries against same mesh report different versions")
	}
	assertPathClean(t, m, small, AgentProfile{Radius: 0.5})
}

func TestUnreachableReasons(t *testing.T) {
	m := Build(corridorScene(), testConfig())
	ctx := context.Background()
	cases := []struct {
		name string
		q    Query
		want UnreachableReason
	}{
		{"start out of bounds", Query{Start: Vec2{-5, 5}, Goal: Vec2{5, 5}}, ReasonOutOfBounds},
		{"goal out of bounds", Query{Start: Vec2{5, 5}, Goal: Vec2{100, 5}}, ReasonOutOfBounds},
		{"agent too large", Query{Start: Vec2{5, 5}, Goal: Vec2{6, 5}, Agent: AgentProfile{Radius: 99}}, ReasonAgentTooLarge},
		{"start in obstacle", Query{Start: Vec2{10, 32}, Goal: Vec2{5, 5}}, ReasonStartBlocked},
		{"goal in obstacle", Query{Start: Vec2{5, 5}, Goal: Vec2{10, 32}}, ReasonGoalBlocked},
		{"goal forbidden", Query{Start: Vec2{5, 5}, Goal: Vec2{56, 56}}, ReasonGoalBlocked},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			res := m.FindPath(ctx, tc.q)
			if res.Found || res.Reason != tc.want {
				t.Fatalf("got found=%v reason=%v, want reason=%v", res.Found, res.Reason, tc.want)
			}
		})
	}
}

func TestNoPathWhenFullyWalled(t *testing.T) {
	scene := Scene{
		Bounds: Rect{0, 0, 32, 32},
		Zones: []Zone{
			// A sealed box around the goal.
			{ID: "b1", Kind: ZoneObstacle, Rect: Rect{16, 16, 24, 18}},
			{ID: "b2", Kind: ZoneObstacle, Rect: Rect{16, 22, 24, 24}},
			{ID: "b3", Kind: ZoneObstacle, Rect: Rect{16, 16, 18, 24}},
			{ID: "b4", Kind: ZoneObstacle, Rect: Rect{22, 16, 24, 24}},
		},
	}
	m := Build(scene, testConfig())
	res := m.FindPath(context.Background(), Query{Start: Vec2{4, 4}, Goal: Vec2{20, 20}})
	if res.Found || res.Reason != ReasonNoPath {
		t.Fatalf("expected no_path, got found=%v reason=%v", res.Found, res.Reason)
	}
}

func TestForbiddenNeverCrossed(t *testing.T) {
	// The only geometric passage to the goal goes through a forbidden
	// zone; the query must fail rather than cross it.
	scene := Scene{
		Bounds: Rect{0, 0, 64, 32},
		Zones: []Zone{
			{ID: "wall-top", Kind: ZoneObstacle, Rect: Rect{30, 0, 34, 12}},
			{ID: "wall-bot", Kind: ZoneObstacle, Rect: Rect{30, 20, 34, 32}},
			{ID: "gate", Kind: ZoneCost, Rect: Rect{30, 12, 34, 20}, Forbidden: true},
		},
	}
	m := Build(scene, testConfig())
	res := m.FindPath(context.Background(), Query{Start: Vec2{8, 16}, Goal: Vec2{56, 16}})
	if res.Found {
		t.Fatal("path must not cross the forbidden gate")
	}
	if res.Reason != ReasonNoPath {
		t.Fatalf("expected no_path, got %v", res.Reason)
	}
}

func TestDeterministicQuery(t *testing.T) {
	m := Build(uShapedScene(), testConfig())
	q := Query{Start: Vec2{8, 8}, Goal: Vec2{32, 30}, Agent: AgentProfile{Radius: 0.5}}
	r1 := m.FindPath(context.Background(), q)
	r2 := m.FindPath(context.Background(), q)
	if r1.Found != r2.Found || r1.Cost != r2.Cost || len(r1.Points) != len(r2.Points) {
		t.Fatal("same query twice gave different results")
	}
	for i := range r1.Points {
		if r1.Points[i] != r2.Points[i] {
			t.Fatal("waypoints differ between identical queries")
		}
	}
}

func TestExpansionLimit(t *testing.T) {
	cfg := testConfig()
	cfg.MaxExpansions = 2
	m := Build(uShapedScene(), cfg)
	res := m.FindPath(context.Background(), Query{Start: Vec2{2, 2}, Goal: Vec2{62, 62}})
	if res.Found || res.Reason != ReasonExpansionLimit {
		t.Fatalf("expected expansion_limit, got found=%v reason=%v", res.Found, res.Reason)
	}
}
