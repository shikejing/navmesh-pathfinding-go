package navmesh

import (
	"context"
	"fmt"
	"strings"
	"sync"
	"testing"
)

// 增量更新只重建受影响 tile，未受影响 tile 在新旧版本间共享。
func TestIncrementalUpdate(t *testing.T) {
	rt := NewRuntime(testConfig())
	defer rt.Close()
	obs, zones := testGeometry()
	v1 := rt.Rebuild(obs, zones)
	m1 := rt.Snapshot()

	// 在 tile (2,2)（世界 32..48）内新增障碍。
	v2, err := rt.ApplyUpdate(GeometryUpdate{
		AddObstacles: []Obstacle{{ID: 100, Box: AABB{Vec2{36, 36}, Vec2{40, 40}}, MinY: 0, MaxY: 3}},
	})
	if err != nil {
		t.Fatal(err)
	}
	if v2 <= v1 {
		t.Fatal("version should increase")
	}
	m2 := rt.Snapshot()
	if m2.Tiles[TileKey{2, 2}] == m1.Tiles[TileKey{2, 2}] {
		t.Fatal("affected tile should be rebuilt")
	}
	if m2.Tiles[TileKey{0, 0}] != m1.Tiles[TileKey{0, 0}] {
		t.Fatal("unaffected tile should be shared, not rebuilt")
	}

	// 无实际变化的更新不产生新版本。
	v3, _ := rt.ApplyUpdate(GeometryUpdate{RemoveObstacleIDs: []uint64{9999}})
	if v3 != v2 {
		t.Fatal("no-op update should not bump version")
	}

	// 移除障碍后 tile 恢复。
	v4, _ := rt.ApplyUpdate(GeometryUpdate{RemoveObstacleIDs: []uint64{100}})
	if v4 <= v2 {
		t.Fatal("version should increase after removal")
	}
	m4 := rt.Snapshot()
	if !tileEqual(m4.Tiles[TileKey{2, 2}], m1.Tiles[TileKey{2, 2}]) {
		t.Fatal("tile should be identical after add+remove round trip")
	}
}

func tileEqual(a, b *Tile) bool {
	if a.W != b.W || a.H != b.H || len(a.Cells) != len(b.Cells) {
		return false
	}
	for i := range a.Cells {
		if a.Cells[i] != b.Cells[i] {
			return false
		}
	}
	return true
}

// 更新前后路径差异可用版本号与变更日志解释。
func TestExplainDiff(t *testing.T) {
	rt := NewRuntime(testConfig())
	defer rt.Close()
	rt.Rebuild(nil, nil)
	req := PathRequest{Start: Vec2{8, 16}, Goal: Vec2{56, 48}, Agent: AgentProfile{Radius: 0.5}}

	r1, err := rt.FindPath(context.Background(), req)
	if err != nil || r1.Reason != ReasonNone {
		t.Fatalf("baseline path failed: %v %v", r1.Reason, err)
	}
	// 横断墙挡住直线路径。
	v2, _ := rt.ApplyUpdate(GeometryUpdate{
		AddObstacles: []Obstacle{{ID: 7, Box: AABB{Vec2{0, 30}, Vec2{64, 34}}, MinY: 0, MaxY: 3}},
	})
	r2, err := rt.FindPath(context.Background(), req)
	if err != nil {
		t.Fatal(err)
	}
	if r2.Reason != ReasonNoConnection {
		t.Fatalf("expected NoConnection after wall, got %v", r2.Reason)
	}
	if r1.Version == r2.Version {
		t.Fatal("results should carry different versions")
	}
	ex := rt.ExplainDiff(r1.Version, v2)
	joined := strings.Join(ex, "\n")
	if !strings.Contains(joined, "obstacle 7 added") {
		t.Fatalf("explanation should mention the added obstacle, got:\n%s", joined)
	}
}

// 并发查询与增量更新同时进行：任一查询只能看到自洽版本，无数据竞争。
func TestConcurrentQueryAndUpdate(t *testing.T) {
	cfg := testConfig()
	cfg.QueryWorkers = 8
	rt := NewRuntime(cfg)
	defer rt.Close()
	obs, zones := testGeometry()
	rt.Rebuild(obs, zones)

	var wg sync.WaitGroup
	errs := make(chan error, 4096)

	// 查询协程：不同尺寸代理并发查询。
	for w := 0; w < 16; w++ {
		wg.Add(1)
		go func(w int) {
			defer wg.Done()
			for i := 0; i < 100; i++ {
				req := PathRequest{
					Start: Vec2{float64((w*7+i)%60) + 2, float64((w*3+i)%60) + 2},
					Goal:  Vec2{float64((w*11+i*3)%60) + 2, float64((w*5+i*7)%60) + 2},
					Agent: AgentProfile{Radius: float64(w%4) * 0.8, Height: 1 + float64(w%3)},
				}
				res, err := rt.FindPath(context.Background(), req)
				if err != nil {
					errs <- err
					return
				}
				if res.Version == 0 {
					errs <- fmt.Errorf("query returned version 0")
					return
				}
			}
		}(w)
	}
	// 更新协程：持续增量变更。
	for u := 0; u < 4; u++ {
		wg.Add(1)
		go func(u int) {
			defer wg.Done()
			for i := 0; i < 20; i++ {
				id := uint64(1000 + u*100 + i)
				x := float64((u*13 + i*7) % 48)
				rt.ApplyUpdate(GeometryUpdate{
					AddObstacles: []Obstacle{{ID: id, Box: AABB{Vec2{x, x}, Vec2{x + 4, x + 4}}, MinY: 0, MaxY: 2}},
				})
			}
		}(u)
	}
	wg.Wait()
	close(errs)
	for e := range errs {
		t.Fatal(e)
	}
}

// 快照一致性：更新发布后，旧快照上的查询结果仍基于旧数据。
func TestSnapshotIsolation(t *testing.T) {
	rt := NewRuntime(testConfig())
	defer rt.Close()
	rt.Rebuild(nil, nil)
	old := rt.Snapshot()

	rt.ApplyUpdate(GeometryUpdate{
		AddObstacles: []Obstacle{{ID: 1, Box: AABB{Vec2{0, 30}, Vec2{64, 34}}, MinY: 0, MaxY: 3}},
	})
	req := PathRequest{Start: Vec2{8, 16}, Goal: Vec2{56, 48}, Agent: AgentProfile{Radius: 0.5}}

	// 旧快照：仍能直线穿过（数据自洽）。
	rOld := FindPath(context.Background(), old, req)
	if rOld.Reason != ReasonNone {
		t.Fatalf("old snapshot should still allow the path, got %v", rOld.Reason)
	}
	// 新快照：被墙阻断。
	rNew, _ := rt.FindPath(context.Background(), req)
	if rNew.Reason != ReasonNoConnection {
		t.Fatalf("new snapshot should block the path, got %v", rNew.Reason)
	}
}
