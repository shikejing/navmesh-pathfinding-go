package navmesh

import (
	"context"
	"fmt"
	"testing"
)

// 吞吐基准：不同地图规模与并发度下查询耗时可预期。
func BenchmarkFindPath(b *testing.B) {
	for _, size := range []int{64, 256} {
		cfg := Config{
			WorldMin: Vec2{0, 0}, WorldMax: Vec2{float64(size), float64(size)},
			CellSize: 1, TileCells: 32, QueryWorkers: 8,
		}
		rt := NewRuntime(cfg)
		var obs []Obstacle
		for i := 0; i < size/4; i++ {
			x := float64((i * 37) % (size - 8))
			y := float64((i * 53) % (size - 8))
			obs = append(obs, Obstacle{ID: uint64(i + 1), Box: AABB{Vec2{x, y}, Vec2{x + 6, y + 6}}, MinY: 0, MaxY: 3})
		}
		rt.Rebuild(obs, nil)
		req := PathRequest{
			Start: Vec2{2, 2}, Goal: Vec2{float64(size) - 2, float64(size) - 2},
			Agent: AgentProfile{Radius: 1, Height: 1},
		}
		b.Run(fmt.Sprintf("size=%d", size), func(b *testing.B) {
			b.RunParallel(func(pb *testing.PB) {
				for pb.Next() {
					res, err := rt.FindPath(context.Background(), req)
					if err != nil {
						b.Fatal(err)
					}
					_ = res
				}
			})
		})
		rt.Close()
	}
}

// 增量更新基准：局部变更的代价与地图规模的关系。
func BenchmarkIncrementalUpdate(b *testing.B) {
	for _, size := range []int{64, 256} {
		cfg := Config{
			WorldMin: Vec2{0, 0}, WorldMax: Vec2{float64(size), float64(size)},
			CellSize: 1, TileCells: 32,
		}
		rt := NewRuntime(cfg)
		rt.Rebuild(nil, nil)
		b.Run(fmt.Sprintf("size=%d", size), func(b *testing.B) {
			for i := 0; i < b.N; i++ {
				id := uint64(1000 + i)
				rt.ApplyUpdate(GeometryUpdate{
					AddObstacles: []Obstacle{{ID: id, Box: AABB{Vec2{4, 4}, Vec2{8, 8}}, MinY: 0, MaxY: 2}},
				})
				rt.ApplyUpdate(GeometryUpdate{RemoveObstacleIDs: []uint64{id}})
			}
		})
		rt.Close()
	}
}
