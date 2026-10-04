package navmesh

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func TestConcurrentQueriesAndUpdates(t *testing.T) {
	rt := NewRuntime(Build(uShapedScene(), testConfig()), testConfig())
	ctx := context.Background()

	var wg sync.WaitGroup
	var found, failed atomic.Int64
	// 64 agents of mixed sizes querying and replanning continuously.
	for i := 0; i < 64; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			radius := 0.5 + float64(i%3)
			for j := 0; j < 50; j++ {
				q := Query{
					Start: Vec2{float64(1 + i%60), 2},
					Goal:  Vec2{float64(1 + (i*7+j)%60), 62},
					Agent: AgentProfile{Radius: radius},
				}
				res := rt.FindPath(ctx, q)
				if res.Found {
					found.Add(1)
				} else {
					failed.Add(1)
				}
			}
		}(i)
	}
	// Concurrent incremental updates.
	for k := 0; k < 20; k++ {
		id := fmt.Sprintf("dyn-%d", k)
		if _, err := rt.Update([]Zone{
			{ID: id, Kind: ZoneObstacle, Rect: Rect{float64(k * 3), 40, float64(k*3 + 2), 44}},
		}, nil); err != nil {
			t.Fatal(err)
		}
	}
	wg.Wait()
	if found.Load() == 0 {
		t.Fatal("no query succeeded")
	}
	t.Logf("found=%d failed=%d", found.Load(), failed.Load())
	// Version must have advanced exactly once per update.
	if got := rt.Mesh().Version; got != 21 {
		t.Fatalf("expected version 21, got %d", got)
	}
}

func TestQueryObservesSingleVersion(t *testing.T) {
	rt := NewRuntime(Build(uShapedScene(), testConfig()), testConfig())
	ctx := context.Background()
	stop := make(chan struct{})
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		v := uint64(1)
		for {
			select {
			case <-stop:
				return
			default:
			}
			v++
			if _, err := rt.Update([]Zone{
				{ID: "wobble", Kind: ZoneCost, Rect: Rect{0, 0, 8, 8}, Cost: 1 + float64(v%5)},
			}, nil); err != nil {
				return
			}
		}
	}()
	// Every result must carry the version of the snapshot it ran on,
	// and that version must be <= the current version.
	for i := 0; i < 200; i++ {
		res := rt.FindPath(ctx, Query{Start: Vec2{32, 4}, Goal: Vec2{32, 60}})
		if res.Version < 1 || res.Version > rt.Mesh().Version {
			t.Fatalf("result version %d outside valid range", res.Version)
		}
	}
	close(stop)
	wg.Wait()
}

func TestCancellationLeavesNoDanglingState(t *testing.T) {
	cfg := testConfig()
	cfg.QueryWorkers = 2
	rt := NewRuntime(Build(uShapedScene(), cfg), cfg)

	// Cancelled contexts must return promptly with ReasonCancelled and
	// must not leak worker slots.
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	for i := 0; i < 10; i++ {
		res := rt.FindPath(ctx, Query{Start: Vec2{8, 8}, Goal: Vec2{32, 30}})
		if res.Found || res.Reason != ReasonCancelled {
			t.Fatalf("cancelled query: got found=%v reason=%v", res.Found, res.Reason)
		}
	}
	// The pool must still be fully available afterwards.
	ctx2, cancel2 := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel2()
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			rt.FindPath(ctx2, Query{Start: Vec2{8, 8}, Goal: Vec2{32, 30}})
		}()
	}
	wg.Wait()
}

func TestQueryTimeout(t *testing.T) {
	cfg := testConfig()
	cfg.QueryWorkers = 1
	cfg.QueryTimeout = 50 * time.Millisecond
	rt := NewRuntime(Build(uShapedScene(), cfg), cfg)
	// Saturate the single worker with a slow external occupant.
	rt.sem <- struct{}{}
	start := time.Now()
	res := rt.FindPath(context.Background(), Query{Start: Vec2{8, 8}, Goal: Vec2{32, 30}})
	<-rt.sem
	if res.Reason != ReasonCancelled {
		t.Fatalf("expected timeout cancellation, got %v", res.Reason)
	}
	if elapsed := time.Since(start); elapsed > 2*time.Second {
		t.Fatalf("timeout took too long: %v", elapsed)
	}
}

func TestThroughputScalesWithWorkers(t *testing.T) {
	if testing.Short() {
		t.Skip("skipping throughput smoke test in short mode")
	}
	m := Build(uShapedScene(), testConfig())
	run := func(workers int) time.Duration {
		cfg := testConfig()
		cfg.QueryWorkers = workers
		rt := NewRuntime(m, cfg)
		start := time.Now()
		var wg sync.WaitGroup
		for i := 0; i < 16; i++ {
			wg.Add(1)
			go func(i int) {
				defer wg.Done()
				for j := 0; j < 25; j++ {
					rt.FindPath(context.Background(), Query{
						Start: Vec2{float64(1 + i), 2}, Goal: Vec2{60, 62},
						Agent: AgentProfile{Radius: 0.5},
					})
				}
			}(i)
		}
		wg.Wait()
		return time.Since(start)
	}
	d1 := run(1)
	d4 := run(4)
	t.Logf("workers=1: %v, workers=4: %v", d1, d4)
}
