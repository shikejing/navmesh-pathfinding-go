package navmesh

import (
	"context"
	"runtime"
	"sync"
	"sync/atomic"
)

// Runtime serves concurrent queries and incremental updates over
// versioned, immutable mesh snapshots. A query always observes one
// self-consistent version; updates never block in-flight queries.
type Runtime struct {
	cfg  Config
	snap atomic.Pointer[NavMesh]
	sem  chan struct{}
	mu   sync.Mutex // serialises updates so versions stay linear
}

// NewRuntime creates a Runtime serving the given initial mesh.
func NewRuntime(initial *NavMesh, cfg Config) *Runtime {
	cfg = cfg.withDefaults()
	workers := cfg.QueryWorkers
	if workers <= 0 {
		workers = runtime.GOMAXPROCS(0)
	}
	r := &Runtime{cfg: cfg, sem: make(chan struct{}, workers)}
	if initial != nil {
		r.snap.Store(initial)
	}
	return r
}

// Mesh returns the current mesh snapshot. Safe for concurrent use.
func (r *Runtime) Mesh() *NavMesh {
	return r.snap.Load()
}

// Update applies zone upserts/removals and atomically publishes the new
// version. In-flight queries keep running on their own snapshot.
func (r *Runtime) Update(upserts []Zone, removeIDs []string) (Diff, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	cur := r.snap.Load()
	nm, diff, err := cur.Update(upserts, removeIDs)
	if err != nil {
		return Diff{}, err
	}
	r.snap.Store(nm)
	return diff, nil
}

// FindPath executes a query against the current snapshot, bounded by the
// worker pool and the configured query timeout. The query observes exactly
// one mesh version. Cancellation leaves no dangling state.
func (r *Runtime) FindPath(ctx context.Context, q Query) PathResult {
	if r.cfg.QueryTimeout > 0 {
		var cancel context.CancelFunc
		ctx, cancel = context.WithTimeout(ctx, r.cfg.QueryTimeout)
		defer cancel()
	}
	select {
	case r.sem <- struct{}{}:
		defer func() { <-r.sem }()
	case <-ctx.Done():
		return PathResult{Reason: ReasonCancelled}
	}
	m := r.snap.Load()
	if m == nil {
		return PathResult{Reason: ReasonNoPath}
	}
	return m.FindPath(ctx, q)
}
