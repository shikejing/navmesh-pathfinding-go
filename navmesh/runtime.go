package navmesh

import (
	"context"
	"errors"
	"fmt"
	"io"
	"sort"
	"sync"
	"sync/atomic"
)

// ErrClosed 表示运行时已关闭。
var ErrClosed = errors.New("navmesh: runtime is closed")

// GeometryUpdate 为一次增量几何变更。
type GeometryUpdate struct {
	AddObstacles      []Obstacle
	RemoveObstacleIDs []uint64
	AddZones          []Zone
	RemoveZoneIDs     []uint64
}

// changeEntry 记录一次版本变更的摘要，用于解释路径差异。
type changeEntry struct {
	Version uint64
	Summary string
}

const changeLogCap = 128

// Runtime 管理导航数据的版本化生命周期与并发查询。
//
// 一致性语义：
//   - 每次更新产生一个新的不可变 NavMesh 快照并原子发布；
//   - 查询在快照上执行，任一查询只会看到某个自洽的完整版本；
//   - 更新只重建受影响 tile，未受影响的 tile 在新旧版本间共享（从不原地修改）；
//   - 更新与查询互不阻塞；查询并发度受 QueryWorkers 限制。
type Runtime struct {
	cfg Config

	mu        sync.Mutex // 串行化更新
	cur       atomic.Pointer[NavMesh]
	obstacles map[uint64]Obstacle
	zones     map[uint64]Zone
	version   uint64
	changeLog []changeEntry

	sem     chan struct{} // 并发查询配额
	waiters atomic.Int32  // 排队中的查询数
	closed  atomic.Bool
}

// NewRuntime 创建运行时（尚未有任何导航数据）。
func NewRuntime(cfg Config) *Runtime {
	cfg = cfg.withDefaults()
	r := &Runtime{
		cfg:       cfg,
		obstacles: make(map[uint64]Obstacle),
		zones:     make(map[uint64]Zone),
		sem:       make(chan struct{}, cfg.QueryWorkers),
	}
	return r
}

// sortedGeometry 返回按 ID 排序的几何切片，保证构建顺序确定。
func (r *Runtime) sortedGeometry() ([]Obstacle, []Zone) {
	obs := make([]Obstacle, 0, len(r.obstacles))
	for _, o := range r.obstacles {
		obs = append(obs, o)
	}
	zns := make([]Zone, 0, len(r.zones))
	for _, z := range r.zones {
		zns = append(zns, z)
	}
	sort.Slice(obs, func(i, j int) bool { return obs[i].ID < obs[j].ID })
	sort.Slice(zns, func(i, j int) bool { return zns[i].ID < zns[j].ID })
	return obs, zns
}

// publish 发布新快照并记录变更摘要。调用方需持有 r.mu。
func (r *Runtime) publish(m *NavMesh, summaries []string) {
	r.version++
	m.Version = r.version
	r.cur.Store(m)
	for _, s := range summaries {
		r.changeLog = append(r.changeLog, changeEntry{Version: r.version, Summary: s})
	}
	if len(r.changeLog) > changeLogCap {
		r.changeLog = append([]changeEntry(nil), r.changeLog[len(r.changeLog)-changeLogCap:]...)
	}
}

// Rebuild 全量重建并发布为新版本。
func (r *Runtime) Rebuild(obstacles []Obstacle, zones []Zone) uint64 {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.obstacles = make(map[uint64]Obstacle, len(obstacles))
	for _, o := range obstacles {
		r.obstacles[o.ID] = o
	}
	r.zones = make(map[uint64]Zone, len(zones))
	for _, z := range zones {
		r.zones[z.ID] = z
	}
	obs, zns := r.sortedGeometry()
	m := NewBuilder(r.cfg).BuildAll(obs, zns)
	r.publish(m, []string{fmt.Sprintf("full rebuild: %d obstacles, %d zones, %d tiles",
		len(obs), len(zns), len(m.Tiles))})
	return r.version
}

// tilesForAABB 返回与盒相交的 tile 键（按确定顺序）。
func (r *Runtime) tilesForAABB(m *NavMesh, box AABB) []TileKey {
	cs := r.cfg.CellSize
	tc := r.cfg.TileCells
	x0 := int((box.Min.X - r.cfg.WorldMin.X) / cs)
	y0 := int((box.Min.Y - r.cfg.WorldMin.Y) / cs)
	x1 := int((box.Max.X - r.cfg.WorldMin.X) / cs)
	y1 := int((box.Max.Y - r.cfg.WorldMin.Y) / cs)
	if x0 < 0 {
		x0 = 0
	}
	if y0 < 0 {
		y0 = 0
	}
	if x1 >= m.cellsX {
		x1 = m.cellsX - 1
	}
	if y1 >= m.cellsY {
		y1 = m.cellsY - 1
	}
	var keys []TileKey
	for ty := y0 / tc; ty <= y1/tc; ty++ {
		for tx := x0 / tc; tx <= x1/tc; tx++ {
			keys = append(keys, TileKey{tx, ty})
		}
	}
	return keys
}

// ApplyUpdate 应用增量更新，仅重建受影响 tile，返回新版本号。
// 更新与并发查询互不阻塞：查询继续读旧快照，新快照原子发布。
func (r *Runtime) ApplyUpdate(u GeometryUpdate) (uint64, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed.Load() {
		return 0, ErrClosed
	}
	old := r.cur.Load()
	if old == nil {
		return 0, errors.New("navmesh: no base data, call Rebuild first")
	}

	var summaries []string
	dirty := make(map[TileKey]struct{})
	mark := func(box AABB) {
		for _, k := range r.tilesForAABB(old, box) {
			dirty[k] = struct{}{}
		}
	}
	for _, o := range u.AddObstacles {
		r.obstacles[o.ID] = o
		mark(o.Box)
		summaries = append(summaries, fmt.Sprintf("obstacle %d added (box %.1f,%.1f-%.1f,%.1f)",
			o.ID, o.Box.Min.X, o.Box.Min.Y, o.Box.Max.X, o.Box.Max.Y))
	}
	for _, id := range u.RemoveObstacleIDs {
		if o, ok := r.obstacles[id]; ok {
			delete(r.obstacles, id)
			mark(o.Box)
			summaries = append(summaries, fmt.Sprintf("obstacle %d removed", id))
		}
	}
	for _, z := range u.AddZones {
		r.zones[z.ID] = z
		mark(z.Box)
		summaries = append(summaries, fmt.Sprintf("zone %d added (forbidden=%v cost=%.1f)", z.ID, z.Forbidden, z.Cost))
	}
	for _, id := range u.RemoveZoneIDs {
		if z, ok := r.zones[id]; ok {
			delete(r.zones, id)
			mark(z.Box)
			summaries = append(summaries, fmt.Sprintf("zone %d removed", id))
		}
	}
	if len(dirty) == 0 {
		return r.version, nil // 无实际变化，不产生新版本
	}

	obs, zns := r.sortedGeometry()
	b := NewBuilder(r.cfg)

	// 写时复制：共享未受影响 tile，替换受影响 tile。
	tiles := make(map[TileKey]*Tile, len(old.Tiles))
	for k, t := range old.Tiles {
		tiles[k] = t
	}
	keys := make([]TileKey, 0, len(dirty))
	for k := range dirty {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].Y != keys[j].Y {
			return keys[i].Y < keys[j].Y
		}
		return keys[i].X < keys[j].X
	})
	for _, k := range keys {
		tiles[k] = b.BuildTile(k.X, k.Y, obs, zns)
		summaries = append(summaries, fmt.Sprintf("tile (%d,%d) rebuilt", k.X, k.Y))
	}

	m := &NavMesh{
		Config: r.cfg,
		Tiles:  tiles,
		cellsX: old.cellsX,
		cellsY: old.cellsY,
	}
	r.publish(m, summaries)
	return r.version, nil
}

// FindPath 提交一次寻路查询，遵守 ctx 的取消与超时。
// 并发度受 Config.QueryWorkers 限制；排队超过 Config.QueryQueueSize 时
// 立即失败（ReasonQueueFull），不会阻塞更新与其它查询。
func (r *Runtime) FindPath(ctx context.Context, req PathRequest) (PathResult, error) {
	if r.closed.Load() {
		return PathResult{Reason: ReasonCanceled}, ErrClosed
	}
	if n := r.waiters.Add(1); n > int32(r.cfg.QueryQueueSize) {
		r.waiters.Add(-1)
		return PathResult{Reason: ReasonQueueFull}, nil
	}
	defer r.waiters.Add(-1)

	select {
	case r.sem <- struct{}{}:
		defer func() { <-r.sem }()
	case <-ctx.Done():
		res := PathResult{Reason: ReasonCanceled}
		if ctx.Err() == context.DeadlineExceeded {
			res.Reason = ReasonTimeout
		}
		return res, nil
	}

	m := r.cur.Load()
	if m == nil {
		return PathResult{}, errors.New("navmesh: no data, call Rebuild first")
	}
	// 在快照上执行，快照不可变，无需持锁。
	return FindPath(ctx, m, req), nil
}

// ExplainDiff 解释版本 v1 到 v2 之间影响寻路的变化。
// 若历史日志已被截断，会在首条说明。
func (r *Runtime) ExplainDiff(v1, v2 uint64) []string {
	r.mu.Lock()
	defer r.mu.Unlock()
	if v1 > v2 {
		v1, v2 = v2, v1
	}
	var out []string
	if len(r.changeLog) > 0 && r.changeLog[0].Version > v1+1 {
		out = append(out, fmt.Sprintf("history truncated: versions <= %d not retained", r.changeLog[0].Version-1))
	}
	for _, e := range r.changeLog {
		if e.Version > v1 && e.Version <= v2 {
			out = append(out, fmt.Sprintf("v%d: %s", e.Version, e.Summary))
		}
	}
	if len(out) == 0 {
		out = append(out, "no recorded changes between the two versions")
	}
	return out
}

// Save 将当前已提交的快照连同源几何写入 w，
// 重启后可通过 LoadFrom 恢复并继续增量更新。
func (r *Runtime) Save(w io.Writer) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	m := r.cur.Load()
	if m == nil {
		return errors.New("navmesh: no data to save")
	}
	obs, zns := r.sortedGeometry()
	return saveData(w, m, obs, zns)
}

// LoadFrom 从 rdr 加载并校验数据，校验通过后作为当前版本发布。
// 校验失败（损坏/版本不匹配）返回错误，运行时状态不变，
// 调用方应改用 Rebuild 重建。
func (r *Runtime) LoadFrom(rdr io.Reader) error {
	m, obs, zns, err := loadData(rdr, r.cfg)
	if err != nil {
		return err
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	r.obstacles = make(map[uint64]Obstacle, len(obs))
	for _, o := range obs {
		r.obstacles[o.ID] = o
	}
	r.zones = make(map[uint64]Zone, len(zns))
	for _, z := range zns {
		r.zones[z.ID] = z
	}
	// 保留已提交的版本号，保证重启前后版本单调。
	if m.Version <= r.version {
		m.Version = r.version + 1
	}
	r.version = m.Version
	r.cur.Store(m)
	r.changeLog = append(r.changeLog, changeEntry{Version: r.version, Summary: "loaded from persistent storage"})
	return nil
}

// Version 返回当前最新版本号。
func (r *Runtime) Version() uint64 {
	return r.version
}

// Snapshot 返回当前版本快照（只读使用，调用方不得修改）。
func (r *Runtime) Snapshot() *NavMesh {
	return r.cur.Load()
}

// Close 释放运行时资源。关闭后查询返回 ErrClosed。
// 运行时无后台 goroutine，取消/超时不会留下悬挂状态。
func (r *Runtime) Close() {
	r.closed.Store(true)
}
