# navmesh-pathfinding-go

本地进程内的导航网格与多代理寻路运行时。面向"场景几何持续变化、大量不同尺寸代理并发查询与频繁重规划"的场景，构建结果与路径结果均确定且可复现。

## 数据模型

```
World (Config.WorldMin..WorldMax)
 └─ Tile[TileCells × TileCells cells]     增量更新的最小单位
     └─ Cell { Walkable, Forbidden, Cost, Clearance, CeilY }
     └─ Region[] + Adj[][]               可行走格贪心切分出的凸矩形及相邻关系
```

- **Obstacle**：水平 AABB + 垂直区间 `[MinY, MaxY]`。贴地障碍（`MinY<=0`）完全阻挡所在格；悬空障碍压低格子净高 `CeilY`。
- **Zone**：为覆盖格设置进入代价倍率（`Cost`）与禁行标记（`Forbidden`）。
- **Cell.Clearance**：到最近不可行走格的距离（多源 BFS，单位：格），供任意半径的代理在查询侧过滤，**不需要为每种代理尺寸重建数据**。
- **AgentProfile**：`Radius` 决定所需净空（`ceil(Radius/CellSize)`），`Height` 决定所需净高。过滤发生在查询侧，不同尺寸查询共享同一快照、互不污染。

## 一致性语义

- **确定性构建**：所有输入按 ID 排序后应用，扫描顺序固定，不依赖 map 迭代顺序；同一几何 + 同一 `Config` 的构建结果（含序列化字节）完全一致。
- **版本化快照（MVCC）**：每次更新产生新的不可变 `NavMesh` 快照并原子发布。查询只在快照上读，任一查询只会看到某个自洽的完整版本；更新与查询互不阻塞。
- **增量更新**：`ApplyUpdate` 只重建与变更 AABB 相交的 tile，未受影响 tile 在新旧版本间共享（写时复制，从不原地修改）。基准显示局部更新代价与地图规模无关。
- **差异可解释**：每个 `PathResult` 携带数据 `Version`；`ExplainDiff(v1, v2)` 返回两版本间的变更摘要（增删的几何、重建的 tile）。
- **取消与超时**：查询遵守 `context.Context`，取消/超时立即中断并返回 `ReasonCanceled`/`ReasonTimeout`；运行时无后台 goroutine，不留悬挂状态。
- **背压**：并发查询数受 `QueryWorkers` 限制，排队超过 `QueryQueueSize` 立即以 `ReasonQueueFull` 失败，慢查询不会阻塞更新与其它查询。

## 不可达语义

| 情况 | Reason | 说明 |
|---|---|---|
| 起点不在任何可通行区域 | `ReasonStartOffMesh` | 含投影搜索失败 |
| 终点不可投影 | `ReasonGoalOffMesh` | 终点在界外或障碍深处 |
| 起终点合法但不连通 | `ReasonNoConnection` | 附带到最近可达点的部分路径（`Partial=true`） |
| 终点在障碍内但附近可投影 | 成功 + `Partial=true` | 路径终止于最近可通行格 |

禁行（Forbidden）格对任何代理都不可通行，任何情况下都不会返回穿越禁行区域的路径。

## 持久化与校验

- `Runtime.Save(w)` / `Save(w, mesh)`：写入 magic + 格式版本 + CRC32 校验和 + 负载（网格与源几何）。
- `Runtime.LoadFrom(r)` / `Load(r, cfg)`：逐项校验 magic、格式版本、CRC32、构建配置（世界范围/CellSize/TileCells）。
- 校验失败返回 `ErrChecksum` / `ErrVersionMismatch` / `ErrInvalidHeader`，调用方必须重建，**不会静默使用过期或损坏数据**。重启后版本号保持单调。

## 配置项（Config）

| 字段 | 默认 | 说明 |
|---|---|---|
| `WorldMin/WorldMax` | 必填 | 世界范围 |
| `CellSize` | 1 | 格子边长（世界单位），决定精度与内存 |
| `TileCells` | 32 | tile 边长（格），决定增量更新粒度 |
| `QueryWorkers` | 4 | 并发查询上限（吞吐调参） |
| `QueryQueueSize` | 1024 | 排队上限，超出立即失败 |
| `MaxExpandedNodes` | 200000 | 单次查询 A* 扩展上限（慢查询保护） |

## 使用示例

```go
rt := navmesh.NewRuntime(navmesh.Config{
    WorldMin: navmesh.Vec2{0, 0}, WorldMax: navmesh.Vec2{256, 256},
    CellSize: 1, TileCells: 32, QueryWorkers: 8,
})
defer rt.Close()
v1 := rt.Rebuild(obstacles, zones)

res, _ := rt.FindPath(ctx, navmesh.PathRequest{
    Start: a, Goal: b, Agent: navmesh.AgentProfile{Radius: 1.5, Height: 2},
})

v2, _ := rt.ApplyUpdate(navmesh.GeometryUpdate{AddObstacles: newWalls})
fmt.Println(rt.ExplainDiff(res.Version, v2)) // 解释重规划原因
```

## 测试

```sh
go test ./... -race          # 确定性构建/多尺寸代理/增量更新/并发/不可达/持久化
go test ./... -bench .       # 吞吐与增量更新基准
```
