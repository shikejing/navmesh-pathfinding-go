# navmesh-pathfinding-go

进程内的导航网格与多代理寻路运行时。场景几何可持续变化，大量不同尺寸的
代理并发查询与重规划；构建结果与路径结果均确定且可复现。所有输入来自
内存或本地文件，无外部依赖。

## 数据模型

```
Scene                 世界包围盒 + Zone 列表（唯一输入几何）
 └─ Zone              矩形区域：Obstacle（禁行）或 Cost（代价/禁行标记/限高）
NavMesh（不可变）      一次构建/更新产生的完整版本
 ├─ Cell             栅格单元：blocked / cost / forbidden / ceiling / clearance
 ├─ Tile             固定大小的单元块，增量重建与版本间结构共享的最小单位
 └─ Region           可通行矩形区域（凹区域被切分为多个），携带
                     cost / forbidden / ceiling / min-clearance 与邻接表
```

- **栅格化**：以 `CellSize` 把场景离散为单元格。障碍单元格不可通行；
  Cost Zone 按 ID 字典序取第一个覆盖单元格中心的区域，保证与输入顺序无关。
- **Clearance**：每个单元格记录到最近障碍（或地图边缘）的距离，多源 BFS
  计算，超过 `MaxAgentRadius + CellSize` 饱和。查询时代理半径只需与
  clearance 比较，**不需要为每种代理尺寸重建数据**。
- **区域切分**：按扫描线把可通行单元格合并为最大矩形；只有 cost、
  forbidden、ceiling 与 clearance 波段（MaxAgentRadius 的 1/4、1/2、1 倍
  分四档）都相同的单元格才会合并，因此凹区域被正确切分，且开阔区不会被
  墙边低 clearance 污染。邻接关系由共享边建立，跨区域（跨 Tile）同样生效。
- **区域 ID**：`tileIndex<<32 | 局部序号`，全局唯一且跨版本稳定；未变化的
  Tile 在新版本中按指针共享，区域 ID 不会漂移。

## 一致性语义

- **确定性构建**：同一 `Scene` + `Config` 必然产生逐字节一致的网格
  （`NavMesh.Checksum()` 相同），与 Zone 输入顺序、地图遍历顺序无关。
  A* 以 `(f, 节点ID)` 全序打破平局，同版本同查询结果完全一致。
- **快照隔离**：`Runtime` 持有原子指针指向当前 `NavMesh` 版本。每次查询
  开始时取一次快照，整个查询只看到这一个自洽版本；增量更新构建新版本后
  原子换出，进行中的查询不受阻塞、也不看到半成品。
- **增量更新**：`Update` 只重建被变更 Zone（旧形状与新形状）触及的 Tile
  及其 clearance halo（`ceil(MaxAgentRadius/CellSize)+1` 格），其余 Tile
  结构共享。测试保证增量结果与全量重建的 Checksum 一致。
- **差异解释**：`Update` 返回 `Diff{ChangedTiles, RemovedRegions,
  AddedRegions}`。更新前后路径不同，可用变更 Tile 是否覆盖编辑区域、
  哪些区域被替换来解释（见 `TestVersionConsistency`）。
- **不可达区分**：`PathResult.Reason` 区分 `out_of_bounds` /
  `agent_too_large` / `start_blocked` / `goal_blocked`（含禁行）/
  `no_path`（两端可通行但不连通，即部分可达）/ `cancelled` /
  `expansion_limit`。任何情况下都不会返回穿越禁行或障碍区域的路径。
- **并发与背压**：查询并发度由 `QueryWorkers` 信号量限制；超时与
  `context` 取消统一返回 `cancelled`，不占用 worker、不留悬挂状态；
  慢查询只占用自己的 worker 槽，不阻塞更新与其它查询。
- **持久化校验**：`Save` 写入 `magic | 格式版本 | 配置哈希 | 网格Checksum
  | gob 载荷 | CRC32`。`Load` 依次校验 CRC、magic、格式版本、配置哈希，
  并在解码后重算网格 Checksum；任一不匹配都返回错误，调用方必须重建，
  绝不静默使用过期或损坏数据。

## 配置项（Config）

| 字段 | 默认 | 作用 |
|---|---|---|
| `CellSize` | 1 | 栅格单元边长（世界单位）。越小越精确，构建越慢 |
| `TileCells` | 16 | Tile 边长（格）。增量重建粒度；越小更新越局部 |
| `MaxAgentRadius` | 4×CellSize | clearance 精确上限与增量更新 halo；超过此半径的代理在查询时被拒绝 |
| `QueryWorkers` | GOMAXPROCS | 并发查询上限（吞吐调参） |
| `QueryTimeout` | 无 | 单次查询默认超时 |
| `MaxExpansions` | 不限 | 单次 A* 扩展上限，防止病态查询拖垮 worker |

吞吐随地图规模与查询数量可预期：单次查询代价约为 O(区域数 log 区域数)，
区域数随地图规模近似线性；并发吞吐近似随 `QueryWorkers` 线性
（见 `TestThroughputScalesWithWorkers`）。

## 持久化与重启恢复

```go
// 提交后落盘
f, _ := os.Create("navmesh.bin")
navmesh.Save(f, mesh)

// 重启后校验；失败必须重建，不得降级使用
m, err := navmesh.Load(f, cfg)
if err != nil {
    m = navmesh.Build(scene, cfg) // 损坏 / 版本或配置不匹配 → 重建
}
```

## 测试

```sh
go test ./...          # 全部测试
go test ./... -race    # 含并发竞争检测
```

覆盖：确定性构建（含输入乱序）、凹区域切分与邻接对称、多尺寸代理共享
网格、禁行不穿越、各类不可达原因、增量更新局部性与增量=全量一致性、
版本快照隔离、并发查询与更新（-race）、取消/超时无悬挂、持久化往返与
损坏/版本/配置不匹配检测。
