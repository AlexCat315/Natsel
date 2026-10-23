# Natsel (Swift) vs Box3D (C) 性能报告

机器：Apple M1，8 核，macOS 27.0.1
构建：C 为 `-O3 -DNDEBUG`（Release，NEON 4 宽求解器）；Swift 为 `swift build -c release`
方法：每个场景跑 3 次取最小值，单线程（`workerCount = 1`），`dt = 1/60`，4 子步
场景定义：与 `box3d/benchmark/main.c` + `shared/benchmarks.c` 逐行对应

## 一、总表

| 场景 | C (ms) | Swift (ms) | 倍数 |
|---|---:|---:|---:|
| large_pyramid | 1537.5 | 4286.2 | 2.79x |
| many_pyramids | 1634.9 | 4120.7 | 2.52x |
| joint_grid | 864.6 | 4322.7 | 5.00x |
| trees25 | 377.1 | 1419.9 | 3.77x |
| trees50 | 173.5 | 669.2 | 3.86x |
| trees100 | 101.5 | 411.0 | 4.05x |
| sleep | 597.3 | 2150.2 | 3.60x |
| spinner | 2825.6 | 12453.0 | 4.41x |
| convex_pile | 5819.2 | 22622.1 | 3.89x |
| washer | 17802.5 | 79218.5 | 4.45x |
| **合计** | **31733.7** | **131673.5** | **4.15x** |

**未达到 5% 以内的目标。** 当前整体差距 4.15 倍。

## 二、本轮改动前后对比（同一台机器）

| 场景 | 改动前 (ms) | 改动后 (ms) | 提升 |
|---|---:|---:|---:|
| large_pyramid | 6185 | 4286 | 1.44x |
| many_pyramids | 6312 | 4121 | 1.53x |
| joint_grid | 7534 | 4323 | 1.74x |
| convex_pile | 56280 | 22622 | 2.49x |
| spinner | 75562 | 12453 | 6.07x |
| washer | 83038 | 79219 | 1.05x |
| trees25 | 1464 | 1420 | 1.03x |

整体从约 5.0x 缩到 4.15x。trees / washer 场景几乎没动，因为它们的耗时不在这些结构开销上。

## 三、找到并修掉的真实问题

这些是 Swift 特有的、C 里不存在的开销：

1. **`JointSim` 1714 字节，按值整数组复制**
   `constraintGraph.colors[c].jointSims` 在三个热循环里被 `var list = ...` 复制后写回，
   每个关节每次 1714 字节。改成 `withUnsafeMutableBufferPointer` 原地修改。
   `joint_grid` 7534 → 4227 ms。**这是最大单点收益。**

2. **`solverSets[...]` 下标复制整个 `SolverSet`**
   该结构含 4 个数组，每次下标都是一次 ARC 保留/释放。求解器每个约束读两次状态
   （`solverState` + `storeSolverState`），等于每约束两次整体复制。
   改为每阶段借出一次 `bodyStates` 缓冲区。

3. **`ManifoldConstraint`（324 字节）逐 manifold 取值/写回**
   solve / push / warmStart / restitution 四个阶段都是 `var cm = pool[i]` + 写回。
   改为 `pool.baseAddress! + i` 指针原地读写。`large_pyramid` 的 solve 8.23 → 5.68 ms/step。

4. **`Contact`（232 字节，含 3 个引用类型字段）反复复制**
   `storeImpulses` 的 `var contact = contacts[id]` 复制后写回，使 `manifolds` 数组
   变成多重引用，于是每次写点都触发整个数组 CoW。改为通过
   `contacts.withUnsafeMutableBufferPointer` / 新增的 `ContactStore.withContact` 原地写。

5. **`ContactStore(&contacts)` 每个接触点新建一次**
   `collideOneContact` 每处理一个接触就取一次写引用，使整个 contacts 数组多重引用，
   下一次写入即整数组复制。washer 约 2 万接触 × 4 子步 × 1000 步，放大极严重。
   改为整遍（`collideEachContact`）共用一个 store。

6. **`finalizeBodies` 里 shapes 数组的 CoW 与 solverSets 来回搬运**
   `var shape = shapes[id]` + 写回导致整张 shape 表复制；快速刚体分支还把整个
   `SolverSet` 写进 `solverSets` 再从里面读回来（每刚体一次）。改为原地指针遍历，
   并把 `solveContinuous` 改成接收 `inout SolverSet`，循环内不再搬运。
   spinner 75562 → 12453 ms。

7. **`computeSeparatingAxis` 每次调用堆分配 13+ 个 scratch 数组**
   改为栈上 `InlineArray<132, Float>`（新增 `withInlineBuffer` 辅助）。

8. **`collideStatePass` / `emitContactHitEvents` 全量扫描时复制每个 Contact**
   改为先读 `flags` 标量，未命中就 `continue`。

## 四、剩下的差距在哪

C 的分阶段数据（large_pyramid，ms/step）：

| 阶段 | C (NEON) | C (纯标量) | Swift |
|---|---:|---:|---:|
| pairs | 0.27 | 0.40 | 1.04 |
| collide | 1.59 | 1.43 | 4.11 |
| prepare | 1.17 | 1.30 | 2.61 |
| warm | 0.52 | 1.04 | 1.19 |
| push | 1.07 | 2.04 | 3.57 |
| solve | 1.62 | 2.98 | 8.23 → 5.68 |
| store | 0.61 | 0.58 | 3.52 → 2.10 |
| **合计** | **7.4** | **10.3** | **~22** |

关键结论：

- **Swift 目前已经接近 C 的纯标量路径**（22 vs 10.3，约 2.1x），但仍慢一倍。
- **C 的默认构建用的是 4 宽 NEON 求解器**，比它自己的标量路径快约 1.5x
  （AMD 参考数据：scalar 5027 ms / SSE2 1483 ms，同一比例）。
  Swift 只有 `pushContactsWide` 一条宽路径，且默认关闭、只覆盖 push 阶段，
  打开后反而更慢（6250 vs 6100 ms），因为它不完整。
- 也就是说：要达到 1.05x，Swift 需要**同时**补上 4 宽 SIMD 求解器（warmStart / push /
  solve / restitution / store 五个阶段），**并且**再消掉对标量 C 余下的约 2x ARC/下标开销。

## 五、结论与建议

本轮把显存/ARC 结构性问题清理掉了，整体 5.0x → 4.15x，几个场景（joint_grid、spinner、
convex_pile）提升 1.7–6x，380 项测试全绿（含全部确定性哈希）。
但 **5% 以内没有达成**，差距是数量级问题而非调参问题。

要真正接近 C，后续需要两件事，且都属独立大工程：

1. **补完整的 4 宽求解器**，与 `contact_solver_wide.inl` 对齐，覆盖全部五个阶段。
   现有 `WideTypes.swift` 可复用，但需要按 C 的 SoA 布局重排约束数据（C 的
   `b3ContactConstraintWide` 是分离数组，这是能向量化的前提）。
2. **消除 `Contact` / `SolverSet` 的引用类型字段**，改成值类型 SoA 池
   （C 的 contact 数组就是纯 POD）。这是剩余 2x 里最硬的一块。

如果只想要单场景达标，`joint_grid` 现在 5.00x 是最差的一个，
它 100% 是关节求解开销，补完关节的宽路径收益最直接。
