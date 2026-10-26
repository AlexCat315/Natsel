# Natsel (Swift) vs Box3D (C) 性能报告

> 第一至五节保留原始测量，第六、七节为阶段记录。最新完整十场景复测见第八节。

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


## 六、四路 SIMD 与碰撞热路径优化（2026-10-23～24）

### 实现

- 新增持久化四路 SoA 接触约束，覆盖 warm start、push、solve、restitution 和冲量回存；
  凸体和网格多 manifold 接触都使用这条路径，默认开启。每个完整物理步只打包一次，
  所有子步复用；回存直接从 SIMD lane 写回 Contact，不再每个阶段复制世界数组。
- 仅对图着色保证动态刚体互不冲突的颜色使用 SIMD。overflow 保持顺序求解，
  不足四个约束的尾批次用掩码处理；不存在的 manifold lane 与真实的零点 manifold 分开标记。
- 矩阵乘法保持标量求值括号顺序。SIMD 比较/选择保持 min/max 的操作数规则，
  取负用符号位翻转，绕过 Swift SIMD 一元负号丢失 signed zero 的行为。
- 全部八种关节的 warm start/solve 每个颜色只借出一次 bodyStates 缓冲区，
  避免每个约束反复读取和修改 SolverSet。motor warm start 保持原有运动学刚体写入语义。
- 接触回收、凸体 manifold 更新及冲量存储借用原地数据，减少 Contact 引用字段的 ARC 和 manifold CoW。
- 凸体/三角形裁剪的双缓冲区、凸体点归约缓冲区及凸体接触更新的四点 scratch 改用 InlineArray 栈存储。
  指针仅在借用闭包内使用。
- island 全量结构检查仅在 Debug 执行。

保持原有 4 子步、求解顺序和浮点运算顺序。可设置 `world.enableWideSolver = false`
使用标量路径；基准程序支持 `NATSEL_WIDE=0` 与 `NATSEL_WIDE=1` 显式切换，
`NATSEL_HASH=1` 在计时区间之后输出状态哈希。

### 复测方法

Apple M1、macOS 27.0.1、Swift 6.4.1，标准 `swift build -c release`，单线程、4 子步。
修改前基于提交 `1600740b69e2ab96bce9fef941a25cc35192a7ef` 保留 Release 可执行文件。
测试完成后重新执行 `swift build -c release`，避免使用测试构建设置的基准产物。
逐场景交替执行修改前/后版本，各 3 次取最小值；第二次交换前后顺序。
世界创建和最终状态哈希均在计时区间之外。

复测脚本：`Tools/Benchmarks/compare.py`。例如：

```sh
swift build -c release
python3 Tools/Benchmarks/compare.py /path/to/Benchmarks-before .build/release/Benchmarks \
  --repeats 3 --output /tmp/natsel-comparison
```

脚本保存每次完整输出与 `comparison.json`。本机存在其他并行编译任务，绝对耗时有波动；
因此不把第一节历史 C 时间与本轮 Swift 时间混算成新的 C/Swift 倍数。

### 第一轮实测（不含后续 POD 存储改动）

| 场景 | 修改前 ms | 第一轮后 ms | 提速 |
|---|---:|---:|---:|
| large_pyramid | 3667.8 | 2925.5 | 1.25x |
| many_pyramids | 3649.8 | 2796.9 | 1.30x |
| joint_grid | 3943.4 | 4014.5 | 0.98x |
| spinner | 13688.6 | 13235.6 | 1.03x |
| sleep | 2587.4 | 1911.2 | 1.35x |
| trees25 | 1817.0 | 1656.9 | 1.10x |
| trees50 | 757.8 | 705.6 | 1.07x |
| trees100 | 403.2 | 396.7 | 1.02x |
| washer | 67907.6 | 56681.3 | 1.20x |
| convex_pile | 18621.7 | 15284.5 | 1.22x |
| **合计** | **117044.2** | **99608.6** | **1.18x** |

全量原始数值见 `Tools/Benchmarks/Results/round1-2026-10-24.json`。
同日 C 四宽参考合计 **25878.8 ms**，见 `c-reference-2026-10-24.json`。
第一轮 Swift/C 为 **3.85x**；继续优化的验收上限为同机 C 耗时的 **1.10x**，尚未达到。

### 验证与边界

`swift test` 和 `swift test -c release` 均通过 **386 项测试、110 个 suite**。

原有 C 参考确定性测试保留，包括波浪地形、网格、ragdoll、滚动阻力与冲量哈希。
新增逐步 scalar/SIMD 对照、各求解阶段对照、混合反弹与运动学接触、warm starting 开关、
尾批次、overflow 顺序，以及 signed zero/NaN 操作数位模式回归。

第一轮时 Contact 仍含引用字段，关节数学求解仍为标量；后续进展见第七节。
当前准备阶段也保留标量约束和 SIMD 约束两份数据，以复用准备逻辑及标量回退，
代价是额外的约束内存与一次打包。尚不能据此宣称达到 C 的 5% 以内目标。


## 七、继续优化：Swift/C ≤ 1.10（进行中）

同机 C 四宽参考总计 25878.8 ms，因此完整十场景的 Swift 验收上限为 **28466.6 ms**。
第一轮 Swift 总计 99608.6 ms，尚未达到。当前目标保持不变，不能用单场景或局部测试代替全量达标。

新增实现：

- JointSim 通过 POD 枚举共享八种关节的有效载荷，stride 降至 **444 字节**。
- Contact 元数据为 **200 字节的 BitwiseCopyable 值**。世界持有稳定地址的 payload slab；
  单 manifold 内联，多 manifold 才使用数组，Contact 复制不再保留数组。热点视图直接返回稳定地址，
  避免自定义 `_read` / `_modify` 下标的协程帧分配。查询视图沿用 C 的下一次世界修改前有效的语义。
- 树查询的 512 项遍历栈使用未初始化的临时存储；默认材质混合内联，仍支持自定义回调。
- SAT 支持点查询和 face dots 使用四路 SIMD；索引的低位嵌入、近似并列选择及求值顺序保持一致。
  SAT 的 25 组 scratch 共用未初始化临时 slab，逐项先写后读。
- 无角向弹簧、马达和限制的球形关节使用四路 point-to-point 求解，按连续独立约束分组，
  overflow 与复杂关节保留标量路径；仍保留关闭角向功能之后的缓存 warm-start 冲量。
- 网格候选三角形的三个变换顶点内联；局部接触点填入一份 arena，全部写完后再生成共享只读切片，
  避免每个三角形单独分配点数组以及写入期间的 arena CoW。

已完成的 POD/SAT 生产版单次检查（**不含随后新增的关节 SIMD 与网格 arena**）：

| 场景 | Swift ms | 状态哈希 |
|---|---:|---|
| large_pyramid | 2814.7 | 6cff0dc45097a74b |
| joint_grid | 2270.7 | ee8b3ae184f13ac5 |
| convex_pile | 14015.4 | dea6ead80e6871d7 |
| washer | 51137.4 | 467e0d32bfbea71c |

这些哈希与第一轮对应场景一致，原始数值见
`Tools/Benchmarks/Results/storage-sat-checkpoint-2026-10-24.json`。这是阶段检查，不能当作十场景总计。

POD/SAT 版本 Release 全量 391 项测试通过。关节 SIMD 与网格 arena 加入后 Debug 全量
392 项测试通过；最新版本的 Release、Address Sanitizer 和生产性能测量正在验证。

后续重点是去掉标量约束再打包为 SIMD 的双重准备成本，并继续压低网格处理及窄相开销。
在十场景复测证明 Swift/C ≤ 1.10 之前，目标不算完成。

## 八、Builtin 后续优化与完整复测（2026-10-24）

本节为当前工作树的最新生产构建结果。第六、七节保留阶段记录，不代表当前性能。

本轮以已经包含 `FloatW` Builtin 向量运算、直接 SoA 准备和持久化碰撞 scratch 的工作树为起点，继续处理以下开销：

- 四路刚体状态改为直接构造全部字段，消除先默认初始化再覆盖的数据。生产汇编中 push 内核从约 693 条减到 631 条，solve 从约 1090 条减到 1025 条，收集状态时不再调用默认构造器。
- finalize 一次借出整个 awake 刚体 sim/state 缓冲区，CCD 复用同一缓冲区，减少每个刚体的 Array 唯一性检查；刚体处理和 CCD 顺序保持不变。
- prepare 的四个 lane 使用常量下标，避免动态 SIMD 下标导致的中间落栈。
- 网格四点归约复用 worker 投影 scratch，并原地缩减 cluster 点数组，保留 C 的容差、并列选择和 swap-remove 顺序。
- 四元数旋转、四元数乘法及矩阵乘法显式内联，减少 finalize 和窄相中的调用；未改变算式或引入 FMA。
- 基准程序在计时前解析环境变量，避免每步构造诊断环境字典。这是计时辅助开销的清理，不能全部算作物理引擎的提速。

### 最新十场景 Swift/C 对照

Apple M1、Swift 6.4.1、生产 Release、单 worker、四路 SIMD、4 子步。每场景分别交替运行 C 和 Swift 三次，取各自最小值。C 使用同一已构建的 `-O3 -DNDEBUG -ffp-contract=off` 静态库；未把 C 求解器链接进 Swift。

| 场景 | C ms | Swift ms | Swift/C |
|---|---:|---:|---:|
| large_pyramid | 1231.3 | 1795.1 | 1.458x |
| many_pyramids | 1418.5 | 1965.0 | 1.385x |
| joint_grid | 821.2 | 736.7 | 0.897x |
| spinner | 2252.9 | 3555.6 | 1.578x |
| sleep | 542.0 | 820.5 | 1.514x |
| trees25 | 333.8 | 479.5 | 1.437x |
| trees50 | 158.9 | 234.4 | 1.475x |
| trees100 | 96.3 | 147.2 | 1.529x |
| washer | 15618.9 | 21099.5 | 1.351x |
| convex_pile | 4912.1 | 6944.6 | 1.414x |
| **合计** | **27386.0** | **37778.2** | **1.379x** |

**整体仍慢约 37.9%，未达到 Swift/C ≤ 1.10。** joint_grid 已快于 C，其余九个场景仍未达标。washer 为约 1.351x，不能用它或局部求解器的结果替代完整十场景验收。

十场景每个版本各运行三次，60 个最终状态哈希逐场景一致。washer 为 `fad87a98c0b30df`。哈希覆盖 C 的世界状态字段；接触点/冲量等更细的行为还由测试套件中的 C 参考与 scalar/wide 对照检查。

完整时间、每轮哈希、Swift 阶段耗时、二进制 SHA-256 与本轮阶段检查保存在 `Tools/Benchmarks/Results/builtin-followup-2026-10-24.json`。当前原始运行日志在 `/tmp/natsel-next/final-validation/`。

### 复测工具

新增 `Tools/Benchmarks/reference.c`，它只包装上游 benchmark 的 world 销毁以在计时之外打印 `b3HashWorldState`；`validate.py` 同时验证时间和哈希，固定 C 单线程与 SIMD width 4，并清除继承的 NATSEL 调试覆盖。

```sh
B3_CHECKOUT=/Users/cyke666/dev/box3d-main
clang -O3 -DNDEBUG -DBOX3D_INTERNAL_BENCHMARKS -std=c17 -ffp-contract=off \
  "-DNATSEL_BOX3D_BENCHMARK_MAIN=\"$B3_CHECKOUT/benchmark/main.c\"" \
  -I"$B3_CHECKOUT/include" -I"$B3_CHECKOUT/src" \
  -I"$B3_CHECKOUT/build-bench/src" -I"$B3_CHECKOUT/shared" \
  Tools/Benchmarks/reference.c \
  "$B3_CHECKOUT/build-bench/shared/libshared.a" \
  "$B3_CHECKOUT/build-bench/src/libbox3d.a" -o /tmp/natsel-c-reference
swift build -c release
python3 Tools/Benchmarks/validate.py /tmp/natsel-c-reference .build/release/Benchmarks \
  --repeats 3 --output /tmp/natsel-c-swift-validation
```

### 后续热点

仍需压低接触约束准备/求解、凸体碰撞与 CCD 的开销。最新 washer 最快一次的阶段平均值约为：

| 阶段 | Swift ms/step |
|---|---:|
| pairs | 2.608 |
| collide | 9.481 |
| prepare | 2.445 |
| warm | 0.478 |
| push | 1.455 |
| solve | 2.003 |
| store | 0.708 |
| finalize | 1.540 |

本轮内存结构修改后 Debug 与 Release 各 395 项测试通过，Address Sanitizer 的 39 项相关测试通过。随后只调整了数学内联与基准配置解析；最终全量测试及生产构建正在复核。

## 九、Box3D 之外的新系统（2026-10-24）

新增载具、软体/布料、角色控制器、浮力和四类关节后，原有十个场景不受影响：这些阶段在没有对应对象时
直接返回，`JointSim` 仍为 444 字节（新载荷最大为齿轮关节 252 字节，小于球关节的 260 字节；六自由度
关节的逐轴状态放在独立表中）。与 HEAD 同机交替测量：joint_grid 745.4 / 745.1 ms，many_pyramids
2034.8 / 2034.9 ms，large_pyramid 1911 / 1889 ms（噪声范围内）。

新场景（`Sources/Benchmarks/FeatureScenes.swift`，`swift build -c release`，3 次取最快，`NATSEL_WORKERS`
设置线程数）：

| 场景 | 内容 | 1 线程 ms/step | 8 线程 ms/step |
|---|---|---:|---:|
| vehicles | 64 辆四驱车，防倾杆，圆柱轮投射，起伏网格地形，600 步 | 1.19 | 0.71 |
| cloth | 8 块 40×40 布料（二面角弯曲），覆盖球体并与动态箱子耦合，300 步 | 4.46 | 1.09 |
| soft_cubes | 27 个 6×6×6 四面体软体方块堆叠，300 步 | 2.22 | 0.56 |

### 做过的优化

1. **软体约束 4 宽批处理。** 按锚点向外排序的边约束相邻两条共享顶点，每条都要等上一条的开方与除法，
   单条约 17 ns。现在在 64 条窗口内贪心组成顶点互不相交的 4 条一批，用 `SIMD4` 求解，批内结果与逐条
   求解相同。布料 3304 → 1338 ms（2.5x），软体方块 1627 → 667 ms（2.4x）。二面角弯曲约 25 ns/条，
   仍是布料最大开销。
2. **软体按对象并行。** 每个软体的碰撞检测与求解只读世界、只写自身记录；刚体速度变化在之后按软体
   顺序串行合并，所以结果与线程数无关（有测试覆盖）。8 线程约 4x。
3. **载具车轮状态改为纯数据。** `VehicleWheel` 原先内嵌带数组的轮胎曲线，每次拷贝都要 retain/release
   并 memmove 约 400 字节。曲线移到旁表，求解器状态改为 `BitwiseCopyable`；离合器耦合与圆柱投射代理
   不再每子步分配堆内存；记录用 swap 移出而非拷贝，避免写时复制。1158 → 713 ms。
4. **车轮投射并行。** 剩余开销主要是每帧 256 次圆柱投射（约 3.5 µs/次，GJK 对网格三角形）。投射只读
   世界，现按载具并行；唤醒与保持清醒仍串行。8 线程 713 → 423 ms，结果与串行逐位一致（有测试覆盖）。
   约束求解本身每步约 0.2 ms。

## 十、多线程约束求解器（2026-10-24）

按 C 版 `b3SolverStage` 实现：`workerCount > 1` 时，每步唤醒常驻线程池（`Base/WorkerPool.swift`），
覆盖接触/关节准备、子步求解、恢复系数、冲量存储和刚体收尾。线程在阶段之间自旋等待阶段号，求解器外
睡在信号量上。

| 场景 | 1 线程 ms | 2 线程 ms | 4 线程 ms | 加速（4 线程） |
|---|---:|---:|---:|---:|
| large_pyramid | 1668 | 1032 | 640 | 2.6x |
| many_pyramids | 1863 | 1149 | 768 | 2.4x |
| joint_grid | 754 | 477 | 349 | 2.2x |

Apple M1（4 性能核 + 4 能效核）、`swift build -c release`、4 子步，每项 3 次取最快；机器上同时
开着 UI，数值有几个百分点的噪声。`NATSEL_WORKERS=8` 与 4 相同，因为求解器线程数上限是性能核数。

### 设计要点

1. **确定性。** 同一图着色内的约束不共享动态刚体，所以按任意方式切块结果都一样；溢出约束和载具会
   碰任意刚体，在颜色之间由主线程按串行顺序执行。任意线程数下结果与单线程逐位一致，步与步之间改
   `workerCount` 也安全（`ParallelSolverTests` 覆盖七个场景）。
2. **两遍准备。** 串行规划用前缀和给每个接触分配约束槽位与流形偏移，再并行填充；流形池保留历史最大
   容量，不再每步重新分配。关节先跨颜色展平并行准备，再并行打包成宽批次。
3. **刚体收尾并行。** 变换、睡眠速度、移动事件、惯量和形状包围盒并行计算；岛的保持清醒位和拆岛候选
   写入每线程数据，再按串行的平局规则合并。快速刚体的连续碰撞、子弹列表和 `markProxyMoved` 之后按
   刚体顺序串行执行，所以宽相位移动缓冲的顺序不变。large_pyramid 4 线程 834 → 640 ms。
4. **线程数上限为性能核数。** 阶段之间是屏障，能效核上的线程会拖住每个阶段：M1 上 8 线程原本比 4
   线程慢。
5. **唤醒延迟。** 从信号量唤醒要几十微秒，第一个阶段原本几乎全由主线程做完。`begin` 现在等所有线程
   报到（约 40 µs，每步一次），关节准备 1.34 → 0.38 ms。自旋用 `yield` 指令提示而非 `sched_yield`。
6. **Swift 注意事项。** 工作线程上不能对类属性数组调用 `withUnsafeMutableBufferPointer`（动态独占
   检查会冲突），也要避免跨核的数组 retain/release；阶段代码一律用主线程预先取好的裸指针。

### 剩余串行部分

- 接触准备的规划遍（约 0.45 ms/步，large_pyramid）。
- 碰撞阶段仍用自己的 `concurrentPerform`，没有并入线程池，扩展性一般。
- 宽相位更新、岛睡眠与拆岛、事件发布。

## 十一、第二轮缺口：路径约束、状态快照、骨骼布娃娃、软体互碰（2026-10-24）

新场景（`Sources/Benchmarks/FeatureScenes.swift`，Apple M1，`swift build -c release`，4 子步，
3 次取最快，`NATSEL_WORKERS` 设线程数；机器开着 UI，有几个百分点噪声）：

| 场景 | 内容 | 1 线程 ms | 4 线程 ms |
|---|---|---:|---:|
| path_beads | 64 条倾斜闭环 × 16 个珠子：1024 个路径约束，速度马达 / 摩擦 / 四种旋转约束，珠子互相碰撞，300 步 | 858 | 279 |
| ragdoll_motors | 100 个骨骼布娃娃（11 节），每步采样走路动画并用关节马达驱动，300 步 | 1297 | 553 |
| rollback | large_pyramid 每步存一次快照，每 30 步回滚 8 步并重算，200 步 | 2436 | 1117 |
| soft_cubes | 27 个体积软体方块堆叠，现在互相碰撞，300 步 | 1463 | 1037 |
| cloth_stack | 6 块 30×30 布料叠落在球上，自碰撞 + 布料互碰，300 步 | 4872 | 2586 |
| cloth | 8 块 40×40 布料（互不接触），对照 | 1346 | 340 |

### 说明

1. **快照开销。** rollback 比 large_pyramid（1668 ms）多出的约 770 ms 里，约 42 步是回滚后的重算，
   其余是每步一次 `saveState`：约 2 ms 一次（5050 个刚体、全部接触流形与树）。数组是写时复制，
   快照本身只复制手动内存容器（id 池、位集、哈希集）和接触载荷。
2. **soft_cubes 变慢是功能变化。** 之前 667 ms 时方块互相穿透；现在 27 个方块互相接触，成为一个
   接触组逐子步一起求解。组内每个软体的约束和刚体碰撞在线程上并行，组内接触投影仍是串行的。
3. **软体接触检测的三步优化。** cloth_stack 初版 12051 ms，其中检测占大头：
   - 每个目标体只建一次网格，而不是每对重建；
   - 每个顶点只保留最近的 4 个三角形，自碰撞排除静止姿态下两条平均边长以内的配对（同一层的邻域，
     不是折叠）：每步接触从约 13 万降到约 2 万；
   - 特征只按自身顶点的位移加厚度扩展，顶点按自身位移查询格子，慢顶点几乎不查别的格子；
     检测按体对并行。最终 4872 ms（单线程），4 线程 2586 ms。
4. **能量问题。** 子步开始时已在厚度内的顶点若一次推出，XPBD 会把推出量变成速度，27 个软方块
   堆在约 240 步后爆开。现在把每子步的穿透恢复限制在 `contactSpeed`（默认 3 m/s）以内。
5. **顺带修掉的泄漏。** 碰撞阶段和传感器阶段每步分配一个位集从不释放（小场景每步 8 字节，
   随接触数和传感器数增长）。

## 十二、软体边碰撞（2026-10-25）

边对边、边对刚体细特征的接触默认开启（`enableEdgeCollision`）。Apple M1，`swift build -c release`，
4 子步，300 步，2 次取最快：

| 场景 | 无边碰撞 1 线程 | 有边碰撞 1 线程 | 有边碰撞 4 线程 |
|---|---:|---:|---:|
| cloth（8 块布落在球和箱子上） | 1346 | 1710 | 438 |
| soft_cubes（27 个软方块堆叠互碰） | 1463 | 2424 | 1567 |
| cloth_stack（6 块布叠落，自碰撞 + 互碰） | 4872 | 12121 | 5295 |

### 过程

1. **初版 cloth_stack 86 s。** 每条边的候选都按整步最大位移的球形扩展，下落的布每条边要测上百条边。
   改成扫掠盒（从当前位置沿速度到步末，再加四分之一位移和重力增量的余量）后候选少了数倍。
2. **刚体边接触。** 两端顶点已经贴在同一形状上且法线夹角小于约 8° 时跳过该边（中间的表面在这个
   尺度上是平的，细特征塞不进去）。布料堆上的刚体边接触从约 3000 降到 100～600。
3. **能量注入。** 叠放的布在"静止"后仍有顶点以 6～19 m/s 抖动，检测越来越慢（第 250 步约 900 ms）。
   原因是每步按当前位置重新判断接触在哪一侧：略微压进厚度内的配对会读错侧，然后被推穿。
   顶点-面接触改为：读数清楚就按几何判断，只有读数模糊（小于半个厚度）时沿用上一步记住的侧；
   边-边接触在读数模糊时这一步直接跳过，交给顶点-面接触。修完后 52 s → 12 s。
4. **为什么边-边不能用历史。** 边-边的公共法线是两条边方向的叉积，两条边转过平行时法线会翻转；
   折叠的布这种情况很常见，记住的侧（不论存符号还是存世界方向）都会让折叠的布条穿过自己。
5. **自碰撞排除表。** 每个顶点要忽略的面、每条边要忽略的边，在创建软体时（或改自碰撞 / 厚度时）
   用静止姿态算一次，每块 30×30 布约 19 ms，避免第一步卡顿。

剩余开销主要在叠放布料的边-边候选测试；需要更快时可对单个软体关闭 `enableEdgeCollision`。


## 双精度大世界

世界位置 `Pos` 使用三个 `Double`，世界变换使用双精度平移和单精度旋转。碰撞、关节、
速度及 SIMD 求解器仍使用局部单精度数据；先在双精度中减去参考位置，再进入单精度计算。
软体采用双精度原点和单精度顶点偏移；`withSoftBodyLocalVertexPositions` 可直接读取顶点缓冲区，
避免每帧构造双精度世界顶点数组。

宽阶段树仍使用向外舍入的浮点包围盒，保留紧凑节点。极远坐标处包围盒会扩大，候选数可能增加，
因此双精度世界坐标不意味着宽阶段成本与原点场景完全相同。精确包围盒使用 `WorldAABB` 及
`shapeWorldAABB` / `bodyWorldAABB` / `softBodyWorldAABB` 获取。局部形状与超长关节仍受单精度限制；
在 `10^12` 世界单位处，双精度坐标自身的间距约为 `0.000122`。

2026-10-25，Apple M1 / Swift 6.4.1，Release，`NATSEL_WORKERS=4`。以修改前的 `5187e6a`
独立源码快照为基线；编译完成后交替运行两个版本，每个场景各测三次，取最小耗时。以下为原点附近
的整场景步进耗时，不包含建场景时间；不是极远坐标下的宽阶段压力测试。

| 场景 | 步数 | 原单精度世界位置 | 双精度世界位置 | 耗时变化 |
| --- | ---: | ---: | ---: | ---: |
| large_pyramid | 200 | 624.74 ms | 645.33 ms | +3.3% |
| joint_grid | 100 | 341.82 ms | 344.35 ms | +0.7% |
| cloth | 300 | 426.50 ms | 432.07 ms | +1.3% |

Debug 与 Release 各通过 492 项测试。复杂形状轨迹和接触哈希由独立 C 双精度构建生成；
多线程结果继续逐位一致。浮点树包围盒的舍入中点补偿，以及软体分组的双精度包围盒检查，
分别避免远处查询漏检和不相关软体误合组。

## 碰撞质量与软体空间索引（2026-10-26）

本轮增加可选的动态配对 CCD、软体顶点多平面接触、凸 Compound 的动态实例化，以及软体分组和射线索引。
`enableDynamicContinuous` 默认关闭，以保留原有 Box3D 轨迹；打开后增加双精度扫掠配对和 TOI 冲量处理。
原有十场景的 C/Swift 性能总表本轮没有重测，不能用下面的专项测量替代它。

Apple M1、Swift 6.4.1、Release、单 worker。`soft_queries` 每步对一个 64×64 顶点、7,938 个面的软体
投射 512 条射线；模拟每帧更新顶点版本，因此计时包含 BVH 重建。创建和首步预热在计时之外，计时共 29 步。
同一个二进制交替开启/关闭 BVH，各运行三次：

| 模式 | 第一次 ms | 第二次 ms | 第三次 ms | 最快 ms |
| --- | ---: | ---: | ---: | ---: |
| BVH | 147.75 | 130.52 | 130.14 | 130.14 |
| 逐面扫描 | 553.58 | 551.86 | 551.49 | 551.49 |

最快值之比约 **4.24×**。六次命中结果的校验值均为 `60958409.99992585`。这反映重复射线查询的收益；
每帧只有少量射线时，重建成本可能高于扫描，可设置 `world.enableSoftBodyQueryAcceleration = false`。

`sparse_soft` 包含 2,000 个相互分离、保持唤醒的软体，位于 `10^12` 附近。99 个计时步的三次结果为
313.10 / 317.45 / 313.22 ms，最快约 **3.16 ms/步**。这是当前版本绝对耗时，没有测修改前同场景的耗时。
分组的扫描算法另有确定性的工作量检查：沿任一主轴分离的 5,000 个包围盒，候选测试为 0；原逐对枚举
需要 12,497,500 次。重叠场景仍可能产生二次数量的候选。

复测：

```sh
swift build -c release
NATSEL_WORKERS=1 NATSEL_SOFT_QUERY_BVH=1 .build/release/Benchmarks --benchmark=soft_queries --repeats=3
NATSEL_WORKERS=1 NATSEL_SOFT_QUERY_BVH=0 .build/release/Benchmarks --benchmark=soft_queries --repeats=3
NATSEL_WORKERS=1 .build/release/Benchmarks --benchmark=sparse_soft --repeats=3
```

完整时间、校验值和二进制 SHA-256 保存在 `Tools/Benchmarks/Results/quality-2026-10-26.json`。
本轮的碰撞边界、接口和剩余限制见 `PORTING.md` 的 “Collision quality and spatial indexing”。

## 大型布料自碰撞（2026-10-26）

当前按独立引擎方向优化：旧 C 哈希用于诊断，同版本、不同 worker 数的确定性和物理质量仍需通过测试。
面与边的扫掠包围盒改用可重拟合 BVH，避免跨多个网格单元重复插入和排序；静止邻域排除缓存用于
提前剪枝。大型单软体的候选检测可按源顶点/边分段并行，按固定段序合并，约束求解顺序保持一致。

Apple M1 / macOS arm64 / Swift 6.4.1，Release，`cloth_self64`。该场景按原始顶点顺序创建
64×64 布料，固定一行、开启自碰撞，5 次软体迭代，每次世界步进为 1/60 秒、4 个刚体子步。
先预热 60 步，再测量 180 步。优化前单 worker 测一次；优化后各 worker 配置测三次，表中为
三次运行各自 P95 的中位数，不是每步平均耗时。

| 版本 / workers | 步进 P95 | 相对优化前 |
| --- | ---: | ---: |
| 优化前 / 1 | 39.302 ms | 1× |
| 优化后 / 1 | 15.178 ms | 2.59× |
| 优化后 / 4 | 8.476 ms | 4.64× |
| 优化后 / 7 | 6.321 ms | 6.22× |

所有运行最终顶点位置校验值均为 `772246b7f26de325`；软体专用校验值由 `NATSEL_HASH=1` 输出。
原 `worldStateHash` 不覆盖纯软体顶点，不能用于此场景的结果验证。完整 518 项测试通过，其中
新增单布料在 1/2/4/8 workers 下顶点逐位一致的回归检查。该场景尚未达到 GuavaStudio 为
120 FPS 分配的 1 ms 完整物理 CPU 预算；这里没有测量游戏逻辑或渲染，不能据此保证整帧帧率。

```sh
swift build -c release --product Benchmarks
NATSEL_WORKERS=1 NATSEL_HASH=1 .build/release/Benchmarks --benchmark=cloth_self64 --repeats=3 --record-steps
NATSEL_WORKERS=4 NATSEL_HASH=1 .build/release/Benchmarks --benchmark=cloth_self64 --repeats=3 --record-steps
NATSEL_WORKERS=7 NATSEL_HASH=1 .build/release/Benchmarks --benchmark=cloth_self64 --repeats=3 --record-steps
```
