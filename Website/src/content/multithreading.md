# 多线程与性能

Natsel 的求解器会并行处理互相独立的约束。这一页讲三件实用的事：怎么配置线程数、怎么用 profile 找到瓶颈、以及为什么开更多线程不会改变模拟结果。

## 模型

物理步的耗时分成几个阶段，其中**窄相位碰撞检测**和**约束求解**都可以并行：

- 不同的接触对互不依赖，可以分片给多个线程
- 求解按岛屿（island）和约束图着色分批，同一批次内的约束互不影响

因此并行是安全的，但**收益取决于场景规模**。约束太少时，线程调度与同步的开销会超过并行节省的时间，此时单线程反而最快。

这是设计上必须接受的现实：线程数不是"越大越好"的旋钮，而是需要按场景实测的参数。

## 设置线程数

```rust
pub worker_count: i32  // 默认 1
```

这是 `World` 上的**公开字段**，直接赋值即可：

```rust
let mut world = World::default();
world.worker_count = 4;
```

不需要任何初始化调用或回调——线程池在第一次需要时按需创建，之后复用。

| 值 | 行为 |
|----|------|
| `1` | 完全单线程，无同步开销 |
| `2..=8` | 窄相位碰撞与求解阶段并行 |
| 更大 | 参与并行的线程数有上限，多余的不会带来收益 |

线程数有实际封顶：超出的数值会被内部限制到可用并行度。因此不必担心设成 `64` 会创建 64 个线程，但也没有理由设得远超物理核心数。

### 实测的收益拐点

固定场景下增加线程数的效果（同一台机器，每步耗时均值）：

| 刚体数 | 接触数 | 1 线程 | 4 线程 | 8 线程 |
|--------|--------|--------|--------|--------|
| 100 | 352 | 1.61 ms | 2.20 ms | 3.30 ms |
| 400 | 1502 | 6.55 ms | 2.92 ms | 4.83 ms |
| 900 | 3452 | 15.95 ms | 5.46 ms | 5.92 ms |

规律很清楚：

- **小场景（100 刚体）多线程是负收益**，开销超过收益
- **中等场景（400 刚体）4 线程约 2.2 倍加速**，8 线程反而退化
- **大场景（900 刚体）4 线程仍是最佳**，8 线程收益被同步开销吃掉

实践建议：从 `1` 开始，用真实场景逐步加，取实测最快值。4 通常是安全的选择，8 以上很少有意义。

## 宽求解器

```rust
pub enable_wide_solver: bool  // 默认 true
```

控制是否用 SIMD 通道（一次处理 4 个约束）来求解。默认开启。

关键性质：**它与标量路径给出完全相同的结果**。这不是"近似加速"，而是同一套数学的不同执行宽度。因此可以放心地按性能开关它：

```rust
world.enable_wide_solver = false;  // 用于对比标量路径性能
```

只有在诊断性能差异时才需要关闭它。

## 读取 profile

```rust
pub fn profile_data(&self) -> Profile
```

`profile` 也是 `World` 上的公开字段，两种读法等价。`Profile` 的每个字段都是**上一步**的耗时，单位毫秒：

| 字段 | 阶段 |
|------|------|
| `step` | 整步总耗时 |
| `pairs` | 宽相位配对更新 |
| `collide` | 窄相位碰撞检测 |
| `prepare_constraints` | 约束准备 |
| `integrate_velocities` | 子步：速度积分（各子步累加） |
| `warm_start` | 子步：热启动 |
| `solve_impulses` | 子步：冲量求解（各子步累加） |
| `integrate_positions` | 子步：位置积分（各子步累加） |
| `relax_impulses` | 子步：冲量松弛（各子步累加） |
| `restitution` | 恢复系数处理 |
| `store_impulses` | 冲量存储 |
| `transforms` | 位姿回写 |
| `refit` | 宽相位树重建 |
| `bullets` | CCD 扫掠 |
| `sleep_islands` | 睡眠岛屿处理 |
| `sensors` | 传感器重叠 |
| `soft_bodies` | 软体 |
| `vehicles` | 车辆 |

> 注意：`Profile` 里还有 `solve`、`solver_setup`、`constraints`、`split_islands`、`sensor_hits`、`joint_events`、`hit_events` 等字段，但当前版本没有给它们赋值，恒为 `0.0`。判断瓶颈请用上表列出的字段。

子步相关的字段（`integrate_velocities`、`warm_start`、`solve_impulses`、`integrate_positions`、`relax_impulses`）跨所有子步**累加**，所以每步 4 个子步时它们的和会明显大于其他单次阶段。做比较时用同样子步数才有意义。

## 读取计数器

```rust
pub fn counters(&self) -> Counters
```

`Counters` 提供场景规模与内部结构的信息：

| 字段 | 说明 |
|------|------|
| `body_count` | 存活刚体数 |
| `shape_count` | 存活形状数 |
| `contact_count` | 存活接触数 |
| `joint_count` | 存活关节数 |
| `island_count` | 岛屿数 |
| `awake_contact_count` | 清醒且未接触的接触数 |
| `recycled_contact_count` | 复用既有接触的数量 |
| `sat_call_count` | 分离轴测试调用次数 |
| `sat_cache_hit_count` | 其中命中缓存的次数 |
| `static_tree_height` | 静态树高度 |
| `tree_height` | 动态树高度 |
| `color_counts` | 各着色批次的约束数 |

`color_counts` 是长度 24 的数组，每一项是某个批次内的约束（接触 + 关节）数量。它直接反映并行粒度：如果几乎全部约束集中在少数几个批次里，并行就无法充分展开。

`sat_cache_hit_count / sat_call_count` 的比值反映缓存效果，比值低说明形状在频繁改变相对位姿。

## 完整示例

下面同时验证两件事：改变线程数不改变结果，以及如何定位耗时阶段。

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;

/// 搭一个 6x6 的箱子堆，跑固定步数，返回最终状态哈希与平均每步耗时。
fn run(workers: i32) -> (u64, f32) {
    let mut world = World::default();
    world.worker_count = workers;
    world.enable_sleep = false;

    // 地面
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_box_hull(20.0, 0.5, 20.0),
    );

    // 6x6 箱子堆
    let mut box_def = ShapeDef::default();
    box_def.density = 1000.0;
    for i in 0..6 {
        for j in 0..6 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(i as f64 * 1.1 - 3.0, 1.0 + j as f64 * 1.1, 0.0);
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(body, &box_def, &make_box_hull(0.5, 0.5, 0.5));
        }
    }

    let mut total = 0.0f32;
    for _ in 0..180 {
        world.step(1.0 / 60.0, 4);
        total += world.profile_data().step;
    }

    (world.world_state_hash(), total / 180.0)
}

fn main() {
    let (hash1, ms1) = run(1);
    let (hash4, ms4) = run(4);
    let (hash8, ms8) = run(8);

    // 线程数不影响结果
    println!("1 vs 4 结果一致: {}", hash1 == hash4);
    println!("1 vs 8 结果一致: {}", hash1 == hash8);

    println!("平均每步: 1线程 {:.3}ms, 4线程 {:.3}ms, 8线程 {:.3}ms", ms1, ms4, ms8);

    // 定位瓶颈：取一次运行的 profile
    let mut world = World::default();
    world.worker_count = 4;
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_box_hull(20.0, 0.5, 20.0),
    );
    let mut box_def = ShapeDef::default();
    box_def.density = 1000.0;
    for i in 0..6 {
        for j in 0..6 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(i as f64 * 1.1 - 3.0, 1.0 + j as f64 * 1.1, 0.0);
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(body, &box_def, &make_box_hull(0.5, 0.5, 0.5));
        }
    }
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
    }

    let p = world.profile_data();
    println!(
        "总计 {:.3}ms | 配对 {:.3} | 碰撞 {:.3} | 准备 {:.3} | 求解 {:.3} | 位置积分 {:.3} | 回写 {:.3}",
        p.step, p.pairs, p.collide, p.prepare_constraints,
        p.solve_impulses, p.integrate_positions, p.transforms
    );

    let c = world.counters();
    println!(
        "刚体 {} 形状 {} 接触 {} 岛屿 {} 清醒接触 {} 动态树高 {}",
        c.body_count, c.shape_count, c.contact_count,
        c.island_count, c.awake_contact_count, c.tree_height
    );
    println!("前 6 个着色批次的约束数: {:?}", &c.color_counts[..6]);
}
```

## 用基准程序实测

仓库附带基准程序，用于在真实场景上比较配置：

```sh
# 列出全部场景
cargo run --release --bin benchmarks -- --list

# 跑单个场景
cargo run --release --bin benchmarks -- --benchmark=large_pyramid

# 跑全部场景
cargo run --release --bin benchmarks -- --all

# 重复次数，默认 4
cargo run --release --bin benchmarks -- --benchmark=large_pyramid -r=5
```

不带 `--all` 也不带 `--benchmark` 时只跑默认的几个场景；`-r` / `--repeats` 默认是 4 次。`-b=NAME` 与 `--benchmark=NAME` 等价，也可传场景编号（用 `--list` 查看）。

用环境变量覆盖配置，无需改代码：

```sh
NATSEL_WORKERS=4 cargo run --release --bin benchmarks -- --benchmark=large_pyramid
NATSEL_WIDE=0  cargo run --release --bin benchmarks -- --benchmark=large_pyramid
```

| 变量 | 作用 |
|------|------|
| `NATSEL_WORKERS` | 设置 `worker_count` |
| `NATSEL_WIDE` | `1` 开启宽求解器，`0` 关闭 |
| `NATSEL_PHASE` | `1` 时输出各阶段耗时 |
| `NATSEL_HASH` | `1` 时输出状态哈希，用于比对确定性 |
| `NATSEL_CCD` | `0` 时关闭连续碰撞检测 |

典型的排查流程：

```sh
NATSEL_WORKERS=1 NATSEL_HASH=1 cargo run --release --bin benchmarks -- --benchmark=large_pyramid
NATSEL_WORKERS=4 NATSEL_HASH=1 cargo run --release --bin benchmarks -- --benchmark=large_pyramid
```

两次输出的状态哈希必须相同。若不同，说明配置影响到了结果——这在当前版本中不应发生，属于需要报告的问题。

务必用 `--release`。debug 构建的物理耗时会高出一个数量级，且不能反映真实性能。

## 为什么线程数不改变结果

这是 Natsel 的核心保证：**同一份输入、同一版本与配置，在不同线程数下产生相同的结果**。

实现上依赖三点：

1. **按岛屿划分**：互不耦合的刚体组独立求解，组之间没有数据竞争
2. **约束图着色**：同一批次求解的约束作用在不同刚体上，顺序不影响结果
3. **固定顺序的合并**：各线程的结果按索引顺序合并，而不是按完成顺序

因此并行只是"同时做同一件工作"，不是"改变做事的顺序"。

这一点有实际意义：你可以在低端设备上少开线程、在高性能设备上多开，而不必担心两端模拟产生分歧（例如多人同步或录像回放）。

想自己验证确定性，用 `world_state_hash()`：

```rust
let mut world = World::default();
world.worker_count = 8;
// ... 搭建场景 ...
for _ in 0..600 {
    world.step(1.0 / 60.0, 4);
}
let hash_with_8_threads = world.world_state_hash();
```

把 `worker_count` 改成 `1` 重跑同样步数，两个哈希必须完全相同。

> `enable_wide_solver` 同样不影响结果，也可用这个方式验证。

## 常见问题

**多线程反而更慢。** 场景约束太少时，线程同步开销超过并行收益。按上文表格的思路实测：小场景用 `1`，中等以上试 `4`。

**改了 `worker_count` 哈希变了。** 不应该发生。先确认两次运行的其余配置完全一致（同样的子步数、同样的 `enable_wide_solver`、同样的 `safety_factor`），再检查是否用了自定义回调。

**`enable_dynamic_continuous` 或 `is_bullet` 会影响性能。** 它们让每步都做扫掠检测。若 `Profile.bullets` 偏高，检查是否有过多子弹标记。

**`Profile` 里部分字段恒为 0。** `solve`、`solver_setup`、`constraints`、`split_islands`、`sensor_hits`、`joint_events`、`hit_events` 当前未赋值。用 `step`、`pairs`、`collide`、`prepare_constraints`、`solve_impulses` 等实际有值的字段定位瓶颈。

**子步数会成倍影响耗时。** `world.step(dt, sub_step_count)` 的第二个参数直接决定子步循环次数，求解阶段耗时大致线性增长。4 是常用的平衡点；提高到 8 能改善堆叠稳定性但明显变慢。

**不要用 debug 构建评估性能。** 用 `--release`，并注意 LTO 与 `codegen-units` 已在此 profile 中配置好。

**线程池是惰性创建的。** 改成 `1` 时不会创建线程池；从 `1` 改到 `4` 会在下次 `step` 时创建。因此第一次切换的耗时包含创建成本，测量时先跑几十步预热。

## 相关页面

- [确定性](/docs/determinism) —— 线程数无关性的完整说明
- [连续碰撞检测](/docs/ccd) —— CCD 的性能开销
- [核心概念](/docs/concepts) —— 岛屿与求解流程概览
- [架构](/docs/architecture) —— 物理步的阶段划分
