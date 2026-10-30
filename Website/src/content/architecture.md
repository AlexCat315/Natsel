# 架构总览

本页从**使用者视角**说明 Natsel 的组织方式：它提供了哪些能力、这些能力在 `World` 上以哪几类 API 呈现、典型游戏循环里该怎么排布调用顺序，以及源码目录的大致分工。

## 一句话结构

Natsel 是一个 `std`-only、零第三方依赖的 Rust crate。你只与一个类型打交道：`World`。所有功能都是 `World` 上的方法，或者由 `World` 创建并持有的对象（角色、布娃娃）。

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main" }
```

```rust
use natsel::simulation::World;

let mut world = World::default();
```

## 按能力划分的模块

|crate 模块 | 提供的能力 | 你什么时候直接用到 |
|-----------|-----------|-------------------|
| `base` | 版本号、断言与日志钩子、内存统计、长度单位 | 启动时设置单位，或接入自己的断言处理 |
| `math` | `Vec2`/`Vec3`/`Pos`/`Quat`/`Transform`/`Matrix3`/`AABB`/`Plane`，四元数与向量运算 | 全程都用 |
| `containers` | 内部存储结构 | 一般不需要直接使用 |
| `types` | 全部定义结构体：`BodyDef`、`ShapeDef`、`WorldDef`、`JointDef`、查询与事件类型 | 全程都用 |
| `collision` | 几何构建与底层碰撞算法：凸包构建、网格烘焙、高度场、射线/形状投射 | 构建几何体（`make_box_hull`、`TriangleMesh::new`、`Compound::new`） |
| `simulation` | `World` 本体：刚体、形状、接触、关节、事件、快照、查询 | 主入口 |
| `character` | `CharacterVirtual` 角色控制器 | 做第一/第三人称角色时 |
| `vehicle` | 车辆系统：悬挂、轮胎、传动 | 做载具时 |
| `skeleton` | `Skeleton`、`Ragdoll`、动画采样 | 做布娃娃时 |
| `soft_body` | 软体与布料 | 做布料、果冻、可变形物体时 |

顶层还导出了几个便捷函数：

```rust
use natsel::{get_version, is_double_precision, get_max_manifold_points};

let v = get_version();                 // Version { major: 0, minor: 2, revision: 0 }
assert!(is_double_precision());        // 世界坐标是 f64
let max_points = get_max_manifold_points();  // 4，每个接触最多几个接触点
```

## 一个游戏循环里的调用顺序

物理循环有严格的阶段划分。事件读取必须在 `step` 之后立刻进行，因为缓冲在下一步开始时清空。

```rust
use natsel::simulation::World;

let mut world = World::default();
// ... 建场景 ...

let fixed_dt = 1.0 / 60.0;
let sub_steps = 4;

loop {
    // 1) 输入阶段：给刚体施力、设速度、驱动车辆和角色
    //    apply_body_force / set_body_linear_velocity / set_vehicle_driver_input

    // 2) 推进模拟（唯一会改变物理状态的调用）
    world.step(fixed_dt, sub_steps);

    // 3) 立即消费事件（下一步会被清空）
    //    body_events / contact_events / sensor_events / continuous_impacts / joint_event_list

    // 4) 读状态：位置、速度，同步到渲染
    //    body_transform / body_linear_velocity

    // 5) 可选：AI、传感器逻辑、特效触发

    // 6) 可选：调试绘制
    //    draw(&debug_draw, u64::MAX)
}
```

各阶段对应的方法类别：

| 阶段 | 调用 | 说明 |
|------|------|------|
| 输入 | `apply_body_force`、`apply_body_torque`、`apply_body_linear_impulse`、`set_body_linear_velocity` | 力在 `step` 末尾清零；冲量是即时的 |
| 推进 | `step(dt, sub_step_count)` | 固定步长才能保证确定性 |
| 事件 | `body_events`、`contact_events`、`sensor_events`、`continuous_impacts`、`joint_event_list` | 每步重置 |
| 读取 | `body_transform`、`body_position`、`body_linear_velocity`、`body_mass` | 只读，不触发模拟 |
| 查询 | `cast_ray_closest`、`overlap_shape`、`shape_cast_closest` | 只读，可在渲染帧内任意调用 |
| 调试 | `draw(&DebugDraw, mask_bits)` | 通过回调交给你的渲染器 |

角色和车辆是**例外**：它们在 `step` 之外自己更新。

```rust
// 车辆：设好输入后由 step 内部的求解器驱动
world.set_vehicle_driver_input(vehicle, 1.0, 0.0, 0.0, 0.0);

// 角色：自己调用 update，位置由控制器而非刚体求解器决定
character.set_position(character.position());
character.update(fixed_dt);
```

角色的 `update` 会读写世界，因此它需要 `&mut World`。同一时刻不能既持有角色又调用世界方法——这是 Rust 借用规则，不是限制设计。

## World 的 API 分类

`World` 上的公开方法按用途分成十类。完整签名见 [API 速查](/docs/api-reference)。

| 类别 | 代表方法 | 用途 |
|------|----------|------|
| 生命周期 | `new`、`default`、`step` | 创建与推进 |
| 刚体 | `create_body`、`destroy_body`、`body_transform`、`set_body_transform`、`body_linear_velocity` | 实体管理 |
| 形状 | `create_sphere_shape`、`create_hull_shape`、`create_mesh_shape`、`shape_type`、`shape_aabb` | 碰撞几何 |
| 材质与过滤 | `shape_friction`、`set_shape_friction`、`set_shape_filter`、`set_custom_filter` | 表面属性与筛选 |
| 查询 | `cast_ray_closest`、`overlap_shape`、`shape_cast_closest`、`shape_closest_point` | 拾取、视线、地形探测 |
| 事件 | `body_events`、`contact_events`、`sensor_events`、`continuous_impacts` | 游戏逻辑回调 |
| 关节 | `create_revolute_joint`、`create_distance_joint`、`joint_is_valid`、`destroy_joint` | 连接结构 |
| 专用系统 | `create_vehicle`、`create_soft_body`、`CharacterVirtual::new`、`Ragdoll::new` | 车辆/软体/角色/布娃娃 |
| 快照与确定性 | `save_state`、`restore_state`、`world_state_hash` | 回滚与验证 |
| 诊断 | `counters`、`profile_data`、`body_count`、`awake_body_count`、`draw` | 性能与调试 |

一点重要的 API 设计：**所有查询与访问都不返回内部引用**。`body_position` 返回 `Pos` 值，`shape_hull` 返回克隆的 `Hull`。你永远不会因为持有引擎内部数据而阻挡后续修改。

## 世界设置怎么改

世界级设置是 `World` 上的公开字段，直接赋值即可，不需要 `WorldDef` 构造流程：

```rust
use natsel::math::types::Vec3;
use natsel::simulation::World;

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);
world.worker_count = 4;              // 0 或 1 表示单线程
world.enable_sleep = true;
world.enable_continuous = true;
world.max_linear_speed = 400.0;
```

常用可写字段：

| 字段 | 默认 | 作用 |
|------|------|------|
| `gravity` | `(0, -10, 0)` | 重力加速度 |
| `worker_count` | `1` | 工作线程数，不影响结果 |
| `enable_sleep` | `true` | 是否允许刚体睡眠 |
| `enable_warm_starting` | `true` | 是否用上一步冲量做热启动 |
| `enable_continuous` | `true` | 是否启用连续碰撞检测 |
| `enable_speculative` | `true` | 推测接触，减少高速穿透 |
| `max_linear_speed` | `400.0` | 线速度上限 |
| `hit_event_threshold` | `1.0` | 撞击事件的速度阈值 |
| `restitution_threshold` | `1.0` | 弹性生效的速度阈值 |
| `contact_hertz` | `30.0` | 接触软度 |
| `contact_damping_ratio` | `10.0` | 接触阻尼比 |

`WorldDef` 结构体存在，用于描述一整套默认设置（并且是唯一携带回调与容量提示的类型），但**没有 `World::from_def` 这类构造器**。`World::new(Capacity)` 只接受容量提示：

```rust
use natsel::simulation::World;
use natsel::types::world_def::Capacity;

let mut world = World::new(Capacity {
    static_shape_count: 64,
    dynamic_shape_count: 512,
    static_body_count: 8,
    dynamic_body_count: 256,
    contact_count: 2048,
});
```

容量只是预分配提示，给少了不会出错，只是会多几次重新分配。`World::default()` 等价于 `World::new(Capacity::default())`。

回调通过专门的方法注册：

```rust
use std::sync::Arc;

world.set_custom_filter(Some(Arc::new(|shape_a, shape_b| {
    // 返回 false 表示这一对不碰撞
    true
})));
world.set_pre_solve_callback(Some(Arc::new(|a, b, point, normal| {
    // 返回 false 表示本步禁用这个接触
    true
})));
```

## 和源码目录的对应关系

如果你需要查源码，大致对应如下（每个目录一行即可看懂）：

| 目录 | 内容 |
|------|------|
| `src/base/` | 版本、断言、日志、平台相关（计时/线程/原子/预取）、工作线程池 |
| `src/math/` | 向量、四元数、矩阵、变换与数学函数 |
| `src/containers/` | 引擎内部使用的容器与分配器 |
| `src/types/` | 所有公开定义结构体与枚举 |
| `src/collision/` | 凸包构建、网格、高度场、复合、射线与形状投射、流形生成 |
| `src/simulation/` | `World` 本体，刚体/形状/接触/关节/事件/快照/查询 |
| `src/character/` | 角色控制器 |
| `src/vehicle/` | 车辆系统 |
| `src/skeleton/` | 骨骼、布娃娃、动画 |
| `src/soft_body/` | 软体与布料 |
| `src/bin/` | 基准测试程序 |
| `src/tests/` | 测试套件 |

仓库根目录的文档：

- `README.md`：项目简介与快速示例。
- `DEVELOPMENT.md`：开发约定、平台支持、确定性要求。
- `PERFORMANCE.md`：性能测量方法与历史优化记录。
- `MIGRATION.md`：从早期 Swift 实现到 Rust 的迁移记录（只对追溯历史有用）。

## 线程模型对使用者的影响

`World` 自带工作线程池。你只需要设置 `worker_count`：

```rust
world.worker_count = 4;
```

- `0` 或 `1` 表示单线程。
- 大于 `1` 时，并行的是宽相配对生成与窄相碰撞两个阶段；求解器本身按约束图颜色串行推进。
- **改变 `worker_count` 不改变结果**，这是硬性保证。

`WorldDef` 结构体里虽然存在 `enqueue_task` / `finish_task` / `create_debug_shape` 这类回调字段，但 0.2.0 版本没有把它们接入 `World`——`World` 只接受 `Capacity` 容量提示，工作线程池由引擎自己管理。如果需要在你的调度器上运行物理，把 `worker_count` 设为 1 并在自己的线程里调用 `step` 即可，语义完全等价。

调试绘制走的是 `DebugDraw` 结构体上的回调，那是可用的：

```rust
world.draw(&debug_draw, u64::MAX);
```

## 平台与构建

- Rust 1.85+，edition 2024。
- `std`-only，零第三方依赖，MIT 许可。
- 官方 CI 在 Linux、Windows、macOS 三平台构建并运行测试。
- `validate` cargo feature 会在 debug 构建里打开内部一致性校验；它只用于排查问题，正式构建请关闭。

```sh
cargo build --release
cargo test
cargo run --release --bin benchmarks -- --all
```

## 下一步

- [核心概念](/docs/concepts)：对象模型与一次 `step` 的内部顺序。
- [API 速查](/docs/api-reference)：按功能分组的完整签名表。
- [多线程](/docs/multithreading)：`worker_count` 的用法与确定性保证。
- [确定性](/docs/determinism)：什么保证、什么会变。
