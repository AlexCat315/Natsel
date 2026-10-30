# 快速开始

本页从零搭建一个可以运行的 Natsel 场景：创建世界、添加地面与动态刚体、推进模拟并读回结果。完成后你会得到一个方块落到地面上并静止的最小工程。

## 环境要求

- Rust 1.85 及以上（crate 使用 edition 2024）
- 无需第三方依赖，引擎是 `std`-only

如果本机工具链较旧，先升级：

```sh
rustup update stable
rustc --version
```

## 添加依赖

在 `Cargo.toml` 中通过 git 依赖引入：

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main" }
```

本地开发时也可以直接指向克隆下来的目录：

```toml
[dependencies]
natsel = { path = "../Natsel" }
```

依赖树的完整说明（`cargo add`、`validate` 特性、平台支持）见 [安装与集成](/docs/installation)。

## 核心模型

写代码之前先理解三件事，后面的 API 就都是它们的自然结果：

| 概念 | 作用 | 关键类型 |
|------|------|----------|
| 世界 `World` | 持有所有刚体、形状、接触与约束，负责推进时间 | `natsel::simulation::World` |
| 刚体 `Body` | 一个可变换的坐标框架，本身没有碰撞形状 | `BodyDef`、`BodyID` |
| 形状 `Shape` | 挂在刚体上的碰撞几何与材质 | `ShapeDef`、`Sphere`、`Capsule`、`Hull` 等 |

刚体与形状是分开创建的：`create_body` 只负责类型、位置、速度等运动学状态；形状通过 `create_hull_shape` 之类的调用挂上去，并决定质量、摩擦与事件。因此"一个刚体多个形状"是常态，例如一个人形刚体上挂胶囊和凸包。

形状创建之后不能修改几何，只能销毁重建；但材质、过滤、事件开关可以在运行时改。

另一个要点是**世界坐标用 `Pos`（f64），局部几何与速度用 `Vec3`（f32）**。`BodyDef.position` 是 `Pos`，而 `Sphere::new` 的圆心是 `Vec3`。混用会编译报错，这是有意的。

## 完整示例

下面这段代码可以直接作为 `src/main.rs` 运行，涵盖本页所有内容：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;

fn main() {
    // 1. 空世界。World::default() 等价于 World::new(Capacity::default())
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -9.81, 0.0);

    // 2. 地面：BodyDef::default() 的类型是 StaticBody，所以不用改 type_
    let mut ground_def = BodyDef::default();
    ground_def.position = Pos::new(0.0, -0.5, 0.0);
    ground_def.name = Some("ground".to_string());
    let ground = world.create_body(&ground_def);

    let ground_shape = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &ground_shape, &make_box_hull(10.0, 0.5, 10.0));

    // 3. 动态方块。必须显式把 type_ 改成 DynamicBody
    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, 4.0, 0.0);
    body_def.name = Some("crate".to_string());
    let body = world.create_body(&body_def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 500.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    // 4. 推进 3 秒（60Hz，每帧 4 个子步）
    for _ in 0..180 {
        world.step(1.0f32 / 60.0, 4);
    }

    // 5. 读回状态
    let p = world.body_position(body);
    println!(
        "y={:.4} mass={:.1} shapes={}",
        p.y,
        world.body_mass(body),
        world.body_shape_count(body)
    );
}
```

运行后输出类似：

```text
y=0.4999 mass=500.0 shapes=1
```

`y` 是方块中心高度：方块半高 0.5，地面顶面在 `-0.5 + 0.5 = 0`，所以静止时中心停在 0.5 附近。剩下的一点偏差来自接触求解允许的微小穿透，属正常数值行为。

## 逐步拆解

### 创建世界

```rust
let mut world = World::default();
```

`World::default()` 就是 `World::new(Capacity::default())`，零容量起步，内部数组会按需增长。已知规模时可以用 `Capacity` 预分配：

```rust
use natsel::types::world_def::Capacity;
let mut world = World::new(Capacity {
    static_body_count: 1,
    dynamic_body_count: 500,
    ..Default::default()
});
```

### 设置重力与其他参数

世界参数直接是 `World` 的公开字段，构造后写字段即可，**没有** `WorldDef` 构造入口：

```rust
world.gravity = Vec3::new(0.0, -20.0, 0.0);
world.worker_count = 4;
world.enable_sleep = false;
```

不要写 `World::new(def)`。世界参数一律通过上面这种方式直接赋值。可调的公开字段与对应辅助方法见 [多线程](/docs/multithreading)。

### 推进模拟

```rust
world.step(time_step, sub_step_count);
```

两个参数都必须显式传入，Rust 没有默认参数。

- `time_step` 是这一帧推进的秒数，常用 `1.0 / 60.0`。
- `sub_step_count` 是求解器内部把这一帧切成多少个子步。**子步越多，接触越稳，开销也越大**。默认习惯值是 4；对堆叠、快速旋转的物体可以提到 8，而 1 适合只需要大致结果的场景。

`step` 会清空上一帧的事件缓冲、更新宽相、碰撞、求解、积分并写回变换。因此事件必须在 `step` 之后立刻读取，下一次 `step` 就覆盖了。

### 读取状态

刚体查询都通过 `BodyID` 进行：

```rust
let position = world.body_position(body);        // Pos
let rotation = world.body_rotation(body);        // Quat
let velocity = world.body_linear_velocity(body); // Vec3
let all = world.body_transform(body);            // WorldTransform { p: Pos, q: Quat }
```

### 清理

```rust
world.destroy_body(body);
```

销毁刚体会一并销毁它上面的所有形状、接触和关节。`BodyID` 之后失效，`world.body_is_valid(id)` 会返回 `false`。ID 槽位会被回收复用，但 generation 会递增，所以旧的 `BodyID` 不会意外指向新刚体。

## 常见错误

**忘记设置 `type_`。** `BodyDef::default()` 的 `type_` 是 `BodyType::StaticBody`，质量为零、不受力。直接 `create_body(&BodyDef::default())` 得到的是一个不动的物体。这是最常见的"我的物体为什么不下落"。

**用 `Quat::default()` 当旋转。** `Quat` 派生的 `Default` 是全零四元数，不是单位旋转。必须用 `Quat::IDENTITY`，否则 `create_body` 会拒绝这个刚体定义。

**把 `Pos` 和 `Vec3` 混用。** `BodyDef.position` 要 `Pos::new(0.0, 5.0, 0.0)`，`Sphere::new` 的第一个参数要 `Vec3::new(...)`。编译器的类型错误就是在提醒你这层区分。

**在创建形状前就读取质量。** 刚体没有形状时质量为 0（对应 `inv_mass = 0`）。质量是形状密度积分出来的，挂上形状时才计算。

**每帧重新施加恒力却忘了力的生命周期。** `apply_body_force` 累加的力在一个 `step` 结束时清零，所以持续推力必须在每帧 `step` 之前重新施加。详见 [刚体](/docs/bodies)。

## 下一步

- [核心概念](/docs/concepts)：`Vec3`、`Transform`、`BodyID` 等公共类型的设计。
- [安装与集成](/docs/installation)：`validate` 特性、构建与测试命令、平台支持。
- [刚体](/docs/bodies)：三种刚体类型、运动锁、阻尼、睡眠、力与冲量。
- [形状](/docs/shapes)：六种形状与 `ShapeDef` 全字段。
- [几何构建器](/docs/geometry)：用代码生成凸包、网格、高度场与复合体。
- [材质与过滤](/docs/materials)：摩擦、弹性、逐三角形材质与碰撞过滤。
- [确定性](/docs/determinism)：为什么改变线程数不影响结果。
