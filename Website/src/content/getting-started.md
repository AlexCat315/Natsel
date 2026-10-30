# 快速开始

本指南带你通过 Cargo 集成 Natsel，并运行第一个物理世界。

## 环境要求

- Rust 1.85 及以上（edition 2024）
- 无需第三方依赖，`std`-only

## 添加依赖

在 `Cargo.toml` 中通过 git 依赖引入：

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main" }
```

## 创建一个世界

```rust
use natsel::simulation::world_runtime::World;

let mut world = World::default();
```

## 添加地面

地面是一个静态刚体：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::Pos;
use natsel::types::body_def::BodyDef;
use natsel::types::shape_def::ShapeDef;

let mut ground_def = BodyDef::default();
ground_def.position = Pos::new(0.0, -1.0, 0.0);
let ground = world.create_body(&ground_def);

let shape_def = ShapeDef::default();
let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(20.0, 1.0, 20.0));
```

## 添加一个动态盒子

```rust
let mut body_def = BodyDef::default();
body_def.type_ = BodyType::DynamicBody;
body_def.position = Pos::new(0.0, 5.0, 0.0);
let body = world.create_body(&body_def);

let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
```

`BodyType` 位于 `natsel::types::body_def`。

## 推进模拟

```rust
for _ in 0..60 {
    world.step(1.0 / 60.0, 4);
}
```

`step` 的第二个参数是子步数；求解器内部按子步推进。

## 读取位置

```rust
let position = world.body_position(body);
println!("{position:?}");
```

## 运行测试与基准

```bash
cargo test
cargo run --release --bin benchmarks -- --all -r=1
```

## 下一步

- 阅读 [核心概念](/docs/concepts)，理解 `BodyDef`、形状与世界服务。
- 学习 [确定性](/docs/determinism)，了解多线程下的可重复模拟。
- 查看 [基准性能](/benchmarks)，了解性能数据。