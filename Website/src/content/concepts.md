# 核心概念

本页介绍 Natsel 的核心类型与设计。

## Vec3 与 Transform

公开 API 沿用 b3 风格：`Vec3`、`Pos`、`Quat`、`Transform`、`BodyID`、`World`，以及 `make_box_hull`、`compute_cos_sin` 等自由函数。

世界坐标使用 `Pos`（f64），局部几何与速度使用 `Vec3`（f32）。

```rust
use natsel::math::types::{Quat, Transform, Vec3};

let position = Vec3::new(0.0, 5.0, 0.0);
let transform = Transform::new(position, Quat::IDENTITY);
```

## BodyDef

`BodyDef` 描述一个刚体的静态配置：类型、位置、旋转、速度、阻尼、睡眠阈值等。

```rust
use natsel::math::types::{Pos, Vec3};
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::Sphere;
use natsel::types::shape_def::ShapeDef;

let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;
def.position = Pos::new(0.0, 5.0, 0.0);
let id = world.create_body(&def);

let mut shape_def = ShapeDef::default();
shape_def.base_material.friction = 0.6;
shape_def.base_material.restitution = 0.2;
let _ = world.create_sphere_shape(id, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
```

## 形状

Natsel 支持多种碰撞形状：

| 形状 | 说明 |
|------|------|
| 凸包 Hull | 由 `make_box_hull` 等构建器生成，内容键共享 |
| 网格 Mesh | 三角形网格，带 BVH |
| 高度场 HeightField | 地形 |
| 复合 Compound | 多个子形状组合 |
| 球 / 胶囊 / 盒 | 常用基本体 |

相同内容的凸包会被世界"内化"为同一个实例，避免重复存储。

## BodyID

`create_body` 返回一个 `BodyID`，后续查询与操作都通过它进行，避免直接持有内部数据。

```rust
let position = world.body_position(id);
let velocity = world.body_linear_velocity(id);
```

## 关节

关节通过 `JointDef` 描述，包括旋转、6DOF、轮子、路径等类型。

```rust
use natsel::types::joint_defs::RevoluteJointDef;

let mut hinge = RevoluteJointDef::default();
hinge.base.body_id_a = frame;
hinge.base.body_id_b = door;
hinge.base.local_frame_a.p = Vec3::new(0.0, 1.0, 0.0);
hinge.base.local_frame_b.p = Vec3::new(0.0, 1.0, 0.0);
let joint = world.create_revolute_joint(&hinge);
```

## 传感器与事件

传感器不产生物理响应，只报告重叠。接触、命中与连续冲击通过事件 API 暴露：

```rust
for hit in world.continuous_impacts() { /* 本步 CCD 冲击，每步重置 */ }
```

## 世界服务

- **录制 / 回放**：记录世界状态，用于复现
- **快照**：保存与恢复世界
- **调度器**：多线程按岛屿并行求解

## 特殊系统

- **车辆**：传动、差速、轮胎与碰撞
- **虚拟角色**：角色控制器
- **骨骼**：骨骼映射、布娃娃、骨骼动画
- **软体**：可变形体