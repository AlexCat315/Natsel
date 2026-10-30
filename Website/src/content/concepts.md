# 核心概念

本页建立使用 Natsel 时的**心智模型**：世界里有什么对象、它们如何组合、每一步按什么顺序发生。具体 API 签名见 [API 速查](/docs/api-reference)，各子系统细节见对应章节。

## 对象模型：World → Body → Shape

物理世界只有三层结构，理解这三层就能理解其余一切：

```
World                 一个独立的模拟实例，持有全部状态
└── Body              刚体：质量、速度、变换、类型
    └── Shape         碰撞几何：球、胶囊、凸包、网格、高度场、复合
```

- **World** 拥有整个世界。创建、销毁、推进、查询都通过它进行。
- **Body** 是动力学单位。它自己不携带形状信息，质量由挂在它上面的形状汇总而来。
- **Shape** 是碰撞几何单位。它属于一个 Body，可以多个形状共用一个 Body。

这个组合关系决定了几个日常操作：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::Pos;
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;

let mut world = World::default();

// 1. 先有刚体
let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;      // 默认是 StaticBody，必须显式设成动态
def.position = Pos::new(0.0, 4.0, 0.0);
let body = world.create_body(&def);

// 2. 再给刚体添加形状；形状可以加多个
let shape_def = ShapeDef::default();
let box_shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
```

没有形状的刚体仍然存在、仍然会按速度和重力积分，但**不会与任何东西碰撞**。这是新手最常见的“物体穿模”原因，详见 [常见问题](/docs/faq)。

形状创建之后**无法单独移除**：要移除某个形状，销毁它所属的整个刚体（`destroy_body`）。因此“可换装”的碰撞体通常拆成多个刚体，或用动态复合实例。

## 四种精度尺度

Natsel 在不同环节使用不同精度，这是它能在离原点很远处保持稳定的原因：

| 环节 | 类型 | 精度 | 说明 |
|------|------|------|------|
| 世界位置 | `Pos` | `f64` | 刚体在世界中的绝对位置 |
| 局部几何 | `Vec3` | `f32` | 形状自身的顶点、半径、偏移 |
| 速度 / 质量属性 | `Vec3` | `f32` | 线速度、角速度、力矩 |
| 求解器内部 | `f32` | `f32` | 约束求解与积分 |

实践含义：**世界坐标用 `Pos`，其余用 `Vec3`**。查询类 API 都会把计算搬到“以查询原点为中心”的局部坐标系里再转回 `f64`，所以远离原点时仍然精确。细节见 [坐标与单位](/docs/units)。

## 刚体类型

`BodyType` 决定刚体是否受力和受哪些力：

| 类型 | 质量 | 速度来源 | 典型用途 |
|------|------|----------|----------|
| `StaticBody` | 0 | 永远不动 | 地面、墙、地形 |
| `KinematicBody` | 0 | 用户设置，求解器推动它 | 移动平台、电梯、门 |
| `DynamicBody` | > 0 | 由力、冲量和碰撞决定 | 道具、角色、碎块 |

```rust
use natsel::types::body_def::BodyType;

let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;  // 字段名带下划线：type_ 而不是 type
```

默认值是 `StaticBody`。忘记设置 `type_` 会让物体完全不动，这是第一个 FAQ 条目。

## 形状类型

六种碰撞几何，各自有对应的创建函数和读取函数：

| 形状 | `ShapeType` | 创建 | 读取 | 适用 |
|------|-------------|------|------|------|
| 球 | `SphereShape` | `create_sphere_shape` | `shape_sphere` | 最简单最快，滚动物体 |
| 胶囊 | `CapsuleShape` | `create_capsule_shape` | `shape_capsule` | 角色、柱子、无棱角道具 |
| 凸包 | `HullShape` | `create_hull_shape` | `shape_hull` | 箱子、石头、任意凸体 |
| 网格 | `MeshShape` | `create_mesh_shape` | `shape_mesh` | 静态场景三角面 |
| 高度场 | `HeightShape` | `create_height_field_shape` | `shape_height_field` | 大范围地形 |
| 复合 | `CompoundShape` | `create_compound_shape` | `shape_compound` | 多个子形状烘焙成一个 |

选择原则：能凸包就凸包，能球/胶囊就球/胶囊。网格只用于静态物体，动态物体上的网格碰撞代价高且容易卡住。

凸包内容会被世界去重：两个内容相同的凸包只存一份，`world.hull_database_count()` 可以看到当前驻留的数量。

## 材质与过滤器

形状上的表面属性决定碰撞响应：

| 字段 | 默认值 | 作用 |
|------|--------|------|
| `density` | `1000 / units³` | 密度，乘以体积得到质量 |
| `base_material.friction` | `0.6` | 库仑摩擦系数 |
| `base_material.restitution` | `0.0` | 弹性系数（反弹） |
| `base_material.rolling_resistance` | `0.0` | 滚动阻力 |
| `base_material.tangent_velocity` | 零向量 | 传送带表面的切向速度 |

两个形状接触时，摩擦取几何平均 `sqrt(a*b)`，弹性取较大值 `max(a, b)`。想让传送带或冰面生效，改的是 `SurfaceMaterial`，不是刚体。

**过滤**决定两个形状是否被允许碰撞，有两层机制：

- `Filter { category_bits, mask_bits, group_index }`——位掩码。默认全 `1`，即与一切碰撞。设置后只有当双方掩码互相匹配时才碰撞。
- `ShapeDef::enable_custom_filtering` + `world.set_custom_filter(...)`——回调式过滤，需要双方或至少一方开启该标志才会调用你的回调。

过滤器也作用于查询：`QueryFilter` 使用同样的 `category_bits` / `mask_bits` 语义，所以被过滤掉的形状也不会被射线命中。

## 接触、岛屿与约束图

这三个词只在需要调优或诊断时才会遇到：

- **接触（Contact）**：一对形状之间的碰撞对。接触携带 1~4 个接触点（流形），接触点数量上限由 `natsel::get_max_manifold_points()` 返回，当前为 4。
- **岛屿（Island）**：通过接触和关节连在一起的一组刚体。岛屿之间互不影响，因此可以独立求解、独立睡眠。
- **约束图（Constraint Graph）**：把约束按颜色分批，同色约束互不共享刚体，从而可以无锁并行求解。颜色数固定。

使用者的直观体验是：**整堆箱子会一起醒来、一起睡着**。这是岛屿行为，不是 bug。想拆分则调整 `sleep_threshold` 或改变堆叠结构。

## 关节

关节把两个刚体绑在一起。所有关节定义都包含一个 `base: JointDef`（两个刚体、两个局部坐标系、力/扭矩阈值、约束刚度），再加上各类型特有的参数。

```rust
use natsel::math::types::Vec3;
use natsel::types::joint_defs::RevoluteJointDef;

let mut hinge = RevoluteJointDef::default();
hinge.base.body_id_a = frame;     // 通常是一侧的静态刚体
hinge.base.body_id_b = door;
hinge.base.local_frame_a.p = Vec3::new(0.0, 1.0, 0.0);
hinge.base.local_frame_b.p = Vec3::new(0.0, 1.0, 0.0);
let joint = world.create_revolute_joint(&hinge);
```

可用的关节类型与创建函数见 [关节](/docs/joints)，签名列表见 [API 速查](/docs/api-reference)。

## 传感器与事件

传感器是不产生物理响应的形状——只报告“谁进来了、谁离开了”。设置 `ShapeDef::is_sensor = true` 并打开需要的事件位即可。

事件全部是**每步重置**的缓冲：只有在 `step` 之后、下一次 `step` 之前读取才有内容。

| 事件 | 读取方法 | 内容 |
|------|----------|------|
| 刚体移动 | `body_events()` | 本步移动过的刚体及其新变换 |
| 接触开始/结束/撞击 | `contact_events()` | 三组缓冲 |
| 传感器进出 | `sensor_events()` | 进入与离开 |
| 连续冲击 | `continuous_impacts()` | CCD 解析出的瞬时冲击 |
| 关节断裂 | `joint_event_list()` | 超过阈值的关节 |

## 世界服务

除了推进模拟，`World` 还提供四类“服务”，它们让 Natsel 适合做需要回滚或强同步的游戏：

- **快照与回滚**：`save_state()` / `restore_state()` 能完整保存和恢复世界，包括接触流形、warm start 冲量、事件缓冲与设置项。见 [快照与状态哈希](/docs/snapshots)。
- **状态哈希**：`world_state_hash()` 把每个活跃刚体的变换（醒着的还有速度）压成一个 `u64`，用来证明两次运行“确实一样”。
- **多线程**：`worker_count` 控制工作线程数，**改变它不会改变结果**。见 [确定性](/docs/determinism) 与 [多线程](/docs/multithreading)。
- **诊断**：`counters()` 给出对象数量与树高，`profile_data()` 给出上一步各阶段耗时。

## 特殊系统

四个建立在刚体之上的专用系统，各自有独立章节：

| 系统 | 入口 | 说明 |
|------|------|------|
| 车辆 | `create_vehicle` | 悬挂、轮胎、传动、差速器 |
| 虚拟角色 | `CharacterVirtual::new` | 胶囊角色控制器，爬坡、台阶、贴地 |
| 骨骼 | `Ragdoll::new` | 骨骼映射、布娃娃、动画驱动 |
| 软体 | `create_soft_body` | XPBD 可变形体，布料与体积体 |

## 一次 `step` 里发生什么

使用者只需要知道顺序，不需要知道内部实现：

```
world.step(dt, sub_step_count)
  1. 更新宽相碰撞对
  2. 窄相：生成/更新接触流形
  3. 准备约束（接触、关节、车辆）
  4. 按子步求解：积分速度 → warm start → 求解冲量 → 积分位置 → 松弛
  5. 恢复系数与冲击事件
  6. 积分结果回写刚体、刷新宽相代理、清除力的累积
  7. 睡眠处理、传感器重叠、事件发布
```

两个直接后果：

- **力会在 `step` 结束后清零**。要持续施力，每帧在 `step` 之前调用 `apply_body_force`；一次性推动用冲量 `apply_body_linear_impulse`。
- **事件缓冲在 `step` 开始时清空**，所以必须在 `step` 之后立刻消费。

## 下一步

- [架构总览](/docs/architecture)：模块提供哪些能力，游戏循环里按什么顺序调用。
- [坐标与单位](/docs/units)：`Pos` 与 `Vec3` 的分工，单位如何影响默认值。
- [API 速查](/docs/api-reference)：按功能分组的方法表，可直接抄用。
- [常见问题](/docs/faq)：症状、原因、修法。
