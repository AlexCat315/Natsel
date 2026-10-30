# 关节

关节把两个刚体连接起来，约束它们之间的相对运动。门绕着铰链转、活塞沿导轨滑动、车轮带悬挂、绳索保持距离——这些都是关节。

## 关节模型

每个关节由三部分定义：

1. **两个刚体**：`body_id_a` 与 `body_id_b`
2. **两个局部坐标系**：`local_frame_a` / `local_frame_b`，分别在各自刚体的局部空间中描述连接点与朝向
3. **类型特有的参数**：角度限制、马达、弹簧、传动比等

关节约束的是"两个局部坐标系的相对位姿"，而不是世界坐标。因此 `local_frame_a.p = Vec3::new(0.0, 1.0, 0.0)` 表示"在 A 自身坐标系的 (0,1,0) 处"，当 A 移动时锚点跟着移动。

所有关节定义都含有一个 `base: JointDef` 字段，公共参数都在里面：

```rust
pub struct JointDef {
    pub user_data: Option<*mut core::ffi::c_void>,
    pub body_id_a: BodyID,
    pub body_id_b: BodyID,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub force_threshold: f32,
    pub torque_threshold: f32,
    pub constraint_hertz: f32,
    pub constraint_damping_ratio: f32,
    pub draw_scale: f32,
    pub collide_connected: bool,
}
```

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `user_data` | `None` | 用户指针，随关节事件返回 |
| `body_id_a` | `BodyID::NULL` | 刚体 A |
| `body_id_b` | `BodyID::NULL` | 刚体 B |
| `local_frame_a` | `Transform::IDENTITY` | A 上的锚点与朝向 |
| `local_frame_b` | `Transform::IDENTITY` | B 上的锚点与朝向 |
| `force_threshold` | `f32::MAX` | 超过则产生关节事件（默认从不触发） |
| `torque_threshold` | `f32::MAX` | 同上，力矩版 |
| `constraint_hertz` | `60.0` | 约束柔度频率 |
| `constraint_damping_ratio` | `2.0` | 约束阻尼比 |
| `draw_scale` | 长度单位/米 | 调试绘制缩放 |
| `collide_connected` | `false` | 相连的两刚体是否相互碰撞 |

`collide_connected` 默认是 `false`：门和门框不会互相碰撞。若连接的两个刚体需要碰撞（例如链条环节），显式设为 `true`。

## 关节类型

```rust
pub enum JointType {
    ParallelJoint,
    DistanceJoint,
    FilterJoint,
    MotorJoint,
    PrismaticJoint,
    RevoluteJoint,
    SphericalJoint,
    WeldJoint,
    WheelJoint,
    GearJoint,
    RackAndPinionJoint,
    PulleyJoint,
    SixDOFJoint,
    PathJoint,
}
```

| 类型 | 自由度 | 典型用途 |
|------|--------|---------|
| `RevoluteJoint` | 1（绕轴旋转） | 铰链、门、摆臂 |
| `PrismaticJoint` | 1（沿轴平移） | 活塞、滑轨、升降台 |
| `DistanceJoint` | 保持距离 | 绳索、链条、弹簧 |
| `SphericalJoint` | 3（球窝） | 肩关节、拖挂 |
| `WeldJoint` | 0（完全固定） | 焊接、刚性连接 |
| `WheelJoint` | 悬挂 + 自转 + 转向 | 车轮 |
| `MotorJoint` | 6（软约束） | 驱动器、相对位姿控制 |
| `ParallelJoint` | 保持朝向平行 | 保持两物体同向 |
| `FilterJoint` | 无（仅过滤碰撞） | 禁止两形状碰撞 |
| `GearJoint` | 传动比耦合 | 齿轮组 |
| `RackAndPinionJoint` | 转动↔平移耦合 | 齿条齿轮 |
| `PulleyJoint` | 滑轮比耦合 | 滑轮、配重 |
| `SixDOFJoint` | 每轴独立 | 通用关节、布娃娃 |
| `PathJoint` | 沿路径 | 过山车、导轨 |

## 创建关节

创建方法都在 `World` 上，返回 `JointID`：

```rust
world.create_revolute_joint(&RevoluteJointDef) -> JointID
world.create_prismatic_joint(&PrismaticJointDef) -> JointID
world.create_spherical_joint(&SphericalJointDef) -> JointID
world.create_weld_joint(&WeldJointDef) -> JointID
world.create_wheel_joint(&WheelJointDef) -> JointID
world.create_distance_joint(&DistanceJointDef) -> JointID
world.create_motor_joint(&MotorJointDef) -> JointID
world.create_parallel_joint(&ParallelJointDef) -> JointID
world.create_filter_joint(&FilterJointDef) -> JointID
world.create_path_joint(&PathJointDef) -> JointID
world.create_gear_joint(&GearJointDef) -> JointID
world.create_rack_and_pinion_joint(&RackAndPinionJointDef) -> JointID
world.create_pulley_joint(&PulleyJointDef) -> JointID
world.create_six_dof_joint(SixDOFJointDef) -> JointID   // 按值传递，不加 &
```

注意 `create_six_dof_joint` 是唯一**按值**接收定义的（因为它内含 `Vec`）。

### 旋转关节

```rust
pub struct RevoluteJointDef {
    pub base: JointDef,
    pub target_angle: f32,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub lower_angle: f32,
    pub upper_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
}
```

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `target_angle` | `0.0` | 弹簧目标角度（弧度） |
| `enable_spring` | `false` | 启用角度弹簧 |
| `hertz` | `0.0` | 弹簧频率 |
| `damping_ratio` | `0.0` | 弹簧阻尼 |
| `enable_limit` | `false` | 启用角度限制 |
| `lower_angle` | `0.0` | 下限（弧度） |
| `upper_angle` | `0.0` | 上限（弧度） |
| `enable_motor` | `false` | 启用马达 |
| `max_motor_torque` | `0.0` | 马达最大力矩 |
| `motor_speed` | `0.0` | 马达目标角速度 |

### 距离关节

```rust
pub struct DistanceJointDef {
    pub base: JointDef,
    pub length: f32,
    pub enable_spring: bool,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub min_length: f32,
    pub max_length: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}
```

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `length` | `1.0` | 目标距离 |
| `enable_spring` | `false` | 启用弹簧 |
| `lower_spring_force` | `-f32::MAX` | 弹簧最小力 |
| `upper_spring_force` | `f32::MAX` | 弹簧最大力 |
| `hertz` | `0.0` | 弹簧频率 |
| `damping_ratio` | `0.0` | 弹簧阻尼 |
| `enable_limit` | `false` | 启用距离范围限制 |
| `min_length` | `0.0` | 最小距离 |
| `max_length` | 极大值 | 最大距离 |
| `enable_motor` | `false` | 沿距离轴的马达 |
| `max_motor_force` | `0.0` | 马达最大力 |
| `motor_speed` | `0.0` | 马达目标速度 |

### 轮式关节

```rust
pub struct WheelJointDef {
    pub base: JointDef,
    pub enable_suspension_spring: bool,
    pub suspension_hertz: f32,
    pub suspension_damping_ratio: f32,
    pub enable_suspension_limit: bool,
    pub lower_suspension_limit: f32,
    pub upper_suspension_limit: f32,
    pub enable_spin_motor: bool,
    pub max_spin_torque: f32,
    pub spin_speed: f32,
    pub enable_steering: bool,
    pub steering_hertz: f32,
    pub steering_damping_ratio: f32,
    pub target_steering_angle: f32,
    pub max_steering_torque: f32,
    pub enable_steering_limit: bool,
    pub lower_steering_limit: f32,
    pub upper_steering_limit: f32,
}
```

默认：`enable_suspension_spring = true`、`suspension_hertz = 1.0`、`suspension_damping_ratio = 0.7`、`steering_hertz = 1.0`、`steering_damping_ratio = 0.7`，其余布尔为 `false`。

> 完整的车辆系统（传动、差速、轮胎）请用[车辆](/docs/vehicle)模块，`WheelJoint` 适合简单的手搭轮子。

### 传动关节

`GearJointDef`、`RackAndPinionJointDef` 通过**引用其他关节**建立耦合：

```rust
pub struct GearJointDef {
    pub base: JointDef,
    pub ratio: f32,               // 默认 1.0
    pub reference_joint_a: JointID, // 默认 JointID::NULL
    pub reference_joint_b: JointID,
}

pub struct RackAndPinionJointDef {
    pub base: JointDef,
    pub ratio: f32,
    pub reference_pinion_joint: JointID,
    pub reference_rack_joint: JointID,
}
```

被引用的关节必须是旋转或滑动类型，且必须在创建传动关节**之前**创建。

### 滑轮关节

```rust
pub struct PulleyJointDef {
    pub base: JointDef,
    pub ground_anchor_a: Pos,
    pub ground_anchor_b: Pos,
    pub ratio: f32,       // 默认 1.0
    pub min_length: f32,  // 默认 0.0
    pub max_length: f32,  // 默认 0.0
}
```

`ground_anchor_a` / `ground_anchor_b` 是**世界坐标**，不是局部坐标。

## 检查关节

```rust
pub fn joint_type(&self, joint_id: JointID) -> Option<JointType>
pub fn joint_bodies(&self, joint_id: JointID) -> Option<(BodyID, BodyID)>
pub fn joint_is_valid(&self, joint_id: JointID) -> bool
pub fn joint_is_awake(&self, joint_id: JointID) -> bool
```

返回 `Option` 的方法在 ID 无效或类型不匹配时给出 `None`，因此可以安全地在持有旧 ID 时查询。

| 方法 | 返回 | 说明 |
|------|------|------|
| `joint_collide_connected(id)` | `bool` | 相连刚体是否相互碰撞 |
| `joint_constraint_tuning(id)` | `Option<(f32, f32)>` | `(hertz, damping_ratio)` |
| `joint_force_threshold(id)` | `Option<f32>` | 力阈值 |
| `joint_torque_threshold(id)` | `Option<f32>` | 力矩阈值 |
| `joint_local_frame_a(id)` | `Option<Transform>` | A 上的局部坐标系 |
| `joint_local_frame_b(id)` | `Option<Transform>` | B 上的局部坐标系 |
| `joint_world_anchor_a(id)` | `Option<Pos>` | A 锚点的世界坐标 |
| `joint_linear_separation(id)` | `Option<f32>` | 线性约束违反量 |
| `joint_angular_separation(id)` | `Option<f32>` | 角度约束违反量 |
| `joint_constraint_force(id)` | `Option<Vec3>` | 当前约束力 |
| `joint_constraint_torque(id)` | `Option<Vec3>` | 当前约束力矩 |

对应的写入方法：

```rust
set_joint_collide_connected(id, bool)
set_joint_constraint_tuning(id, hertz, damping_ratio) -> bool
set_joint_force_threshold(id, f32) -> bool
set_joint_torque_threshold(id, f32) -> bool
set_joint_local_frame_a(id, Transform) -> bool
set_joint_local_frame_b(id, Transform) -> bool
```

`joint_linear_separation` 与 `joint_angular_separation` 适合做"约束是否被拉爆"的诊断——数值变大说明求解器跟不上。

## 销毁关节

```rust
pub fn destroy_joint(&mut self, joint_id: i32, wake_attached: bool)
```

> **重要陷阱：这里要传 `i32` 索引，不是 `JointID`。** 所有创建方法返回 `JointID`，但销毁要的是索引。转换方式是取 `index1 - 1`：
>
> ```rust
> let joint = world.create_distance_joint(&rope);
> world.destroy_joint(joint.index1 - 1, true);
> ```
>
> `wake_attached` 控制是否唤醒原本连接的两个刚体——销毁约束后它们通常应该被唤醒，否则会看起来"悬在空中"。

销毁后 `joint_is_valid(joint_id)` 返回 `false`，`joint_count()` 相应减少。

## 完整示例

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Transform, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::joint_defs::{DistanceJointDef, JointType, RevoluteJointDef};
use natsel::types::shape_def::ShapeDef;

fn main() {
    let mut world = World::default();
    world.enable_sleep = false;

    // 静态支架
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let frame = world.create_body(&def);

    // 一根水平臂，右端悬在支架右侧 1 米处
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 5.0, 0.0);
    let arm = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(arm, &shape_def, &make_box_hull(0.5, 0.1, 0.1));

    // 铰链：两锚点都设在臂的左端，绕 z 轴转动
    let mut hinge = RevoluteJointDef::default();
    hinge.base.body_id_a = frame;
    hinge.base.body_id_b = arm;
    hinge.base.local_frame_a = Transform::IDENTITY;      // 支架原点
    hinge.base.local_frame_b.p = Vec3::new(-1.0, 0.0, 0.0); // 臂的左端
    hinge.enable_limit = true;
    hinge.lower_angle = -0.5;
    hinge.upper_angle = 0.5;
    hinge.enable_motor = true;
    hinge.max_motor_torque = 50.0;
    hinge.motor_speed = 1.0;

    let hinge_id = world.create_revolute_joint(&hinge);

    println!("类型 {:?}", world.joint_type(hinge_id));
    println!("有效 {}", world.joint_is_valid(hinge_id));
    println!("连接体 {:?}", world.joint_bodies(hinge_id));
    println!("调参 {:?}", world.joint_constraint_tuning(hinge_id));
    println!("世界锚点 {:?}", world.joint_world_anchor_a(hinge_id));

    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }

    // 马达应把臂推到角度上限附近
    println!("角度偏差 {:?}", world.joint_angular_separation(hinge_id));
    println!("约束力 {:?}", world.joint_constraint_force(hinge_id));

    // 关节事件：阈值降到 0 才会报告
    world.set_joint_force_threshold(hinge_id, 0.0);
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
        let events = world.joint_event_list();
        if !events.is_empty() {
            println!("关节事件 {} 个，首个 user_data={:?}",
                events.len(), events[0].user_data);
            break;
        }
    }

    // 距离关节当作吊绳，带长度限制与弹簧
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 3.0, 0.0);
    let bob = world.create_body(&def);
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut rope = DistanceJointDef::default();
    rope.base.body_id_a = frame;
    rope.base.body_id_b = bob;
    rope.length = 2.0;
    rope.enable_spring = true;
    rope.hertz = 2.0;
    rope.damping_ratio = 0.5;
    rope.enable_limit = true;
    rope.min_length = 1.0;
    rope.max_length = 3.0;

    let rope_id = world.create_distance_joint(&rope);
    assert_eq!(world.joint_type(rope_id), Some(JointType::DistanceJoint));

    // 销毁：传索引，不是 JointID
    world.destroy_joint(rope_id.index1 - 1, true);
    println!("销毁后有效 = {}", world.joint_is_valid(rope_id));
    println!("剩余关节 {}", world.joint_count());
}
```

## 常见问题

**`destroy_joint` 接收 `i32` 索引。** 传 `JointID` 会编译失败；传错值会导致访问越界或销毁错误的关节。始终用 `joint_id.index1 - 1`。

**`local_frame` 是局部坐标。** 锚点相对于刚体自身的原点与朝向，不是世界坐标。两个刚体的锚点在世界中重合，关节才"接得上"。若创建后物体被弹开，通常是锚点没对齐。

**`collide_connected` 默认 `false`。** 相连的两个刚体默认不互相碰撞。手搭链条、齿轮时需要显式设 `true`，否则环节会互相穿过。

**关节阈值默认 `f32::MAX`。** 这意味着默认不产生任何关节事件。要断裂效果，先用 `set_joint_force_threshold` 设一个具体值。

**传动关节必须在被引用关节之后创建。** `GearJointDef` 与 `RackAndPinionJointDef` 引用既有的 `JointID`；先建齿轮关节再建齿轮本体是不行的。

**`create_six_dof_joint` 按值传参。** 其余创建方法接收 `&Def`，只有它接收 `SixDOFJointDef`（因为它内含 `Vec<SixDOFAxisSettings>`）。写成 `&def` 会编译失败。

**关节过软或抖动时先调 `constraint_hertz`。** 默认 `60.0` 对大多数场景合适；大量关节串联（长链条）需要更低的频率或更多子步。`joint_linear_separation` 持续增大说明求解跟不上。

**`wake_attached` 通常应为 `true`。** 销毁约束后若不唤醒，刚体会保持睡眠并停在原位，看起来像卡住。

## 相关页面

- [刚体](/docs/bodies) —— 关节连接的刚体
- [车辆](/docs/vehicle) —— 完整的车辆悬挂与传动
- [骨骼](/docs/skeleton) —— 布娃娃与骨骼关节
- [事件与传感器](/docs/events) —— 关节事件与阈值
- [确定性](/docs/determinism) —— 关节求解顺序与可复现性
