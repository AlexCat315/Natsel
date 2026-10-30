# 事件与传感器

物理世界推进后，游戏逻辑需要知道"发生了什么"：谁撞了谁、什么落到了触发区里、哪些物体睡着了。Natsel 把这些以事件缓冲的形式暴露出来，由你在 `step` 之后读取。

## 事件模型

Natsel 不提供回调式中枢，而是**每步结束时把事件攒进缓冲区**，你在 `step` 返回后一次性取走。这样做的原因：

- 事件顺序固定（按接触 ID 排序），因此事件序列本身也是确定性的
- 你可以在同一帧里用任意顺序处理事件，不必担心在物理步内部修改世界
- 一次 `step` 可能包含多个子步，事件已经合并成"这一步的结果"

关键时序规则：

1. `world.step()` 开始时，`body_move_events`、`sensor_begin_events`、`contact_begin_events`、`continuous_impact_events` 被清空
2. 步进过程中填充事件
3. 步进结束前，`sensor_end_events` 与 `contact_end_events` 被发布为可读状态
4. `step` 返回，你读取事件

因此**必须在下一次 `step` 之前读完事件**，否则会被清掉。

## 事件开关

事件不是白来的，每个形状需要在 `ShapeDef` 里显式请求：

| `ShapeDef` 字段 | 默认 | 开启后产生 |
|-----------------|------|-----------|
| `enable_sensor_events` | `false` | 传感器 begin / end 事件 |
| `enable_contact_events` | `false` | 接触 begin / end 事件，以及 CCD 冲击 |
| `enable_hit_events` | `false` | 高速接触的命中事件 |
| `enable_pre_solve_events` | `false` | 求解前回调 |
| `is_sensor` | `false` | 该形状成为传感器（只报告重叠，不产生物理响应） |

形状创建后也可以用 `enable_shape_sensor_events(shape_id, bool)`、`enable_shape_contact_events(shape_id, bool)`、`enable_shape_hit_events(shape_id, bool)`、`enable_shape_pre_solve_events(shape_id, bool)` 动态开关。

> **关键陷阱：传感器事件需要双方都开启 `enable_sensor_events`。** 传感器形状开启只是必要条件；进入传感器的那个形状（visitor）也必须开启，否则事件永远不触发。而且 visitor 必须是**凸形状**（球、胶囊、凸包）——网格、高度场、复合形状不会作为 visitor 被报告。

## 物体移动事件

```rust
pub fn body_events(&self) -> &[BodyMoveEvent]
```

```rust
pub struct BodyMoveEvent {
    pub user_data: Option<*mut core::ffi::c_void>,
    pub transform: WorldTransform,
    pub body_id: BodyID,
    pub fell_asleep: bool,
}
```

| 字段 | 说明 |
|------|------|
| `user_data` | 创建刚体时传入的 `user_data` 指针 |
| `transform` | 本步结束时的位姿（已含 CCD 回退） |
| `body_id` | 对应的刚体 |
| `fell_asleep` | 这一步是否刚进入睡眠 |

事件只包含**本步在清醒集合中且位姿发生变化**的刚体。`transform` 与 `body_transform(body_id)` 在读取时刻是一致的，CCD 把物体回退到撞击点时两者同样一致，所以可以直接用它做渲染插值而不必再查一次。

返回的是借用切片，不复制数据。

## 传感器事件

```rust
pub fn sensor_events(&self) -> SensorEvents
```

```rust
pub struct SensorEvents {
    pub begin: Vec<SensorBeginTouchEvent>,
    pub end: Vec<SensorEndTouchEvent>,
}

pub struct SensorBeginTouchEvent {
    pub sensor_shape_id: ShapeID,
    pub visitor_shape_id: ShapeID,
}
```

`SensorEndTouchEvent` 字段相同。另有 `sensor_event_buffers()` 返回字段名为 `begin_events` / `end_events` 的结构，内容一致。

这两种返回都**复制**了事件列表（不是借用切片），因此可以自由持有。

传感器的语义：

- 物体进入传感器范围时产生一次 begin
- 持续停留期间不重复产生 begin
- 离开时产生一次 end
- 传感器从创建起就重叠的物体，第一步产生 begin，停留期间不产生 end

## 接触事件

```rust
pub fn contact_events(&self) -> ContactEvents
```

```rust
pub struct ContactEvents {
    pub begin_events: Vec<ContactBeginTouchEvent>,
    pub end_events: Vec<ContactEndTouchEvent>,
    pub hit_events: Vec<ContactHitEvent>,
}

pub struct ContactBeginTouchEvent {
    pub shape_id_a: ShapeID,
    pub shape_id_b: ShapeID,
    pub contact_id: ContactID,
}
```

`ContactEndTouchEvent` 字段相同。只有开启了 `enable_contact_events` 的形状参与的接触才会被报告；形状顺序不保证（`shape_id_a` / `shape_id_b` 不表示谁先谁后）。

### 命中事件

```rust
pub struct ContactHitEvent {
    pub shape_id_a: ShapeID,
    pub shape_id_b: ShapeID,
    pub contact_id: ContactID,
    pub point: Pos,
    pub normal: Vec3,
    pub approach_speed: f32,
    pub user_material_id_a: u64,
    pub user_material_id_b: u64,
}
```

| 字段 | 说明 |
|------|------|
| `point` | 世界坐标撞击点 |
| `normal` | 接触法线 |
| `approach_speed` | 法向接近速度（正值） |
| `user_material_id_a/b` | 两侧的材质 ID |

命中事件只在本步最大接近速度**超过 `world.hit_event_threshold`** 时产生。默认阈值是 `1.0`，想捕获更轻微的撞击就调小它：

```rust
world.hit_event_threshold = 0.5;
```

这是判断"要不要播放撞击音效/粒子"的常用手段——用阈值过滤掉细碎的滑动接触。

> **陷阱：`hit_events` 不会在下一步被清空。** 与其余事件不同，命中事件缓冲区在 `step` 开头不会被清空，因此它会持续保留上一次的命中。如果每帧都遍历它，同一个撞击会被重复处理。正确做法是记录已处理的 `contact_id`，或者只在 `begin_events` 出现时读取命中事件。

## 连续冲击事件

```rust
pub fn continuous_impacts(&self) -> &[ContactHitEvent]
```

暴露本步由 CCD 解决的瞬时冲击，**包括那些在持久接触建立之前就已经解决的冲击**。普通接触事件看不到这种情况：子弹穿过薄墙时物体被回退并反弹，但从未形成稳定的接触流形。

`continuous_impacts` 由 `enable_contact_events` 或 `enable_hit_events` 任一开启触发，**不受 `hit_event_threshold` 过滤**，因此能看到所有 CCD 撞击。每步在 `step` 开头重置。

## 关节事件

```rust
pub fn joint_event_list(&self) -> Vec<JointEvent>
```

```rust
pub struct JointEvent {
    pub joint_id: JointID,
    pub user_data: Option<*mut core::ffi::c_void>,
}
```

当关节的反作用力或力矩超过其阈值时产生事件。默认阈值是 `f32::MAX`，即**默认不产生任何事件**；必须显式降低：

```rust
world.set_joint_force_threshold(joint_id, 0.0);  // 0 表示只要有载荷就报告
world.set_joint_torque_threshold(joint_id, 0.0);
```

用于做"绳索断裂""铰链过载"之类的效果。事件每步重新生成，但会跳过睡眠中的关节。

## 世界统计

这些不产生事件，用于调试与预算：

| 方法 | 返回 | 含义 |
|------|------|------|
| `body_count()` | `i32` | 存活刚体数 |
| `contact_count()` | `i32` | 存活接触数 |
| `joint_count()` | `i32` | 存活关节数 |
| `touching_contact_count()` | `i32` | 实际接触中的接触数 |
| `awake_body_count()` | `i32` | 清醒刚体数 |
| `hull_database_count()` | `usize` | 已内化的凸包数 |
| `compound_shape_count_value()` | `i32` | 存活复合形状数 |
| `body_is_valid(body_id)` | `bool` | 该 ID 是否仍指向存活刚体 |

`body_is_valid` 在刚体被销毁后返回 `false`，适合在持有 `BodyID` 的延迟逻辑里做安全检查。

## 完整示例

下面的程序搭一个地面、一个传感器和一个自由下落的球，逐步读取全部事件。

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::Sphere;
use natsel::types::shape_def::ShapeDef;

fn main() {
    let mut world = World::default();
    world.enable_sleep = false;

    // 地面
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let mut ground_def = ShapeDef::default();
    ground_def.enable_contact_events = true;
    let ground_shape = world.create_hull_shape(
        ground,
        &ground_def,
        &make_box_hull(10.0, 0.5, 10.0),
    );

    // 传感器：一块悬在空中的薄板
    let mut sensor_def = ShapeDef::default();
    sensor_def.is_sensor = true;
    sensor_def.enable_sensor_events = true;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 3.0, 0.0);
    let sensor_body = world.create_body(&def);
    let sensor_shape = world.create_hull_shape(
        sensor_body,
        &sensor_def,
        &make_box_hull(2.0, 0.5, 2.0),
    );

    // 下落的球：传感器事件与接触/命中事件都要开启
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 8.0, 0.0);
    let ball = world.create_body(&def);

    let mut ball_def = ShapeDef::default();
    ball_def.density = 1000.0;
    ball_def.enable_sensor_events = true;   // visitor 也必须开启
    ball_def.enable_contact_events = true;
    ball_def.enable_hit_events = true;
    ball_def.base_material.user_material_id = 7;
    let _ = world.create_sphere_shape(ball, &ball_def, Sphere::new(Vec3::ZERO, 0.5));

    println!("ground={} sensor={}", ground_shape.index1, sensor_shape.index1);

    let mut last_hit_contact = None;

    for step in 0..240 {
        world.step(1.0 / 60.0, 4);

        // 1. 移动与睡眠
        for event in world.body_events() {
            if event.body_id == ball && event.fell_asleep {
                println!("step {}: 球入睡", step);
            }
        }

        // 2. 传感器进出
        let sensors = world.sensor_events();
        for event in &sensors.begin {
            println!(
                "step {}: 进入传感器 {} <- {}",
                step, event.sensor_shape_id.index1, event.visitor_shape_id.index1
            );
        }
        for event in &sensors.end {
            println!(
                "step {}: 离开传感器 {} <- {}",
                step, event.sensor_shape_id.index1, event.visitor_shape_id.index1
            );
        }

        // 3. 接触开始
        let contacts = world.contact_events();
        for event in &contacts.begin_events {
            println!(
                "step {}: 接触开始 {} <-> {}",
                step, event.shape_id_a.index1, event.shape_id_b.index1
            );
        }

        // 4. 命中：去重，避免重复处理同一次撞击
        for event in &contacts.hit_events {
            if last_hit_contact == Some(event.contact_id) {
                continue;
            }
            last_hit_contact = Some(event.contact_id);
            println!(
                "step {}: 撞击速度 {:.2}，材质 {} / {}",
                step, event.approach_speed, event.user_material_id_a, event.user_material_id_b
            );
        }

        // 5. CCD 冲击：不受阈值过滤
        for impact in world.continuous_impacts() {
            println!("step {}: CCD 冲击 {:.2}", step, impact.approach_speed);
        }
    }

    println!("接触数 {}，清醒刚体 {}", world.contact_count(), world.awake_body_count());
}
```

## 常见问题

**事件必须在下一次 `step` 前读完。** 缓冲区在 `step` 开头清空。若把事件存起来延迟处理，请复制出需要的数据（`ShapeID`、`ContactID` 都是 `Copy`），而不是持有引用。

**传感器事件需要双方开启。** 这是最常见的"事件不触发"原因。检查 visitor 形状的 `enable_sensor_events`，并确认它是凸形状。

**`body_events` 是借用切片，`sensor_events` 与 `contact_events` 是副本。** 前者不能跨越可变借用保存，后两者可以。

**命中事件不会自动重置。** 用 `contact_id` 去重，或把命中处理绑定在 `begin_events` 上。

**关节事件默认不触发。** 阈值默认是 `f32::MAX`，必须用 `set_joint_force_threshold` / `set_joint_torque_threshold` 显式调低。

**事件顺序是确定的。** 接触与命中事件按接触 ID 顺序生成，因此事件序列可复现。但同一对形状在 `shape_id_a` / `shape_id_b` 中的位置不保证，判断"是谁"时两端都要比较。

**只关心撞击音效时优先用 `hit_events`。** 它已按速度阈值过滤，比遍历 `begin_events` 再自己算速度更省事。

## 相关页面

- [刚体](/docs/bodies) —— `BodyDef` 中的 `user_data`
- [形状](/docs/shapes) —— `ShapeDef` 的事件开关
- [连续碰撞检测](/docs/ccd) —— CCD 冲击事件的来源
- [关节](/docs/joints) —— 关节阈值与关节事件
- [确定性](/docs/determinism) —— 事件顺序为何可复现
