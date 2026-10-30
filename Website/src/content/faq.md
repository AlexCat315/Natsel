# 常见问题

每个条目按「症状 → 原因 → 修法」组织，修法尽量给出可直接粘贴的代码。

## 我的刚体完全不动

**症状**：创建了刚体，调用 `step` 之后位置一点没变，速度始终为零。

**原因**：`BodyDef::default()` 的 `type_` 是 `BodyType::StaticBody`。静态刚体质量为 0、不受重力、不被求解器推动。

**修法**：显式设成动态。

```rust
use natsel::math::types::Pos;
use natsel::types::body_def::{BodyDef, BodyType};

let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;   // 关键：字段名带下划线
def.position = Pos::new(0.0, 5.0, 0.0);
let body = world.create_body(&def);
```

同时确认刚体有形状——没有形状的刚体质量仍为 0，同样不会动：

```rust
world.create_hull_shape(body, &natsel::types::shape_def::ShapeDef::default(),
                        &natsel::collision::hull::make_box_hull(0.5, 0.5, 0.5));
```

如果刚体是动态的、也有形状，但仍然不动，检查是否被禁用或睡着了：

```rust
assert!(world.is_body_enabled(body));
assert!(world.body_is_awake(body));
```

## 两个物体互相穿透，不发生碰撞

**症状**：物体直穿地面或彼此而过。

按以下顺序排查：

**1. 有没有加形状？** 刚体本身不参与碰撞，只有形状参与。

```rust
let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
assert!(!shape.is_null());   // 返回 NULL 说明创建失败
```

**2. 过滤器是否把这一对挡掉了？** 默认 `category_bits` 和 `mask_bits` 全是 `1`，允许一切碰撞。如果你改过其中之一：

```rust
use natsel::types::shape_def::Filter;

// 规则：双方都要通过
// (a.mask & b.category) != 0 且 (a.category & b.mask) != 0
```

恢复默认：

```rust
world.set_shape_filter(shape, Filter::default_filter());
```

注意 `group_index` 的优先级更高：两个形状 `group_index` 相同且非零时，只由符号决定碰不碰（正数总是碰，负数总是不碰），位掩码不参与。

**3. 开了自定义过滤但没有注册回调？**

```rust
let mut shape_def = natsel::types::shape_def::ShapeDef::default();
shape_def.enable_custom_filtering = true;   // 开启后必须有回调
```

如果形状开了这个标志且世界注册了回调，回调返回 `false` 就表示不碰。回调内部逻辑写错（比如忘记处理某种组合）是最常见的“某些物体不碰”的原因：

```rust
use std::sync::Arc;

world.set_custom_filter(Some(Arc::new(|a, b| {
    // 务必确保你希望碰撞的组合在这里返回 true
    a.index1 != b.index1
})));
```

**4. 是不是被关节禁止了？** 关节默认 `collide_connected = false`，两端刚体不互相碰撞：

```rust
world.set_joint_collide_connected(joint, true);
```

**5. 高速物体隧穿？** 见下面的「物体太快穿过去」。

## `Quat::default()` 导致断言失败或物体行为诡异

**症状**：debug 构建里 panic，或者在 `create_body` / `set_body_transform` 处断言失败；release 构建里物体旋转乱飞、尺寸变成 NaN。

**原因**：`Quat` 派生的是普通 `Default`，全零。全零四元数不是有效旋转，`create_body` 会断言 `is_valid_quat(def.rotation)`——它要求四元数归一化（模长在 1 附近）。

**修法**：用 `Quat::IDENTITY`。

```rust
use natsel::math::types::{Quat, Transform};

// 错误
// def.rotation = Quat::default();

// 正确（其实 BodyDef::default() 已经是这个值，通常不用改）
def.rotation = Quat::IDENTITY;
```

`Transform` 同样：用 `Transform::IDENTITY` 而不是 `Transform::default()`。

需要构造任意旋转时，用函数生成归一化四元数：

```rust
use natsel::math::functions::make_quat_from_axis_angle;
use natsel::math::types::Vec3;

def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.7);
```

## 模拟不稳定：抖动、爆炸、堆叠塌陷

**症状**：物体抖动、堆叠物互相弹开、物体突然飞出很远。

**原因与修法**（按出现频率排序）：

**1. 时间步太大。** 固定步长是物理稳定的前提。1/60 秒配 4 子步是稳妥的起点。

```rust
world.step(1.0 / 60.0, 4);
```

不要传入变化的 `dt`（比如渲染帧间隔）——那会同时破坏稳定性和确定性。

**2. 子步太少。** `sub_step_count` 决定求解器内部的迭代次数。堆叠越高、约束越硬，需要越多子步。

```rust
world.step(1.0 / 60.0, 8);   // 高堆叠或强约束场景
```

**3. 质量比过于极端。** 密度差 1000 倍的物体放在一起，求解器很难收敛。让密度接近：

```rust
let mut shape_def = natsel::types::shape_def::ShapeDef::default();
shape_def.density = 1000.0;   // 显式统一密度，不要依赖默认值
```

**4. 速度没有上限。** `max_linear_speed` 默认 400。如果场景尺度很大，这个值可能不足或过大：

```rust
world.max_linear_speed = 1000.0;
```

**5. 接触软度不匹配。** 默认 `contact_hertz = 30.0`、`contact_damping_ratio = 10.0`。想让接触更硬：

```rust
world.set_contact_tuning(60.0, 10.0, 3.0);   // 更高频率 = 更硬的接触
```

**6. 单位设置与场景尺度不匹配。** 见 [坐标与单位](/docs/units)。

## 物体太快，直接穿过地面

**症状**：抛射物、子弹、快速移动的物体穿过薄墙或地面而不发生碰撞。

**原因**：默认的接触检测是离散的——只在每一帧的位置上检测重叠。如果一帧内移动距离超过物体自身厚度，就会从一侧跳到另一侧。

**修法**：开启连续碰撞检测（CCD）。

```rust
// 1. 世界级开关（默认已开启）
world.enable_continuous = true;

// 2. 给需要的高速物体打标记
world.set_body_bullet(projectile, true);

// 3. 调整触发阈值：值越小越早启用 CCD
world.set_body_safety_factor(projectile, 0.5);   // 默认 0.5
```

`safety_factor` 是“一帧位移超过自身最小尺寸多少倍时启用连续检测”的系数。降到 `0.2` 会更敏感地触发，代价是性能。

另外确认推测接触是开着的（默认开）：

```rust
world.enable_speculative = true;
```

还可以用 CCD 专门的事件确认是否真的触发了：

```rust
world.step(dt, 4);
for impact in world.continuous_impacts() {
    println!("CCD 冲击: {:?} 速度 {}", impact.point, impact.approach_speed);
}
```

`continuous_impacts()` 覆盖“持久接触建立之前就已解决”的冲击，普通 `contact_events().hit_events` 看不到这些。

如果两个**动态**物体高速互穿，还需要开启动态对连续检测：

```rust
world.enable_dynamic_continuous = true;   // 默认 false，按需开启
```

## 我施加的力毫无效果

**症状**：调用了 `apply_body_force`，但物体速度没有变化。

**原因**：力在每次 `step` 结束时被清零。它只影响紧接着的那一次 `step`。

**修法**：持续力需要在每帧的 `step` 之前施加。

```rust
loop {
    // 每帧都施加，而不是只加一次
    if thrust_active {
        world.apply_body_force(body, Vec3::new(0.0, 0.0, 100.0), world.body_position(body), true);
    }
    world.step(1.0 / 60.0, 4);
}
```

一次性推动用冲量，它是立即且永久的：

```rust
world.apply_body_linear_impulse(body, Vec3::new(0.0, 8.0, 0.0), world.body_position(body), true);
```

另外两个常见原因：

- 施力点在刚体中心线之外造成扭矩，看起来“没推动”其实在旋转。
- 刚体是静态的——静态刚体忽略一切力和冲量，静默返回不报错。

## 我的 `BodyID` 突然失效了

**症状**：之前拿到的 `BodyID` 现在让操作静默无效，或者 debug 构建里断言失败。

**原因**：刚体被销毁后，它的槽位会被回收给新刚体。`generation` 计数器递增，使得旧 id 不再匹配。这是有意的设计——它让“使用已释放对象”这种错误可被检测，而不是静默访问别人的数据。

```rust
use natsel::types::ids::BodyID;

let body = world.create_body(&def);
let stale = body;

world.destroy_body(body);

// 之后创建的新刚体会复用同一个槽位，但 generation 不同
let fresh = world.create_body(&def);
assert_ne!(stale.generation, fresh.generation);
```

**修法**：在每次使用前检查，或者把 id 存放在随对象生命周期一起管理的地方。

```rust
if world.body_is_valid(body) {
    let position = world.body_position(body);
}
```

同样的规则适用于 `ShapeID`、`JointID`、`ContactID`：销毁后 id 失效，`generation` 区分新旧。

指定类型的检查函数：

```rust
world.body_is_valid(body);
world.joint_is_valid(joint);
world.contact_is_valid(contact);
world.vehicle_is_valid(vehicle);
world.soft_body_is_valid(soft_body);
```

注意销毁关节时传的是索引，不是 id：

```rust
world.destroy_joint(joint.index1 - 1, true);
```

## 创建了 `CharacterVirtual` 之后无法调用世界方法

**症状**：编译器报错，说 `world` 已经被可变借用。

**原因**：`CharacterVirtual` 持有 `&mut World`——角色更新需要读写世界（碰撞查询、推动动态物体）。这是 Rust 借用规则，不是 API 设计缺陷：它保证同一时刻只有一个东西在修改世界。

**修法**：不要同时持有角色和世界的其他可变借用。按顺序使用：

```rust
use natsel::character::character_virtual::{CharacterDef, CharacterVirtual};

let mut world = natsel::simulation::World::default();
let mut character = CharacterVirtual::new(&mut world, &CharacterDef::default()).unwrap();

// 角色的方法内部会借用世界
character.update(1.0 / 60.0);

// 想调用世界方法时，先放开角色
// 角色本身仍然存在，只是此刻不借用世界
```

如果确实需要在角色存活期间调用世界方法（比如推进物理），方案有两个：

**方案一**：在角色作用域外推进物理。

```rust
{
    let mut character = CharacterVirtual::new(&mut world, &def).unwrap();
    character.update(dt);
}   // character 在这里释放

world.step(dt, 4);
```

**方案二**：让世界拥有稳定地址，用裸指针跨越生命周期。这是测试代码里使用的方式，需要 `unsafe` 并自行保证世界不被移动：

```rust
let mut world = Box::new(natsel::simulation::World::default());
```

大多数游戏不需要方案二——把角色更新和物理推进分阶段执行就够了。

角色状态要独立保存，它不在世界快照里：

```rust
let ws = world.save_state();
let cs = character.save_state();
// 回滚时先世界后角色
world.restore_state(&ws);
character.restore_state(cs);
```

## 改变 `worker_count` 后结果不一样

**症状**：`worker_count = 1` 和 `worker_count = 8` 跑同一个场景，最终位置不同。

**原因**：这不应该发生。Natsel 的确定性保证明确包含“worker 数无关”。如果你观察到差异，那是引擎的缺陷。

**排查步骤**：

```rust
use natsel::simulation::World;

fn run(workers: i32) -> u64 {
    let mut world = World::default();
    world.worker_count = workers;
    // ... 完全相同的场景构建 ...

    for _ in 0..300 {
        world.step(1.0 / 60.0, 4);
    }
    world.world_state_hash()
}

assert_eq!(run(1), run(8), "worker 数影响了结果");
```

先确认差异不是来自场景构建本身（随机数、哈希表迭代顺序、未初始化的 `Vec` 顺序）。用同一个构造函数跑两遍 `run(1)` 应该一致。

如果 `run(1) == run(1)` 但 `run(1) != run(8)`，请把最小复现提交到项目 issue，附上世界状态哈希与 `worker_count`。

也检查你是否在步骤之间改了 `worker_count`——这是允许的，结果同样不应改变：

```rust
for step in 0..60 {
    world.worker_count = [1, 3, 8, 2][step % 4];
    world.step(1.0 / 60.0, 4);
}
```

## 这个 crate 需要第三方依赖吗

**不需要。** Natsel 是 `std`-only、零第三方依赖的 crate，MIT 许可，Rust 1.85+ / edition 2024。

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main" }
```

版本查询：

```rust
let version = natsel::get_version();       // { major: 0, minor: 2, revision: 0 }
assert!(natsel::is_double_precision());    // 世界坐标是 f64
```

## 为什么没有 `World::new(&WorldDef)` 这样的构造器

**症状**：找到了 `WorldDef` 结构体，但找不到用它创建世界的函数。

**原因**：`WorldDef` 存在，用于描述一整套默认设置（也是唯一携带回调与容量提示的类型），但 0.2.0 没有把它接入 `World` 的构造路径。`World::new` 只接受 `Capacity` 容量提示。

**修法**：创建世界后直接赋值设置字段，回调用专门的方法注册。

```rust
use natsel::math::types::Vec3;
use natsel::simulation::World;
use natsel::types::world_def::Capacity;

let mut world = World::new(Capacity {
    static_shape_count: 256,
    dynamic_shape_count: 2048,
    static_body_count: 16,
    dynamic_body_count: 1024,
    contact_count: 8192,
});

world.gravity = Vec3::new(0.0, -9.81, 0.0);
world.worker_count = 4;
world.enable_sleep = true;
world.enable_continuous = true;
```

```rust
use std::sync::Arc;

world.set_custom_filter(Some(Arc::new(|a, b| a.index1 != b.index1)));
world.set_pre_solve_callback(Some(Arc::new(|_a, _b, _point, _normal| true)));
```

容量只是预分配提示。给少了不会出错，只会多几次重新分配；`World::default()` 等价于 `World::new(Capacity::default())`。

## 传感器没有事件

**症状**：设置了传感器，物体明明穿过去了，事件数组却是空的。

**原因**：有三个独立的开关，必须都满足。

**1. 形状标记为传感器。**

```rust
let mut def = natsel::types::shape_def::ShapeDef::default();
def.is_sensor = true;
```

**2. 打开了传感器事件位。**

```rust
def.enable_sensor_events = true;
// 或者运行时：
// world.enable_shape_sensor_events(sensor_shape, true);
```

**3. 在 `step` 之后立刻读取。** 事件缓冲每步重置。

```rust
world.step(dt, 4);

let events = world.sensor_events();
for e in &events.begin {
    println!("进入: sensor={:?} visitor={:?}", e.sensor_shape_id, e.visitor_shape_id);
}
for e in &events.end {
    println!("离开: sensor={:?} visitor={:?}", e.sensor_shape_id, e.visitor_shape_id);
}
```

同样检查过滤器：被过滤掉的形状不会触发传感器。

## 接触事件（`contact_events`）没有内容

对照传感器那一条，接触事件需要：

```rust
let mut def = natsel::types::shape_def::ShapeDef::default();
def.enable_contact_events = true;   // 开始/结束接触
def.enable_hit_events = true;       // 撞击（还需要速度超过阈值）
```

撞击事件还有一个世界级门槛：

```rust
world.hit_event_threshold = 1.0;   // 接近速度低于此值不产生 hit 事件；设 0 表示全部报告
```

## 关节没有连接效果

**症状**：创建了关节，两个刚体仍然自由分开。

**原因与修法**：

**1. 局部坐标系没设。** 关节通过两个刚体上的局部坐标系连接。默认都在各自原点，通常不是你想要的：

```rust
use natsel::math::types::{Transform, Vec3};
use natsel::types::joint_defs::RevoluteJointDef;

let mut hinge = RevoluteJointDef::default();
hinge.base.body_id_a = frame;
hinge.base.body_id_b = door;
hinge.base.local_frame_a = Transform::new(Vec3::new(0.0, 1.0, 0.0), natsel::math::types::Quat::IDENTITY);
hinge.base.local_frame_b = Transform::new(Vec3::new(0.0, 1.0, 0.0), natsel::math::types::Quat::IDENTITY);
let joint = world.create_revolute_joint(&hinge);
assert!(world.joint_is_valid(joint));
```

**2. 刚体 id 是 `NULL`，或者两端指向同一个刚体。**

`JointDef::default()` 的 `body_id_a` / `body_id_b` 都是 `BodyID::NULL`（`index1 == 0`）。直接用它创建关节会在 debug 构建里断言失败，因为 `index1 - 1 == -1`，而且两端相同。

```rust
use natsel::types::joint_defs::DistanceJointDef;

// 错误：两个 id 都是 NULL，会触发断言
// let bad = world.create_distance_joint(&DistanceJointDef::default());

// 正确：两端都设成有效且不同的刚体
let mut good = DistanceJointDef::default();
good.base.body_id_a = anchor;
good.base.body_id_b = link;
let joint = world.create_distance_joint(&good);
```

引擎断言的是「两端不能是同一个刚体」。如果两个 id 相同（包括都是 `NULL`），debug 构建会直接 panic。

**3. 约束太软。** 默认 `constraint_hertz = 60.0`、`constraint_damping_ratio = 2.0`。太软会让关节看起来在“拖尾”：

```rust
world.set_joint_constraint_tuning(joint, 60.0, 1.0);
```

**4. 被力阈值弄断了。** 默认 `force_threshold` 和 `torque_threshold` 是 `f32::MAX`（不会断）。如果你改小了，超限会产生 `joint_event_list()` 事件：

```rust
world.step(dt, 4);
for e in world.joint_event_list() {
    println!("关节断裂: {:?}", e.joint_id);
}
```

## 我设了速度但物体不动

**症状**：`set_body_linear_velocity` 调用成功，但位置不变。

**原因**：

- 刚体是静态的。静态刚体会静默忽略速度设置。
- 刚体被禁用了。
- 刚体睡着且速度被清零了。非零速度的设置会自动唤醒刚体，但如果你先设速度再让世界睡它，睡眠会清掉速度。

**修法**：

```rust
assert!(world.is_body_enabled(body));
world.set_body_awake(body, true);
world.set_body_linear_velocity(body, Vec3::new(1.0, 0.0, 0.0));
```

如果希望它永远不睡：

```rust
world.enable_body_sleep(body, false);
```

## 物体沉降得很慢或悬空

**症状**：物体落地后慢慢下沉，或者停在地面上方一点。

**原因**：接触允许少量穿透（容差按长度单位派生），接触求解本身是软约束。重力小、接触频率低时会更明显。

**修法**：

```rust
world.set_contact_tuning(60.0, 10.0, 3.0);   // 提高接触刚度
world.step(1.0 / 60.0, 8);                    // 增加子步
```

确认重力不是被单个刚体的 `gravity_scale` 削弱了：

```rust
assert_eq!(world.body_gravity_scale(body), 1.0);
```

## `restore_state` 返回 `false`

**症状**：回滚没有生效，函数返回 `false`。

**原因**只有两个：

- 快照属于另一个世界。快照记录保存它的世界身份，只能恢复到同一个世界。
- 世界正在 `step` 之中或某个回调之内。

**修法**：在两次 `step` 之间恢复。

```rust
world.step(dt, 4);

// 正确时机
if !world.restore_state(&snapshot) {
    eprintln!("恢复失败：快照不属于这个世界，或世界正忙");
}
```

如果确实需要跨世界迁移初始状态，正确做法是重放场景构建代码，而不是搬运快照。

## 内存持续增长

**症状**：运行一段时间后内存不断上涨。

**排查**：

- **快照没释放。** 每个 `WorldState` 都是完整副本，用 `byte_count()` 监控总量，做成环形缓冲。
- **创建了刚体但没销毁。** 用 `world.body_count()` 和 `world.counters()` 观察。

```rust
let counters = world.counters();
println!(
    "刚体 {} 形状 {} 接触 {} 关节 {}",
    counters.body_count, counters.shape_count,
    counters.contact_count, counters.joint_count,
);
```

- **凸包数据库累积。** 内容相同的凸包会去重，但不同内容的会一直驻留。`world.hull_database_count()` 可以观察。

## 生产环境应该关掉断言吗

**不必手动关。** 断言在 release 构建中自动编译为空操作。

```rust
// debug 构建：断言生效，越界和非法参数会 panic
cargo run
// release 构建：断言消失
cargo run --release
```

调试期建议加 `validate` feature，它会打开额外的内部一致性校验（只在 debug 构建生效，有性能代价）：

```sh
cargo test --features validate
```

正式构建不要带这个 feature。

## 下一步

- [核心概念](/docs/concepts)：这些问题的根源通常在对象模型上。
- [坐标与单位](/docs/units)：精度和尺度相关的问题。
- [快照与状态哈希](/docs/snapshots)：回滚系统的完整用法。
- [API 速查](/docs/api-reference)：全部签名。
