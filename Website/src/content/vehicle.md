# 车辆

车辆是一个约束，把一枚动态底盘刚体和每个轮子接触到的地面连起来。轮子本身不是刚体：每一步每个轮子沿悬挂方向投射世界，悬挂、轮胎摩擦、传动系统与俯仰/侧倾限制在 `step` 的子步里和关节一起求解。

底盘由你先创建，车辆系统只在它上面挂约束。这意味着车辆天然能与其它刚体、传感器、CCD 共存。

## 模型说明

一次车辆求解包含四个环节：

| 环节 | 内容 |
|------|------|
| 地面检测 | 每个轮子按 `collision_tester`（射线 / 球 / 圆柱）沿 `suspension_direction` 投射，得到接触点与接触法线 |
| 悬挂 | 由 `suspension_min_length` 与 `suspension_max_length` 定义的弹簧-阻尼，弹簧自然长度为 `max + preload` |
| 轮胎 | 纵向摩擦按滑移率、横向摩擦按滑移角查 `LinearCurve`，再按 `max_brake_torque` / `max_hand_brake_torque` 施加制动 |
| 传动 | 引擎扭矩 → 变速箱（自动或手动）→ 差速器 → 轮子；履带车由两段履带的角速度差转向 |

轮子的 `position`、`suspension_direction` 等全部在**底盘局部坐标系**里。轮心位置等于 `position + suspension_length * suspension_direction`。

## 创建车辆

三步：创建动态底盘、配置 `VehicleDef`、调用 `create_vehicle`。

```rust
pub fn create_vehicle(&mut self, def: &VehicleDef) -> VehicleID
```

参数非法时返回 `VehicleID::default()`（即 `VehicleID::NULL`，`is_null()` 为真），不会 panic。校验内容：

- `body_id` 有效且类型为 `BodyType::DynamicBody`；
- `wheels` 非空，且每个轮子满足 `radius > 0`、`suspension_max_length >= suspension_min_length >= 0`、`position` 合法、`suspension_direction` / `wheel_forward` / `wheel_up` 长度非零；
- 每个 `anti_roll_bars` 的左右轮索引都在范围内且不相同。

### VehicleDef 字段

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `body_id` | `BodyID` | 空 ID | 动态底盘 |
| `up` | `Vec3` | `(0, 1, 0)` | 车辆上方，底盘局部系 |
| `forward` | `Vec3` | `(0, 0, 1)` | 车辆前方，底盘局部系 |
| `max_pitch_roll_angle` | `f32` | `PI` | 底盘上方与世界上方最大夹角；`>= PI` 关闭限制 |
| `wheels` | `Vec<WheelSettings>` | 空 | 轮子配置 |
| `anti_roll_bars` | `Vec<VehicleAntiRollBar>` | 空 | 防倾杆 |
| `controller` | `VehicleControllerSettings` | `Wheeled(..)` | 控制器变体 |
| `collision_tester` | `VehicleCollisionTester` | `Ray` | 轮子如何找地面 |
| `filter` | `QueryFilter` | 默认 | 轮子碰撞的形状过滤；底盘自身形状与传感器总被忽略 |
| `max_slope_angle` | `f32` | `80°` | 相对悬挂更陡的接触被视为无效 |
| `user_data` | `Option<*mut c_void>` | `None` | 用户指针 |

`VehicleDef::new()` 等价于 `VehicleDef::default()`。

### VehicleCollisionTester

| 变体 | 说明 |
|------|------|
| `VehicleCollisionTester::Ray` | 从悬挂挂点射出一条射线。开销最低，但会漏掉射线与轮胎边缘之间的路缘 |
| `VehicleCollisionTester::Sphere` | 用轮半径的球沿悬挂方向扫掠 |
| `VehicleCollisionTester::Cylinder(f32)` | 用与轮子匹配的圆角圆柱扫掠，最精确；参数是用于圆角的半径/半宽比例 |

`Cylinder` 带一个 `f32` 载荷，即圆角所用的半径/半宽比例。

### WheelSettings 字段

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `position` | `Vec3` | 零 | 悬挂挂点 |
| `suspension_force_point` | `Vec3` | 零 | 悬挂力作用点，配合下一条使用 |
| `enable_suspension_force_point` | `bool` | `false` | 在力作用点施力而不是轮胎接触点；把作用点上移可减少车身侧倾 |
| `suspension_direction` | `Vec3` | `(0, -1, 0)` | 悬挂伸出方向 |
| `steering_axis` | `Vec3` | `(0, 1, 0)` | 转向轴 |
| `wheel_up` | `Vec3` | `(0, 1, 0)` | 未转向时的轮子上方 |
| `wheel_forward` | `Vec3` | `(0, 0, 1)` | 未转向时的滚动方向 |
| `suspension_min_length` | `f32` | `0.3` | 最短悬挂长度，超过由刚性止点抵抗 |
| `suspension_max_length` | `f32` | `0.5` | 最长悬挂长度；地面更远则失去接触 |
| `suspension_preload_length` | `f32` | `0.0` | 弹簧自然长度 = `max + preload`，正值让弹簧在完全伸展时仍被压缩 |
| `suspension_frequency` | `f32` | `1.5` | 弹簧刚度（Hz），相对于该轮承担的簧上质量 |
| `suspension_damping_ratio` | `f32` | `0.5` | 阻尼比（无量纲） |
| `suspension_stiffness` | `f32` | `0.0` | 绝对刚度（N/m）；为正值时覆盖 `suspension_frequency` |
| `suspension_damping` | `f32` | `0.0` | 绝对阻尼（N·s/m）；`suspension_stiffness` 为正值时生效 |
| `radius` | `f32` | `0.3` | 轮半径 |
| `width` | `f32` | `0.1` | 轮宽，圆柱碰撞测试与调试绘制使用 |
| `inertia` | `f32` | `0.9` | 轮子绕轴的转动惯量（kg·m²） |
| `angular_damping` | `f32` | `0.2` | 轮子角阻尼：`dw/dt = -c * w` |
| `max_steer_angle` | `f32` | `70°` | 最大转向角（弧度） |
| `max_brake_torque` | `f32` | `1500.0` | 满刹车力矩（N·m） |
| `max_hand_brake_torque` | `f32` | `4000.0` | 满手刹力矩（N·m） |
| `longitudinal_friction` | `LinearCurve` | 见下 | 纵向摩擦系数随滑移率 `\|w r - v\| / \|v\|` 变化 |
| `lateral_friction` | `LinearCurve` | 见下 | 横向摩擦系数随滑移角（度）变化 |
| `track_longitudinal_friction` | `f32` | `4.0` | 履带轮纵向摩擦 |
| `track_lateral_friction` | `f32` | `2.0` | 履带轮横向摩擦 |

默认的纵向曲线是 `[(0.0, 0.0), (0.06, 1.2), (0.2, 1.0)]`，横向曲线是 `[(0.0, 0.0), (3.0, 1.2), (20.0, 1.0)]`。`WheelSettings::new()` 就是 `Default`。

### 摩擦曲线

`LinearCurve` 是分段线性曲线，两端钳制：

```rust
pub fn new(points: &[Vec2]) -> Self      // 按 x 升序排序后存储
pub fn add_point(&mut self, x: f32, y: f32)
pub fn evaluate(&self, x: f32) -> f32    // 空曲线返回 0
```

调整轮胎抓地力的常用做法是改 `longitudinal_friction` 与 `lateral_friction`：

```rust
use natsel::math::types::Vec2;
use natsel::vehicle::vehicle_types::{LinearCurve, WheelSettings};

let mut wheel = WheelSettings::new();
wheel.longitudinal_friction = LinearCurve::new(&[
    Vec2::new(0.0, 0.0),
    Vec2::new(0.08, 1.4),
    Vec2::new(0.25, 1.1),
]);
wheel.lateral_friction = LinearCurve::new(&[
    Vec2::new(0.0, 0.0),
    Vec2::new(4.0, 1.5),
    Vec2::new(25.0, 1.2),
]);
```

## 控制器

`VehicleControllerSettings` 有三个变体：

```rust
pub enum VehicleControllerSettings {
    Wheeled(WheeledVehicleControllerSettings),
    Motorcycle(MotorcycleControllerSettings),
    Tracked(TrackedVehicleControllerSettings),
}
```

`WheeledVehicleControllerSettings` 组合了 `engine: VehicleEngineSettings`、`transmission: VehicleTransmissionSettings`、`differentials: Vec<VehicleDifferentialSettings>` 与 `differential_limited_slip_ratio`（默认 `1.4`；用 `f32::MAX` 表示开放式中央差速器）。

`MotorcycleControllerSettings` 在 `wheeled` 之上增加倾斜控制：`max_lean_angle`（默认 45°）、`lean_spring_constant`（5000）、`lean_spring_damping`（1000）、`lean_spring_integration_coefficient`、`lean_spring_integration_coefficient_decay`（4.0）、`lean_smoothing_factor`（0.8）、`enable_lean_steering_limit`（`true`）。

`TrackedVehicleControllerSettings` 有 `engine`、`transmission` 与 `tracks: Vec<VehicleTrackSettings>`（默认两条履带）。每条履带的 `VehicleTrackSettings` 含 `driven_wheel`、`wheels`、`inertia`（10.0）、`angular_damping`（0.5）、`max_brake_torque`（15000.0）、`differential_ratio`（6.0）。

引擎与变速箱的关键默认值：

| 类型 | 字段 | 默认 |
|------|------|------|
| `VehicleEngineSettings` | `max_torque` / `min_rpm` / `max_rpm` | `500.0` / `1000.0` / `6000.0` |
| `VehicleEngineSettings` | `inertia` / `angular_damping` | `0.5` / `0.2` |
| `VehicleTransmissionSettings` | `mode` | `TransmissionMode::Automatic` |
| `VehicleTransmissionSettings` | `gear_ratios` | `[2.66, 1.78, 1.3, 1.0, 0.74]` |
| `VehicleTransmissionSettings` | `reverse_gear_ratios` | `[-2.90]`（必须为负） |
| `VehicleTransmissionSettings` | `switch_time` / `clutch_release_time` / `switch_latency` | `0.5` / `0.3` / `0.5` |
| `VehicleTransmissionSettings` | `shift_up_rpm` / `shift_down_rpm` | `4000.0` / `2000.0` |
| `VehicleTransmissionSettings` | `clutch_strength` | `10.0` |

`VehicleDifferentialSettings` 的 `left_wheel` / `right_wheel` 用 `-1` 表示该侧没有驱动轮；`limited_slip_ratio` 用 `f32::MAX` 表示开放式差速器。

## 完整示例

```rust
use natsel::base::constants as b3;
use natsel::collision::hull::{make_box_hull, make_offset_box_hull};
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;
use natsel::vehicle::vehicle_types::{
    VehicleAntiRollBar, VehicleCollisionTester, VehicleControllerSettings, VehicleDef,
    VehicleDifferentialSettings, WheelSettings, WheeledVehicleControllerSettings,
};

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);

// 1. 地面
let ground = world.create_body(&BodyDef::default());
let mut ground_def = ShapeDef::default();
ground_def.base_material.friction = 0.9;
let _ = world.create_hull_shape(
    ground,
    &ground_def,
    &make_offset_box_hull(200.0, 1.0, 200.0, Vec3::new(0.0, -1.0, 0.0)),
);

// 2. 动态底盘：x 左、y 上、z 前
let mut body_def = BodyDef::default();
body_def.type_ = BodyType::DynamicBody;
body_def.position = Pos::new(0.0, 1.2, 0.0);
let chassis = world.create_body(&body_def);

let (half_width, half_length, half_height) = (0.9f32, 2.0f32, 0.25f32);
let volume = 8.0 * half_width * half_length * half_height;
let mut chassis_shape = ShapeDef::default();
chassis_shape.density = 1500.0 / volume;
let _ = world.create_hull_shape(
    chassis,
    &chassis_shape,
    &make_box_hull(half_width, half_height, half_length),
);

// 3. 轮子：前两个转向，后两个手刹
let mut def = VehicleDef::default();
def.body_id = chassis;
def.collision_tester = VehicleCollisionTester::Ray;
def.max_pitch_roll_angle = b3::PI;

let positions = [
    Vec3::new(half_width, -0.1, half_length - 0.4),
    Vec3::new(-half_width, -0.1, half_length - 0.4),
    Vec3::new(half_width, -0.1, -half_length + 0.4),
    Vec3::new(-half_width, -0.1, -half_length + 0.4),
];
for (i, p) in positions.iter().enumerate() {
    let mut wheel = WheelSettings::new();
    wheel.position = *p;
    wheel.radius = 0.35;
    wheel.width = 0.25;
    wheel.suspension_min_length = 0.1;
    wheel.suspension_max_length = 0.4;
    wheel.max_steer_angle = if i < 2 { 30.0 * b3::DEG_TO_RAD } else { 0.0 };
    wheel.max_hand_brake_torque = if i < 2 { 0.0 } else { 4000.0 };
    def.wheels.push(wheel);
}

def.anti_roll_bars = vec![
    VehicleAntiRollBar {
        left_wheel: 0,
        right_wheel: 1,
        ..VehicleAntiRollBar::default()
    },
    VehicleAntiRollBar {
        left_wheel: 2,
        right_wheel: 3,
        ..VehicleAntiRollBar::default()
    },
];

// 4. 前后轴各 50% 扭矩
let mut controller = WheeledVehicleControllerSettings::new();
controller.differentials = vec![
    VehicleDifferentialSettings {
        left_wheel: 0,
        right_wheel: 1,
        engine_torque_ratio: 0.5,
        ..VehicleDifferentialSettings::default()
    },
    VehicleDifferentialSettings {
        left_wheel: 2,
        right_wheel: 3,
        engine_torque_ratio: 0.5,
        ..VehicleDifferentialSettings::default()
    },
];
def.controller = VehicleControllerSettings::Wheeled(controller);

let car = world.create_vehicle(&def);
assert!(!car.is_null());

// 5. 驾驶
let dt: f32 = 1.0 / 60.0;
for _ in 0..300 {
    // forward, right, brake, hand_brake
    world.set_vehicle_driver_input(car, 1.0, 0.0, 0.0, 0.0);
    world.step(dt, 4);
}

let speed = world.vehicle_forward_speed(car);
let state = world.vehicle_drivetrain_state(car).expect("vehicle");
println!("forward speed = {speed}, rpm = {}, gear = {}", state.engine_rpm, state.current_gear);

world.destroy_vehicle(car);
```

## 驾驶输入

| 方法 | 说明 |
|------|------|
| `set_vehicle_driver_input(id, forward, right, brake, hand_brake)` | 轮式车与摩托。`forward` 为 `-1..1`，`right` 为 `-1..1`（负左正右），`brake` 与 `hand_brake` 为 `0..1` |
| `set_tracked_vehicle_driver_input(id, forward, left_ratio, right_ratio, brake)` | 履带车。左右比例在 `-1..1`，相反符号可原地转向 |
| `set_vehicle_transmission(id, gear, clutch_friction)` | 仅 `TransmissionMode::Manual` 生效 |

所有输入都会被夹到有效区间。自动变速箱下，与行进方向相反的 `forward` 会先刹车直到几乎停住，再进入倒挡。有效输入会唤醒睡眠中的底盘。

## 运行时调校

| 方法 | 说明 |
|------|------|
| `vehicle_body(id)` | 底盘 `BodyID`，无效车辆返回 `None` |
| `vehicle_wheel_count(id)` | 轮子数量，无效返回 `0` |
| `vehicle_wheel_settings(id, wheel)` | 读取某轮的 `WheelSettings`（克隆），越界返回 `None` |
| `set_vehicle_wheel_settings(id, wheel, &settings)` | 替换某轮配置；半径或悬挂长度非法时静默忽略 |
| `set_vehicle_engine(id, &engine)` | 替换引擎配置，当前转速会被夹到新范围 |
| `set_vehicle_transmission_settings(id, &settings)` | 替换变速箱配置，当前挡位若仍存在则保留 |
| `set_vehicle_differentials(id, &[differentials])` | 替换差速器；越界的轮索引会被过滤掉 |
| `set_vehicle_anti_roll_bars(id, &[bars])` | 替换防倾杆；越界的索引会被过滤掉 |
| `set_vehicle_world_up(id, Option<Vec3>)` | 俯仰/侧倾限制与摩托倾斜使用的世界上方；`None` 表示用负重力方向 |
| `set_vehicle_max_pitch_roll_angle(id, angle)` | 修改俯仰/侧倾限制 |
| `set_vehicle_collision_tester(id, tester)` | 切换轮子地面检测方式 |
| `set_vehicle_tire_friction_callback(id, Option<TireFrictionCallback>)` | 按轮自定义摩擦 |

`TireFrictionCallback` 的类型是 `Arc<dyn Fn(i32, f32, f32, f32, u64) -> (f32, f32) + Send + Sync>`：参数为轮索引、曲线纵向摩擦、曲线横向摩擦、地面摩擦、地面 `user_material_id`，返回 `(纵向, 横向)` 摩擦系数。

## 状态读取

`vehicle_wheel_state(id, wheel)` 返回 `Option<VehicleWheelState>`：

| 字段 | 类型 | 说明 |
|------|------|------|
| `has_contact` | `bool` | 本轮是否接触地面 |
| `contact_body_id` / `contact_shape_id` | `BodyID` / `ShapeID` | 无接触时为空 ID |
| `contact_position` | `Pos` | 接触点 |
| `contact_normal` | `Vec3` | 接触法线 |
| `contact_longitudinal` | `Vec3` | 投影到地面的滚动方向 |
| `contact_lateral` | `Vec3` | 地面上的侧向 |
| `suspension_length` | `f32` | 当前悬挂长度 |
| `suspension_force` | `f32` | 上一步的悬挂力（N） |
| `longitudinal_force` / `lateral_force` | `f32` | 上一步的轮胎力（N） |
| `longitudinal_slip` | `f32` | 滑移率 `\|w r - v\| / \|v\|` |
| `lateral_slip` | `f32` | 滑移角（弧度） |
| `angular_velocity` | `f32` | 轮子角速度 |
| `rotation_angle` | `f32` | 累计转动角度 |
| `steer_angle` | `f32` | 当前转向角 |

力由冲量乘 `1 / dt` 得到，所以只有在 `step` 之后读取才有意义。

`vehicle_drivetrain_state(id)` 返回 `Option<VehicleDrivetrainState>`：

| 字段 | 类型 | 说明 |
|------|------|------|
| `engine_rpm` | `f32` | 引擎转速 |
| `current_gear` | `i32` | 当前挡位；`0` 空挡，负数倒挡 |
| `clutch_friction` | `f32` | 离合器摩擦 |
| `is_switching_gear` | `bool` | 正在换挡 |
| `lean_angle` | `f32` | 摩托倾斜角（弧度） |
| `track_angular_velocity` | `Vec2` | 履带角速度（左右） |

其它读取接口：

| 方法 | 说明 |
|------|------|
| `vehicle_forward_speed(id)` | 底盘沿车辆前方的速度；无效车辆返回 `0.0` |
| `vehicle_wheel_local_transform(id, wheel, wheel_right, wheel_up)` | 轮子在底盘坐标系下的变换；旋转把 `wheel_right` 映射到轴、`wheel_up` 映射到轮子上方 |
| `vehicle_wheel_transform(id, wheel, wheel_right, wheel_up)` | 同上，但结果在世界坐标系 |
| `vehicle_user_data(id)` | 用户指针 |
| `vehicle_is_valid(id)` / `vehicle_count()` | 有效性 / 存活数量 |

`vehicle_wheel_local_transform` 与 `vehicle_wheel_transform` 都返回 `Option`，且需要你显式给出模型坐标轴 `wheel_right` 与 `wheel_up`（没有默认值，必须显式传，常用 `Vec3::AXIS_X` / `Vec3::AXIS_Y`）。

## 注意事项

- **底盘必须先存在**：`create_vehicle` 要求 `body_id` 是一个有效的**动态**刚体。静态或运动学底盘会让函数返回空 ID。
- **轮子的坐标在底盘局部系**：`position` 是相对底盘质心的挂点，不是世界坐标，也不是轮心——轮心还要沿悬挂方向偏移 `suspension_length`。
- **没有默认参数**：`set_vehicle_driver_input` 的 `brake` 与 `hand_brake` 没有默认值，四个数必须全部写出，例如 `set_vehicle_driver_input(car, 1.0, 0.0, 0.0, 0.0)`。
- **自动变速箱的反向输入是刹车**：`forward` 与行进相反时不会立刻倒车，而是先减速到接近静止。要强制倒车请用 `TransmissionMode::Manual` 配合 `set_vehicle_transmission`。
- **`reverse_gear_ratios` 必须为负**：填正值会让倒挡表现异常。
- **`max_pitch_roll_angle >= PI` 等于关闭限制**：默认值就是 `PI`，所以默认车辆可以翻车。需要防翻车请设一个小角度。
- **`set_vehicle_wheel_settings` 会静默失败**：半径非正、或 `suspension_max_length < suspension_min_length` 时直接返回，不报错。
- **车辆随 `step` 一起求解**：不要期望在 `step` 之外调用任何"更新车辆"的方法；输入只是写入记录，真正的求解发生在下一次 `step` 的子步中。
- **`destroy_vehicle` 不动底盘**：它只释放约束槽位。底盘刚体需要你自己 `destroy_body`。
- **`filter` 不覆盖底盘自身**：底盘形状与传感器总被轮子忽略，无需手工排除。

## 相关页面

- [刚体](/docs/bodies)：底盘是 `DynamicBody`，`BodyDef` 与 `BodyType` 在那一页。
- [形状](/docs/shapes)：底盘用 `create_hull_shape` 等构建。
- [关节](/docs/joints)：车辆在内部按关节方式求解，理解求解顺序有助于调参。
- [材质与过滤](/docs/materials)：`QueryFilter` 与形状摩擦影响轮胎抓地。
- [调试绘制](/docs/debug-draw)：`DebugDraw::draw_vehicles` 可画出轮子与防倾杆。
- [虚拟角色](/docs/character)：驾驶员的角色控制器常与车辆配合使用。
