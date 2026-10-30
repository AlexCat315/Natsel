# API 速查

按功能分组的签名速查。所有签名照抄自 v0.2.0 源码，可直接复制使用。内部用途的方法（标记 `pub(crate)` 的）不在此列。

## 导入路径约定

```rust
// 世界
use natsel::simulation::World;

// 数学
use natsel::math::types::{AABB, Matrix3, Plane, Pos, Quat, Transform, Vec2, Vec3, WorldTransform};
use natsel::math::functions::{make_quat_from_axis_angle, normalize, rotate_vector};

// 定义结构体
use natsel::types::body_def::{BodyDef, BodyType, MotionLocks};
use natsel::types::shape_def::{Filter, QueryFilter, ShapeDef, ShapeType, SurfaceMaterial};
use natsel::types::world_def::{Capacity, Counters, ExplosionDef, Profile, WorldDef};
use natsel::types::geometry::{Capsule, MassData, Mesh, Sphere};
use natsel::types::ids::{BodyID, ContactID, JointID, ShapeID, WorldID};
use natsel::types::queries::{RayResult, ShapeCastInput, ShapeProxy};
use natsel::types::events::{ContactData, ContactEvents, SensorEvents};

// 几何构建
use natsel::collision::hull::{make_box_hull, make_cube_hull, make_offset_box_hull, make_scaled_box_hull, make_transformed_box_hull};
use natsel::collision::hull_builder::create_hull;
use natsel::collision::mesh::TriangleMesh;
use natsel::collision::mesh::build::{MeshBuildOptions, prepare_mesh};
use natsel::collision::height_field::HeightField;
use natsel::collision::height_field::factories::{create_grid_height_field, create_wave_height_field};
use natsel::collision::compound::Compound;

// 关节定义
use natsel::types::joint_defs::{
    JointDef, JointType, DistanceJointDef, RevoluteJointDef, PrismaticJointDef,
    SphericalJointDef, WeldJointDef, WheelJointDef, MotorJointDef, GearJointDef,
    RackAndPinionJointDef, PulleyJointDef, FilterJointDef, ParallelJointDef,
};
```

版本与能力查询：

```rust
pub fn get_version() -> Version                  // { major: 0, minor: 2, revision: 0 }
pub fn is_double_precision() -> bool             // true
pub fn get_max_manifold_points() -> i32          // 4
```

## 世界与推进

```rust
pub fn new(capacity: Capacity) -> Self
pub fn default() -> Self                                  // == new(Capacity::default())
pub fn step(&mut self, time_step: f32, sub_step_count: i32)
```

`step` 的行为约定：

- `time_step` 必须 `>= 0.0`。传 `0.0` 只刷新事件与传感器，不推进物理。
- `sub_step_count` 会被钳到至少 1。
- 力的累积在 `step` 结束时清零，事件缓冲在 `step` 开始时清空。
- 固定 `time_step` 是确定性的前提。

可写世界字段：

```rust
world.gravity = Vec3::new(0.0, -10.0, 0.0);   // 默认 (0,-10,0)
world.worker_count = 4;                        // 默认 1
world.enable_sleep = true;
world.enable_warm_starting = true;
world.enable_continuous = true;
world.enable_dynamic_continuous = false;
world.enable_speculative = true;
world.enable_wide_solver = true;
world.enable_soft_body_query_acceleration = true;
world.max_linear_speed = 400.0;
world.hit_event_threshold = 1.0;
world.restitution_threshold = 1.0;
world.restitution_iterations = 2;
world.enable_restitution_propagation = false;
world.isotropic_inertia_tolerance = 0.2;
world.contact_speed = 3.0;
world.contact_hertz = 30.0;
world.contact_damping_ratio = 10.0;
world.contact_recycle_distance = 0.05;         // 由长度单位派生
world.profile = Profile::default();            // 只读用途，引擎每步覆盖
```

回调与调优：

```rust
pub fn set_custom_filter(&mut self, callback: Option<CustomFilterCallback>)
pub fn set_pre_solve_callback(&mut self, callback: Option<PreSolveCallback>)
pub fn set_contact_tuning(&mut self, hertz: f32, damping_ratio: f32, contact_speed: f32)

pub fn enable_sleeping(&mut self, enable: bool)
pub fn enable_warm_starting(&mut self, enable: bool)
pub fn enable_continuous(&mut self, enable: bool)
pub fn is_sleeping_enabled(&self) -> bool
pub fn is_warm_starting_enabled(&self) -> bool
pub fn is_continuous_enabled(&self) -> bool
```

## 刚体

### 创建与销毁

```rust
pub fn create_body(&mut self, def: &BodyDef) -> BodyID
pub fn destroy_body(&mut self, body_id: BodyID)
pub fn body_is_valid(&self, body_id: BodyID) -> bool
```

`destroy_body` 会连带销毁挂在该刚体上的形状、接触和关节。

### 变换与位置

```rust
pub fn body_position(&self, id: BodyID) -> Pos
pub fn body_rotation(&self, id: BodyID) -> Quat
pub fn body_transform(&self, id: BodyID) -> WorldTransform
pub fn set_body_transform(&mut self, id: BodyID, position: Pos, rotation: Quat)

pub fn body_local_point(&self, id: BodyID, world_point: Pos) -> Vec3
pub fn body_world_point(&self, id: BodyID, local_point: Vec3) -> Pos
pub fn body_local_vector(&self, id: BodyID, world_vector: Vec3) -> Vec3
pub fn body_world_vector(&self, id: BodyID, local_vector: Vec3) -> Vec3

pub fn body_world_center(&self, id: BodyID) -> Pos
pub fn body_local_center(&self, id: BodyID) -> Vec3
```

`set_body_transform` 会同步更新宽相代理，可以在两次 `step` 之间瞬移刚体。

### 速度

```rust
pub fn body_linear_velocity(&self, id: BodyID) -> Vec3
pub fn body_angular_velocity(&self, id: BodyID) -> Vec3
pub fn set_body_linear_velocity(&mut self, id: BodyID, linear_velocity: Vec3)
pub fn set_body_angular_velocity(&mut self, id: BodyID, angular_velocity: Vec3)

pub fn body_local_point_velocity(&self, id: BodyID, local_point: Vec3) -> Vec3
pub fn body_world_point_velocity(&self, id: BodyID, world_point: Pos) -> Vec3
```

对静态刚体设速度是空操作（静默返回，不 panic）。

### 力与冲量

```rust
pub fn apply_body_force(&mut self, id: BodyID, force: Vec3, point: Pos, wake: bool)
pub fn apply_body_force_to_center(&mut self, id: BodyID, force: Vec3, wake: bool)
pub fn apply_body_torque(&mut self, id: BodyID, torque: Vec3, wake: bool)

pub fn apply_body_linear_impulse(&mut self, id: BodyID, impulse: Vec3, point: Pos, wake: bool)
pub fn apply_body_linear_impulse_to_center(&mut self, id: BodyID, impulse: Vec3, wake: bool)
pub fn apply_body_angular_impulse(&mut self, id: BodyID, impulse: Vec3, wake: bool)
```

| 方法 | 生命周期 | 用途 |
|------|----------|------|
| `apply_body_force` | 到下一次 `step` 结束 | 持续推力，每帧调用 |
| `apply_body_linear_impulse` | 立即且永久 | 一次性推动、爆炸、跳跃 |
| `apply_body_torque` | 到下一次 `step` 结束 | 持续扭矩 |
| `apply_body_angular_impulse` | 立即且永久 | 一次性旋转冲量 |

`wake: true` 会唤醒睡眠中的刚体。不带 `_wake` 的版本语义相同（都带 `wake` 参数）；带 `_wake` 后缀的变体是同一行为的额外入口。

### 质量

```rust
pub fn body_mass(&self, id: BodyID) -> f32
pub fn body_inverse_mass(&self, id: BodyID) -> f32
pub fn body_mass_data(&self, id: BodyID) -> MassData
pub fn set_body_mass_data(&mut self, id: BodyID, mass_data: MassData)
pub fn apply_mass_from_shapes(&mut self, id: BodyID)
pub fn apply_body_mass_from_shapes(&mut self, body_id: BodyID)

pub fn body_local_rotational_inertia(&self, id: BodyID) -> Matrix3
pub fn body_world_inverse_rotational_inertia(&self, id: BodyID) -> Matrix3
```

`apply_mass_from_shapes` 与 `apply_body_mass_from_shapes` 等价，都是“按当前形状重算质量”。

### 类型、启用、睡眠

```rust
pub fn body_type(&self, body_id: BodyID) -> BodyType
pub fn set_body_type(&mut self, body_id: BodyID, type_: BodyType)

pub fn disable_body(&mut self, body_id: BodyID)
pub fn enable_body(&mut self, body_id: BodyID)
pub fn is_body_enabled(&self, body_id: BodyID) -> bool

pub fn set_body_awake(&mut self, body_id: BodyID, awake: bool)
pub fn body_is_awake(&self, id: BodyID) -> bool
pub fn is_body_awake_id(&self, body_id: BodyID) -> bool

pub fn set_body_sleep_threshold(&mut self, id: BodyID, sleep_threshold: f32)
pub fn body_sleep_threshold(&self, id: BodyID) -> f32
pub fn enable_body_sleep(&mut self, id: BodyID, enable_sleep: bool)
pub fn body_is_sleep_enabled(&self, id: BodyID) -> bool
pub fn is_body_sleep_enabled(&self, id: BodyID) -> bool
```

`set_body_type` 不支持带复合或高度场形状的刚体。

### 阻尼、重力、锁定

```rust
pub fn set_body_linear_damping(&mut self, id: BodyID, linear_damping: f32)
pub fn body_linear_damping(&self, id: BodyID) -> f32
pub fn set_body_angular_damping(&mut self, id: BodyID, angular_damping: f32)
pub fn body_angular_damping(&self, id: BodyID) -> f32

pub fn set_body_gravity_scale(&mut self, id: BodyID, gravity_scale: f32)
pub fn body_gravity_scale(&self, id: BodyID) -> f32

pub fn set_body_motion_locks(&mut self, body_id: BodyID, locks: &MotionLocks)
pub fn body_motion_locks(&self, body_id: BodyID) -> MotionLocks
```

### 连续碰撞与快速旋转

```rust
pub fn set_body_bullet(&mut self, body_id: BodyID, bullet: bool)
pub fn is_body_bullet(&self, body_id: BodyID) -> bool
pub fn set_body_safety_factor(&mut self, body_id: BodyID, safety_factor: f32)
pub fn body_safety_factor(&self, id: BodyID) -> f32
pub fn is_body_time_of_impact(&self, body_id: BodyID) -> bool

pub fn allow_body_fast_rotation(&mut self, body_id: BodyID, allow: bool)
pub fn is_body_fast_rotation_allowed(&self, body_id: BodyID) -> bool
pub fn is_body_fast(&self, body_id: BodyID) -> bool
```

### 接触复用、名称、用户数据

```rust
pub fn enable_body_contact_recycling(&mut self, body_id: BodyID, enable: bool)
pub fn is_body_contact_recycling_enabled(&self, body_id: BodyID) -> bool

pub fn set_body_name(&mut self, body_id: BodyID, name: Option<&str>)
pub fn body_name(&self, body_id: BodyID) -> Option<&str>
pub fn body_user_data(&self, body_id: BodyID) -> Option<*mut c_void>
pub fn set_body_user_data(&mut self, body_id: BodyID, user_data: Option<*mut c_void>)
```

### 尺寸与包围盒

```rust
pub fn body_min_extent(&self, id: BodyID) -> f32
pub fn body_min_extent_value(&self, body_id: BodyID) -> f32
pub fn body_max_extent(&self, id: BodyID) -> Vec3
pub fn body_max_extent_origin(&self, id: BodyID) -> Pos
pub fn body_compute_aabb(&self, body_id: BodyID) -> AABB
pub fn body_world_aabb(&self, id: BodyID) -> WorldAABB
```

### 多形状刚体

```rust
pub fn body_shape_count(&self, body_id: BodyID) -> i32
pub fn body_joints(&self, body_id: BodyID) -> Vec<JointID>
pub fn body_closest_point(&self, body_id: BodyID, target: Vec3) -> (Vec3, f32)
```

## BodyDef 全部字段

```rust
pub struct BodyDef {
    pub type_: BodyType,          // 默认 StaticBody
    pub position: Pos,            // 默认 ZERO
    pub rotation: Quat,           // 默认 IDENTITY
    pub linear_velocity: Vec3,    // 默认 ZERO
    pub angular_velocity: Vec3,   // 默认 ZERO
    pub linear_damping: f32,      // 默认 0.0
    pub angular_damping: f32,     // 默认 0.0
    pub gravity_scale: f32,       // 默认 1.0
    pub sleep_threshold: f32,     // 默认 0.05 * units
    pub safety_factor: f32,       // 默认 0.5
    pub name: Option<String>,     // 默认 None
    pub user_data: Option<*mut c_void>,
    pub motion_locks: MotionLocks,
    pub enable_sleep: bool,       // 默认 true
    pub is_awake: bool,           // 默认 true
    pub is_bullet: bool,          // 默认 false
    pub is_enabled: bool,         // 默认 true
    pub allow_fast_rotation: bool,          // 默认 false
    pub enable_contact_recycling: bool,     // 默认 true
}
```

## 形状

### 创建

```rust
pub fn create_sphere_shape(&mut self, body_id: BodyID, def: &ShapeDef, sphere: Sphere) -> ShapeID
pub fn create_capsule_shape(&mut self, body_id: BodyID, def: &ShapeDef, capsule: Capsule) -> ShapeID
pub fn create_hull_shape(&mut self, body_id: BodyID, def: &ShapeDef, hull: &Hull) -> ShapeID
pub fn create_compound_shape(&mut self, body_id: BodyID, def: &ShapeDef, compound: &Compound) -> ShapeID
pub fn create_height_field_shape(&mut self, body_id: BodyID, def: &ShapeDef, field: HeightField) -> ShapeID
pub fn create_mesh_shape(&mut self, body_id: BodyID, def: &ShapeDef, data: TriangleMesh, scale: Vec3) -> ShapeID
```

`create_capsule_shape` 对过短的胶囊会自动降级为球体（内部半径不满足最短长度时）。

形状创建后**没有公开的删除方法**。要移除形状，销毁所属刚体。

### 查询形状

```rust
pub fn shape_type(&self, shape_id: ShapeID) -> ShapeType
pub fn shape_body(&self, shape_id: ShapeID) -> BodyID
pub fn shape_aabb(&self, shape_id: ShapeID) -> AABB
pub fn shape_world_aabb(&self, id: ShapeID) -> WorldAABB
pub fn shape_is_sensor(&self, shape_id: ShapeID) -> bool
pub fn shape_closest_point(&self, shape_id: ShapeID, target: Pos) -> Pos
```

几何读取（不匹配的类型返回 `None`）：

```rust
pub fn shape_sphere(&self, shape_id: ShapeID) -> Option<Sphere>
pub fn shape_capsule(&self, shape_id: ShapeID) -> Option<Capsule>
pub fn shape_hull(&self, shape_id: ShapeID) -> Option<Hull>
pub fn shape_mesh(&self, shape_id: ShapeID) -> Option<Mesh>
pub fn shape_height_field(&self, shape_id: ShapeID) -> Option<HeightField>
pub fn shape_compound(&self, shape_id: ShapeID) -> Option<Compound>
```

### 材质

```rust
pub fn shape_density(&self, shape_id: ShapeID) -> f32
pub fn set_shape_density(&mut self, shape_id: ShapeID, density: f32, update_body_mass: bool)

pub fn shape_friction(&self, shape_id: ShapeID) -> f32
pub fn set_shape_friction(&mut self, shape_id: ShapeID, friction: f32)
pub fn shape_restitution(&self, shape_id: ShapeID) -> f32
pub fn set_shape_restitution(&mut self, shape_id: ShapeID, restitution: f32)

pub fn shape_surface_material(&self, shape_id: ShapeID) -> SurfaceMaterial
pub fn set_shape_surface_material(&mut self, shape_id: ShapeID, material: SurfaceMaterial)

pub fn shape_mesh_material_count(&self, shape_id: ShapeID) -> i32
pub fn shape_mesh_surface_material(&self, shape_id: ShapeID, index: i32) -> Option<SurfaceMaterial>
pub fn set_shape_mesh_material(&mut self, shape_id: ShapeID, index: i32, material: SurfaceMaterial)
```

### 过滤器与事件开关

```rust
pub fn shape_filter(&self, shape_id: ShapeID) -> Filter
pub fn set_shape_filter(&mut self, shape_id: ShapeID, filter: Filter)

pub fn enable_shape_sensor_events(&mut self, shape_id: ShapeID, enable: bool)
pub fn enable_shape_contact_events(&mut self, shape_id: ShapeID, enable: bool)
pub fn enable_shape_hit_events(&mut self, shape_id: ShapeID, enable: bool)
pub fn enable_shape_pre_solve_events(&mut self, shape_id: ShapeID, enable: bool)

pub fn shape_are_sensor_events_enabled(&self, shape_id: ShapeID) -> bool
pub fn shape_are_contact_events_enabled(&self, shape_id: ShapeID) -> bool
pub fn shape_are_hit_events_enabled(&self, shape_id: ShapeID) -> bool
pub fn shape_are_pre_solve_events_enabled(&self, shape_id: ShapeID) -> bool
```

### 名称与用户数据

```rust
pub fn set_shape_name(&mut self, shape_id: ShapeID, name: Option<&str>)
pub fn shape_name(&self, shape_id: ShapeID) -> Option<&str>
pub fn shape_user_data(&self, shape_id: ShapeID) -> Option<*mut c_void>
pub fn set_shape_user_data(&mut self, shape_id: ShapeID, user_data: Option<*mut c_void>)
```

## ShapeDef 全部字段

```rust
pub struct ShapeDef {
    pub name: Option<String>,                  // 默认 None
    pub user_data: Option<*mut c_void>,        // 默认 None
    pub materials: Vec<SurfaceMaterial>,       // 默认空（网格逐三角形材质）
    pub base_material: SurfaceMaterial,        // 默认 friction 0.6
    pub density: f32,                          // 默认 1000 / units^3
    pub explosion_scale: f32,                  // 默认 1.0
    pub filter: Filter,                        // 默认全 1
    pub enable_custom_filtering: bool,         // 默认 false
    pub is_sensor: bool,                       // 默认 false
    pub enable_sensor_events: bool,            // 默认 false
    pub enable_contact_events: bool,           // 默认 false
    pub enable_hit_events: bool,               // 默认 false
    pub enable_pre_solve_events: bool,         // 默认 false
    pub invoke_contact_creation: bool,         // 默认 true
    pub update_body_mass: bool,                // 默认 true
    pub enable_speculative_contact: bool,      // 默认 true
}
```

`SurfaceMaterial` 默认值：

```rust
friction: 0.6, restitution: 0.0, rolling_resistance: 0.0,
tangent_velocity: Vec3::ZERO, user_material_id: 0, custom_color: 0
```

## 查询

### 射线

```rust
pub fn cast_ray_closest(&self, origin: Pos, translation: Vec3, filter: QueryFilter) -> RayResult
pub fn cast_ray<F>(&self, origin: Pos, translation: Vec3, filter: QueryFilter, callback: F) -> TreeStats
where F: FnMut(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32
```

回调参数依次为：形状 id、命中点、法线、fraction、用户材质 id、三角形索引、子形状索引。回调返回新的 fraction（通常就是传入值），返回 `-1.0` 表示跳过该形状。

`RayResult` 字段：

```rust
pub struct RayResult {
    pub shape_id: ShapeID,
    pub point: Pos,              // f64 世界位置
    pub normal: Vec3,
    pub user_material_id: u64,
    pub fraction: f32,
    pub triangle_index: i32,
    pub child_index: i32,
    pub node_visits: i32,
    pub leaf_visits: i32,
    pub hit: bool,
}
```

### 形状投射

```rust
pub fn shape_cast_closest(&self, origin: Pos, input: ShapeCastInput, filter: QueryFilter) -> ShapeCastResult
pub fn shape_cast<F>(&self, origin: Pos, input: ShapeCastInput, filter: QueryFilter, callback: F) -> TreeStats
where F: FnMut(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32
```

`ShapeCastInput`：

```rust
pub struct ShapeCastInput {
    pub proxy: ShapeProxy,       // 用于扫掠的凸代理
    pub translation: Vec3,       // 扫掠向量
    pub max_fraction: f32,
    pub can_encroach: bool,      // 允许起始就已重叠
}
```

`ShapeCastResult` 字段与 `RayResult` 相同，但去掉了 `node_visits` / `leaf_visits`。

### 重叠

```rust
pub fn overlap_aabb<F>(&self, aabb: AABB, filter: QueryFilter, callback: F) -> TreeStats
where F: FnMut(ShapeID) -> bool

pub fn overlap_shape<F>(&self, origin: Pos, proxy: ShapeProxy, filter: QueryFilter, callback: F) -> TreeStats
where F: FnMut(ShapeID) -> bool

pub fn overlap_world_aabb<F>(&self, bounds: WorldAABB, filter: QueryFilter, callback: F) -> TreeStats
where F: FnMut(ShapeID) -> bool
```

回调返回 `false` 时提前结束遍历。

`overlap_aabb` 用 `f32` 包围盒，是宽相近似；`overlap_world_aabb` 用 `f64` 边界并做精确复核，适合精确判定。

### 针对单个刚体的查询

```rust
pub fn body_cast_ray(
    &self, body_id: BodyID, origin: Pos, translation: Vec3,
    filter: &QueryFilter, max_fraction: f32, body_transform: WorldTransform,
) -> BodyCastResult

pub fn body_cast_shape(
    &self, body_id: BodyID, origin: Pos, proxy: ShapeProxy, translation: Vec3,
    filter: &QueryFilter, max_fraction: f32, can_encroach: bool,
    body_transform: WorldTransform,
) -> BodyCastResult

pub fn body_overlap_shape(
    &self, body_id: BodyID, origin: Pos, proxy: ShapeProxy,
    filter: &QueryFilter, body_transform: WorldTransform,
) -> bool

pub fn body_collide_mover(
    &self, body_id: BodyID, origin: Pos, mover: &Capsule,
    filter: &QueryFilter, body_transform: WorldTransform, capacity: usize,
) -> Vec<BodyPlaneResult>

pub fn body_time_of_impact_mover(
    &self, body_id: BodyID, origin: Pos, mover: &Capsule, mover_translation: Vec3,
    filter: &QueryFilter, body_transform_1: WorldTransform, body_transform_2: WorldTransform,
) -> BodyTOIResult

pub fn shape_ray_cast(&self, shape_id: ShapeID, origin: Pos, translation: Vec3) -> WorldCastOutput
```

`body_transform` 是调用方已知的该刚体世界变换；传入它可以避免内部冗余查询。

### 角色/移动体专用

```rust
pub fn collide_mover<F>(&self, origin: Pos, mover: &Capsule, filter: &QueryFilter, callback: F)
where F: FnMut(ShapeID, &[PlaneResult]) -> bool

pub fn cast_mover(&self, origin: Pos, mover: &Capsule, translation: Vec3,
                  filter: &QueryFilter, should_collide: Option<&dyn Fn(ShapeID) -> bool>) -> f32

pub fn cast_shape(&self, origin: Pos, proxy: ShapeProxy, translation: Vec3,
                  filter: &QueryFilter, hit: Option<&dyn Fn(ShapeID, Pos, Vec3, f32) -> f32>) -> f32

pub fn world_bounds(&self) -> Option<AABB>
```

## 事件

全部每步重置，必须在 `step` 之后立刻读取。

```rust
pub fn body_events(&self) -> &[BodyMoveEvent]
pub fn contact_events(&self) -> ContactEvents
pub fn sensor_events(&self) -> simulation::sensor::SensorEvents
pub fn sensor_event_buffers(&self) -> types::events::SensorEvents
pub fn continuous_impacts(&self) -> &[ContactHitEvent]
pub fn joint_event_list(&self) -> Vec<JointEvent>
```

注意传感器事件有**两个同名但字段不同的类型**，这是最容易踩的坑：

```rust
// simulation::sensor::SensorEvents —— sensor_events() 返回的
pub struct SensorEvents {
    pub begin: Vec<SensorBeginTouchEvent>,
    pub end: Vec<SensorEndTouchEvent>,
}

// types::events::SensorEvents —— sensor_event_buffers() 返回的
pub struct SensorEvents {
    pub begin_events: Vec<SensorBeginTouchEvent>,
    pub end_events: Vec<SensorEndTouchEvent>,
}
```

日常用 `sensor_events()`，读 `events.begin` / `events.end`：

```rust
let events = world.sensor_events();
for e in &events.begin {
    println!("进入: {:?} {:?}", e.sensor_shape_id, e.visitor_shape_id);
}
```

其余事件结构：

```rust
pub struct ContactEvents {
    pub begin_events: Vec<ContactBeginTouchEvent>,
    pub end_events: Vec<ContactEndTouchEvent>,
    pub hit_events: Vec<ContactHitEvent>,
}

pub struct ContactBeginTouchEvent { pub shape_id_a: ShapeID, pub shape_id_b: ShapeID, pub contact_id: ContactID }
pub struct ContactHitEvent {
    pub shape_id_a: ShapeID, pub shape_id_b: ShapeID, pub contact_id: ContactID,
    pub point: Pos, pub normal: Vec3, pub approach_speed: f32,
    pub user_material_id_a: u64, pub user_material_id_b: u64,
}
pub struct BodyMoveEvent {
    pub user_data: Option<*mut c_void>,
    pub transform: WorldTransform,
    pub body_id: BodyID,
    pub fell_asleep: bool,
}
pub struct JointEvent { pub joint_id: JointID, pub user_data: Option<*mut c_void> }
```

`continuous_impacts` 是 CCD 解析出的瞬时冲击，独立于 `hit_event_threshold`，包含“持久流形建立之前就已解决”的冲击。

接触数据：

```rust
pub fn body_contact_data(&self, body_id: BodyID, capacity: usize) -> Vec<ContactData>
pub fn shape_contact_data(&self, shape_id: ShapeID, capacity: usize) -> Vec<ContactData>
pub fn contact_is_valid(&self, contact_id: ContactID) -> bool
pub fn touching_contact_count(&self) -> i32
```

`ContactData` 含 `contact_id`、`shape_id_a`、`shape_id_b`、`manifolds: Vec<Manifold>`。每个 `Manifold` 有 1~4 个 `ManifoldPoint`，法线从形状 A 指向形状 B。

## 传感器

传感器是形状的一个标志，没有独立对象：

```rust
let mut def = ShapeDef::default();
def.is_sensor = true;
def.enable_sensor_events = true;
let sensor = world.create_hull_shape(body, &def, &make_box_hull(3.0, 1.0, 1.0));
```

运行时开关：

```rust
world.enable_shape_sensor_events(sensor, true);
world.shape_is_sensor(sensor);      // true
```

## 关节

### 创建关节

```rust
pub fn create_revolute_joint(&mut self, def: &RevoluteJointDef) -> JointID   // simulation::revolute_joint
pub fn create_distance_joint(&mut self, def: &DistanceJointDef) -> JointID   // simulation::joint
pub fn create_prismatic_joint(&mut self, def: &PrismaticJointDef) -> JointID // simulation::prismatic_joint
pub fn create_spherical_joint(&mut self, def: &SphericalJointDef) -> JointID // simulation::spherical_joint
pub fn create_weld_joint(&mut self, def: &WeldJointDef) -> JointID           // simulation::weld_joint
pub fn create_wheel_joint(&mut self, def: &WheelJointDef) -> JointID         // simulation::wheel_joint
pub fn create_motor_joint(&mut self, def: &MotorJointDef) -> JointID         // 类型在 types::joint_defs
pub fn create_parallel_joint(&mut self, def: &ParallelJointDef) -> JointID   // simulation::parallel_joint
pub fn create_filter_joint(&mut self, def: &FilterJointDef) -> JointID       // simulation::joint
pub fn create_gear_joint(&mut self, def: &GearJointDef) -> JointID           // simulation::coupling_joints
pub fn create_rack_and_pinion_joint(&mut self, def: &RackAndPinionJointDef) -> JointID
pub fn create_pulley_joint(&mut self, def: &PulleyJointDef) -> JointID
pub fn create_six_dof_joint(&mut self, def: SixDOFJointDef) -> JointID       // 按值传递
pub fn create_path_joint(&mut self, def: &PathJointDef) -> JointID
```

### 通用查询

```rust
pub fn destroy_joint(&mut self, joint_id: i32, wake_attached: bool)   // 注意：传 index1 - 1
pub fn joint_is_valid(&self, joint_id: JointID) -> bool
pub fn joint_type(&self, joint_id: JointID) -> Option<JointType>
pub fn joint_bodies(&self, joint_id: JointID) -> Option<(BodyID, BodyID)>
pub fn joint_is_awake(&self, joint_id: JointID) -> bool
pub fn joint_collide_connected(&self, joint_id: JointID) -> bool
pub fn set_joint_collide_connected(&mut self, joint_id: JointID, should_collide: bool)

pub fn joint_local_frame_a(&self, joint_id: JointID) -> Option<Transform>
pub fn joint_local_frame_b(&self, joint_id: JointID) -> Option<Transform>
pub fn set_joint_local_frame_a(&mut self, joint_id: JointID, frame: Transform) -> bool
pub fn set_joint_local_frame_b(&mut self, joint_id: JointID, frame: Transform) -> bool

pub fn joint_world_anchor_a(&self, joint_id: JointID) -> Option<Pos>
pub fn joint_linear_separation(&self, joint_id: JointID) -> Option<f32>
pub fn joint_angular_separation(&self, joint_id: JointID) -> Option<f32>
pub fn joint_constraint_force(&self, joint_id: JointID) -> Option<Vec3>
pub fn joint_constraint_torque(&self, joint_id: JointID) -> Option<Vec3>

pub fn joint_constraint_tuning(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_joint_constraint_tuning(&mut self, joint_id: JointID, hertz: f32, damping_ratio: f32) -> bool
pub fn joint_force_threshold(&self, joint_id: JointID) -> Option<f32>
pub fn set_joint_force_threshold(&mut self, joint_id: JointID, threshold: f32) -> bool
pub fn joint_torque_threshold(&self, joint_id: JointID) -> Option<f32>
pub fn set_joint_torque_threshold(&mut self, joint_id: JointID, threshold: f32) -> bool
```

`destroy_joint` 接收的是**索引**而不是 `JointID`，习惯用法是 `world.destroy_joint(joint.index1 - 1, true)`。

### 距离关节

```rust
pub fn set_distance_joint_length(&mut self, joint_id: JointID, length: f32)
pub fn set_distance_joint_length_range(&mut self, joint_id: JointID, min_length: f32, max_length: f32)
pub fn is_distance_joint_limit_enabled(&self, joint_id: JointID) -> bool

pub fn enable_distance_joint_spring(&mut self, joint_id: JointID, enable: bool)
pub fn is_distance_joint_spring_enabled(&self, joint_id: JointID) -> bool
pub fn set_distance_joint_spring_force_range(&mut self, joint_id: JointID, lower: f32, upper: f32)
pub fn distance_joint_spring_force_range(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_distance_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32)
pub fn set_distance_joint_spring_damping_ratio(&mut self, joint_id: JointID, ratio: f32)
pub fn distance_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32>
pub fn distance_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32>

pub fn enable_distance_joint_motor(&mut self, joint_id: JointID, enable: bool)
pub fn is_distance_joint_motor_enabled(&self, joint_id: JointID) -> bool
pub fn set_distance_joint_motor_speed(&mut self, joint_id: JointID, motor_speed: f32)
pub fn distance_joint_motor_speed(&self, joint_id: JointID) -> Option<f32>
pub fn set_distance_joint_max_motor_force(&mut self, joint_id: JointID, force: f32)
pub fn distance_joint_motor_force(&self, joint_id: JointID) -> Option<f32>
```

### 旋转关节

```rust
pub fn set_revolute_joint_limits(&mut self, joint_id: JointID, lower: f32, upper: f32) -> bool
pub fn is_revolute_joint_limit_enabled(&self, joint_id: JointID) -> bool

pub fn enable_revolute_joint_motor(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_revolute_joint_motor_enabled(&self, joint_id: JointID) -> bool
pub fn set_revolute_joint_motor_speed(&mut self, joint_id: JointID, speed: f32) -> bool
pub fn revolute_joint_motor_speed(&self, joint_id: JointID) -> Option<f32>
pub fn set_revolute_joint_max_motor_torque(&mut self, joint_id: JointID, torque: f32) -> bool
pub fn revolute_joint_motor_torque(&self, joint_id: JointID) -> Option<f32>

pub fn enable_revolute_joint_spring(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_revolute_joint_spring_enabled(&self, joint_id: JointID) -> bool
pub fn set_revolute_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool
pub fn set_revolute_joint_spring_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool
pub fn revolute_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32>
pub fn revolute_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32>
```

### 滑动关节

```rust
pub fn set_prismatic_joint_limits(&mut self, joint_id: JointID, lower: f32, upper: f32) -> bool
pub fn is_prismatic_joint_limit_enabled(&self, joint_id: JointID) -> bool
pub fn enable_prismatic_joint_motor(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_prismatic_joint_motor_enabled(&self, joint_id: JointID) -> bool
pub fn set_prismatic_joint_motor_speed(&mut self, joint_id: JointID, speed: f32) -> bool
pub fn prismatic_joint_motor_speed(&self, joint_id: JointID) -> Option<f32>
pub fn set_prismatic_joint_max_motor_force(&mut self, joint_id: JointID, force: f32) -> bool
pub fn prismatic_joint_max_motor_force(&self, joint_id: JointID) -> Option<f32>
pub fn prismatic_joint_motor_force(&self, joint_id: JointID) -> Option<f32>
pub fn enable_prismatic_joint_spring(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_prismatic_joint_spring_enabled(&self, joint_id: JointID) -> bool
pub fn set_prismatic_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool
pub fn set_prismatic_joint_spring_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool
pub fn prismatic_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32>
pub fn prismatic_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32>
pub fn set_prismatic_joint_target_translation(&mut self, joint_id: JointID, translation: f32) -> bool
pub fn prismatic_joint_target_translation(&self, joint_id: JointID) -> Option<f32>
```

### 球关节

```rust
pub fn set_spherical_joint_cone_limit(&mut self, joint_id: JointID, cone_angle: f32) -> bool
pub fn enable_spherical_joint_cone_limit(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_spherical_joint_cone_limit_enabled(&self, joint_id: JointID) -> bool
pub fn spherical_joint_cone_limit(&self, joint_id: JointID) -> Option<(bool, f32)>

pub fn set_spherical_joint_twist_limits(&mut self, joint_id: JointID, lower: f32, upper: f32) -> bool
pub fn enable_spherical_joint_twist_limit(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_spherical_joint_twist_limit_enabled(&self, joint_id: JointID) -> bool
pub fn spherical_joint_twist_limits(&self, joint_id: JointID) -> Option<(f32, f32)>

pub fn enable_spherical_joint_motor(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_spherical_joint_motor_enabled(&self, joint_id: JointID) -> bool
pub fn set_spherical_joint_motor_velocity(&mut self, joint_id: JointID, velocity: Vec3) -> bool
pub fn spherical_joint_motor_velocity(&self, joint_id: JointID) -> Option<Vec3>
pub fn set_spherical_joint_max_motor_torque(&mut self, joint_id: JointID, torque: f32) -> bool
pub fn spherical_joint_max_motor_torque(&self, joint_id: JointID) -> Option<f32>

pub fn enable_spherical_joint_spring(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_spherical_joint_spring_enabled(&self, joint_id: JointID) -> bool
pub fn set_spherical_joint_spring(&mut self, joint_id: JointID, hertz: f32, damping_ratio: f32) -> bool
pub fn spherical_joint_spring(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_spherical_joint_target_rotation(&mut self, joint_id: JointID, rotation: Quat) -> bool
pub fn spherical_joint_target_rotation(&self, joint_id: JointID) -> Option<Quat>
```

### 轮关节

```rust
pub fn enable_wheel_joint_spin_motor(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_wheel_joint_spin_motor_enabled(&self, joint_id: JointID) -> bool
pub fn set_wheel_joint_spin_speed(&mut self, joint_id: JointID, speed: f32) -> bool
pub fn wheel_joint_spin_speed(&self, joint_id: JointID) -> Option<f32>
pub fn set_wheel_joint_max_spin_torque(&mut self, joint_id: JointID, torque: f32) -> bool
pub fn wheel_joint_max_spin_torque(&self, joint_id: JointID) -> Option<f32>
pub fn wheel_joint_motor_torque(&self, joint_id: JointID) -> Option<f32>

pub fn enable_wheel_joint_suspension_spring(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_wheel_joint_suspension_enabled(&self, joint_id: JointID) -> bool
pub fn set_wheel_joint_suspension_spring(&mut self, joint_id: JointID, hertz: f32, damping_ratio: f32) -> bool
pub fn wheel_joint_suspension_spring(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_wheel_joint_suspension_limits(&mut self, joint_id: JointID, lower: f32, upper: f32) -> bool
pub fn enable_wheel_joint_suspension_limit(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_wheel_joint_suspension_limit_enabled(&self, joint_id: JointID) -> bool
pub fn wheel_joint_suspension_limits(&self, joint_id: JointID) -> Option<(f32, f32)>

pub fn enable_wheel_joint_steering(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn is_wheel_joint_steering_enabled(&self, joint_id: JointID) -> bool
pub fn set_wheel_joint_target_steering_angle(&mut self, joint_id: JointID, angle: f32) -> bool
pub fn wheel_joint_target_steering_angle(&self, joint_id: JointID) -> Option<f32>
pub fn set_wheel_joint_max_steering_torque(&mut self, joint_id: JointID, torque: f32) -> bool
pub fn wheel_joint_max_steering_torque(&self, joint_id: JointID) -> Option<f32>
pub fn set_wheel_joint_steering_spring(&mut self, joint_id: JointID, hertz: f32, damping_ratio: f32) -> bool
pub fn wheel_joint_steering_spring(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_wheel_joint_steering_limits(&mut self, joint_id: JointID, lower: f32, upper: f32) -> bool
pub fn enable_wheel_joint_steering_limit(&mut self, joint_id: JointID, enable: bool) -> bool
pub fn wheel_joint_steering_limits(&self, joint_id: JointID) -> Option<(f32, f32)>
```

### 马达关节

```rust
pub fn set_motor_joint_linear_velocity(&mut self, joint_id: JointID, velocity: Vec3) -> bool
pub fn motor_joint_linear_velocity(&self, joint_id: JointID) -> Option<Vec3>
pub fn set_motor_joint_angular_velocity(&mut self, joint_id: JointID, velocity: Vec3) -> bool
pub fn motor_joint_angular_velocity(&self, joint_id: JointID) -> Option<Vec3>
pub fn set_motor_joint_max_velocity_force(&mut self, joint_id: JointID, force: f32) -> bool
pub fn motor_joint_max_velocity_force(&self, joint_id: JointID) -> Option<f32>
pub fn set_motor_joint_max_velocity_torque(&mut self, joint_id: JointID, torque: f32) -> bool
pub fn motor_joint_max_velocity_torque(&self, joint_id: JointID) -> Option<f32>
pub fn set_motor_joint_linear_spring(&mut self, joint_id: JointID, hertz: f32, damping_ratio: f32) -> bool
pub fn motor_joint_linear_spring(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_motor_joint_angular_spring(&mut self, joint_id: JointID, hertz: f32, damping_ratio: f32) -> bool
pub fn motor_joint_angular_spring(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_motor_joint_max_spring_force(&mut self, joint_id: JointID, force: f32) -> bool
pub fn motor_joint_max_spring_force(&self, joint_id: JointID) -> Option<f32>
```

### 耦合关节

```rust
pub fn gear_joint_ratio(&self, joint_id: JointID) -> Option<f32>
pub fn set_gear_joint_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool
pub fn rack_and_pinion_joint_ratio(&self, joint_id: JointID) -> Option<f32>
pub fn set_rack_and_pinion_joint_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool
pub fn pulley_joint_ratio(&self, joint_id: JointID) -> Option<f32>
pub fn pulley_joint_ground_anchors(&self, joint_id: JointID) -> Option<(Pos, Pos)>
pub fn pulley_joint_current_lengths(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn pulley_joint_length_range(&self, joint_id: JointID) -> Option<(f32, f32)>
pub fn set_pulley_joint_length_range(&mut self, joint_id: JointID, min_length: f32, max_length: f32) -> bool
```

### 6DOF 关节

```rust
pub fn create_six_dof_joint(&mut self, def: SixDOFJointDef) -> JointID
pub fn six_dof_joint_pose(&self, joint_id: JointID) -> Option<Transform>
pub fn set_six_dof_joint_axis(&mut self, joint_id: JointID, axis: SixDOFAxis, settings: SixDOFAxisSettings) -> bool
pub fn set_six_dof_joint_motor_target(&mut self, joint_id: JointID, axis: SixDOFAxis, target: f32) -> bool
pub fn set_six_dof_joint_target_rotation(&mut self, joint_id: JointID, rotation: Quat) -> bool
```

### 路径关节

```rust
pub fn create_path_joint(&mut self, def: &PathJointDef) -> JointID
pub fn path_joint_fraction(&self, joint_id: JointID) -> Option<f32>
pub fn path_joint_path(&self, joint_id: JointID) -> Option<Arc<ConstraintPath>>
pub fn set_path_joint_path(&mut self, joint_id: JointID, path: Arc<ConstraintPath>, fraction: f32) -> bool
pub fn set_path_joint_motor(&mut self, joint_id: JointID, mode: PathMotorMode, max_force: Option<f32>) -> bool
pub fn set_path_joint_target_velocity(&mut self, joint_id: JointID, velocity: f32) -> bool
pub fn set_path_joint_target_fraction(&mut self, joint_id: JointID, fraction: f32) -> bool
pub fn set_path_joint_max_friction(&mut self, joint_id: JointID, force: f32) -> bool
pub fn set_path_joint_rotation_constraint(&mut self, joint_id: JointID, rotation: PathRotationConstraint) -> bool
```

`ConstraintPath` 构造：

```rust
use natsel::simulation::path_joint::ConstraintPath;

pub fn new(points: Vec<PathPoint>, is_looping: bool) -> Option<ConstraintPath>
pub fn through(positions: &[Vec3], up: Vec3, is_looping: bool) -> Option<ConstraintPath>
pub fn max_fraction(&self) -> f32
pub fn length(&self) -> f32
pub fn point_at(&self, fraction: f32) -> (Vec3, Vec3, Vec3, Vec3)
pub fn closest_fraction_to(&self, position: Vec3, hint: f32) -> f32
pub fn arc_length_at(&self, fraction: f32) -> f32
```

路径用 `Arc` 共享：多个路径关节可以指向同一条路径。

## 车辆

```rust
pub fn create_vehicle(&mut self, def: &VehicleDef) -> VehicleID
pub fn destroy_vehicle(&mut self, vehicle_id: VehicleID)
pub fn vehicle_is_valid(&self, vehicle_id: VehicleID) -> bool
pub fn vehicle_count(&self) -> i32
pub fn vehicle_body(&self, vehicle_id: VehicleID) -> Option<BodyID>
pub fn vehicle_wheel_count(&self, vehicle_id: VehicleID) -> i32
pub fn vehicle_forward_speed(&self, vehicle_id: VehicleID) -> f32
pub fn vehicle_user_data(&self, vehicle_id: VehicleID) -> Option<*mut c_void>
```

驾驶与调校：

```rust
pub fn set_vehicle_driver_input(
    &mut self, vehicle_id: VehicleID,
    forward: f32, right: f32, brake: f32, hand_brake: f32,
)                                                    // 各分量会自动钳制到合法范围

pub fn set_tracked_vehicle_driver_input(
    &mut self, vehicle_id: VehicleID,
    forward: f32, left_ratio: f32, right_ratio: f32, brake: f32,
)

pub fn set_vehicle_transmission(&mut self, vehicle_id: VehicleID, gear: i32, clutch_friction: f32)
pub fn set_vehicle_engine(&mut self, vehicle_id: VehicleID, settings: &VehicleEngineSettings)
pub fn set_vehicle_transmission_settings(&mut self, vehicle_id: VehicleID, settings: &VehicleTransmissionSettings)
pub fn set_vehicle_differentials(&mut self, vehicle_id: VehicleID, differentials: &[VehicleDifferentialSettings])
pub fn set_vehicle_anti_roll_bars(&mut self, vehicle_id: VehicleID, bars: &[VehicleAntiRollBar])
pub fn set_vehicle_world_up(&mut self, vehicle_id: VehicleID, up: Option<Vec3>)
pub fn set_vehicle_max_pitch_roll_angle(&mut self, vehicle_id: VehicleID, angle: f32)
pub fn set_vehicle_collision_tester(&mut self, vehicle_id: VehicleID, tester: VehicleCollisionTester)
pub fn set_vehicle_tire_friction_callback(&mut self, vehicle_id: VehicleID, callback: Option<TireFrictionCallback>)
```

轮子：

```rust
pub fn vehicle_wheel_settings(&self, vehicle_id: VehicleID, wheel: i32) -> Option<WheelSettings>
pub fn set_vehicle_wheel_settings(&mut self, vehicle_id: VehicleID, wheel: i32, settings: &WheelSettings)
pub fn vehicle_wheel_state(&self, vehicle_id: VehicleID, wheel: i32) -> Option<VehicleWheelState>
pub fn vehicle_wheel_local_transform(
    &self, vehicle_id: VehicleID, wheel: i32, wheel_right: Vec3, wheel_up: Vec3,
) -> Option<Transform>
pub fn vehicle_wheel_transform(
    &self, vehicle_id: VehicleID, wheel: i32, wheel_right: Vec3, wheel_up: Vec3,
) -> Option<WorldTransform>
pub fn vehicle_drivetrain_state(&self, vehicle_id: VehicleID) -> Option<VehicleDrivetrainState>
```

`VehicleWheelState` 字段：`has_contact`、`contact_body_id`、`contact_shape_id`、`contact_position`、`contact_normal`、`contact_longitudinal`、`contact_lateral`、`suspension_length`、`suspension_force`、`longitudinal_force`、`lateral_force`、`longitudinal_slip`、`lateral_slip`、`angular_velocity`、`rotation_angle`、`steer_angle`。

`VehicleDrivetrainState` 字段：`engine_rpm`、`current_gear`、`clutch_friction`、`is_switching_gear`、`lean_angle`、`track_angular_velocity`。

## 角色

```rust
use natsel::character::character_virtual::{CharacterDef, CharacterVirtual};

pub fn new(world: &'w mut World, def: &CharacterDef) -> Option<Self>

pub fn position(&self) -> Pos
pub fn up(&self) -> Vec3
pub fn radius(&self) -> f32
pub fn height(&self) -> f32
pub fn set_max_slope_angle(&mut self, angle: f32)

pub fn ground_state(&self) -> CharacterGroundState
pub fn ground_normal(&self) -> Vec3
pub fn ground_position(&self) -> Pos
pub fn ground_velocity(&self) -> Vec3
pub fn ground_body_id(&self) -> BodyID
pub fn ground_shape_id(&self) -> ShapeID
pub fn ground_user_material_id(&self) -> u64
pub fn is_supported(&self) -> bool
pub fn is_slope_walkable(&self, normal: Vec3) -> bool
pub fn contacts(&self) -> &[CharacterContact]
pub fn inner_body_id(&self) -> BodyID

pub fn set_position(&mut self, new_position: Pos)
pub fn set_shape(&mut self, new_radius: f32, new_height: f32, max_penetration: f32) -> bool
pub fn update(&mut self, dt: f32)
pub fn extended_update(&mut self, dt: f32, gravity: Vec3)

pub fn save_state(&self) -> State
pub fn restore_state(&mut self, state: State)
pub fn destroy(&mut self)
```

`CharacterVirtual::new` 在定义非法时返回 `None`（半径必须为正、高度至少两倍半径、位置有限、`up` 非零）。

`CharacterGroundState`：`OnGround`、`OnSteepGround`、`NotSupported`、`InAir`。

## 布娃娃与骨骼

```rust
use natsel::skeleton::ragdoll::{Ragdoll, RagdollSettings};
use natsel::skeleton::skeleton::{Skeleton, SkeletonPose};

pub fn Ragdoll::new(
    world: &'w mut World, settings: RagdollSettings,
    pose: &SkeletonPose, user_data: Option<*mut c_void>,
) -> Option<Ragdoll<'w>>

pub fn RagdollSettings::new(skeleton: Skeleton, parts: Vec<RagdollPart>) -> Self
pub fn RagdollPart::new() -> Self
pub fn RagdollPart::with_shapes(
    shapes: Vec<RagdollShape>, joint: RagdollJointKind, joint_frame: Transform,
) -> Self

pub fn bodies(&self) -> &[BodyID]
pub fn joints(&self) -> &[JointID]
pub fn filter_joints(&self) -> &[JointID]
pub fn root_body(&self) -> BodyID
pub fn pose(&self) -> SkeletonPose
pub fn set_pose(&mut self, pose: &SkeletonPose, reset_velocities: bool)
pub fn drive_to_pose_using_motors(&mut self, pose: &SkeletonPose, max_torque: f32, hertz: f32, damping_ratio: f32)
pub fn drive_to_pose_using_kinematics(&mut self, pose: &SkeletonPose, time_step: f32)
pub fn disable_motors(&mut self)
pub fn set_linear_velocity(&mut self, velocity: Vec3)
pub fn apply_linear_impulse(&mut self, impulse: Vec3)
pub fn total_mass(&self) -> f32
pub fn is_awake(&self) -> bool
pub fn wake(&mut self)
pub fn bounds(&self) -> AABB
pub fn destroy(&mut self)
```

骨骼：

```rust
pub fn Skeleton::new() -> Self
pub fn add_joint(&mut self, name: &str, parent: i32) -> i32
pub fn add_joint_with_parent_name(&mut self, name: &str, parent_name: &str) -> i32
pub fn joint_index(&self, name: &str) -> Option<i32>
pub fn joint_count(&self) -> usize
pub fn joints(&self) -> &[Joint]
pub fn parent_index(&self, joint: i32) -> i32
pub fn is_ancestor(&self, ancestor: i32, joint: i32) -> bool

pub fn SkeletonPose::new(skeleton: Skeleton) -> Self
pub fn calculate_joint_matrices(&mut self)
pub fn joint_count(&self) -> usize

// SkeletonPose 的公开字段
pub skeleton: Skeleton
pub root_offset: Pos
pub joint_states: Vec<SkeletonJointState>
pub joint_matrices: Vec<Transform>

// 每个关节的状态
pub fn SkeletonJointState::new(translation: Vec3, rotation: Quat) -> Self
pub fn SkeletonJointState::from_transform(transform: Transform) -> Self
pub fn SkeletonJointState::transform(&self) -> Transform
```

## 软体

```rust
pub fn create_soft_body(&mut self, def: &SoftBodyDef) -> SoftBodyID
pub fn destroy_soft_body(&mut self, soft_body_id: SoftBodyID)
pub fn soft_body_is_valid(&self, soft_body_id: SoftBodyID) -> bool
pub fn soft_body_count(&self) -> i32
```

顶点访问：

```rust
pub fn soft_body_vertex_count(&self, soft_body_id: SoftBodyID) -> i32
pub fn soft_body_vertex_positions(&self, soft_body_id: SoftBodyID) -> Vec<Pos>
pub fn soft_body_vertex_velocities(&self, soft_body_id: SoftBodyID) -> Vec<Vec3>
pub fn soft_body_vertex_position(&self, soft_body_id: SoftBodyID, vertex: i32) -> Option<Pos>
pub fn set_soft_body_vertex_position(&mut self, soft_body_id: SoftBodyID, vertex: i32, position: Pos)
pub fn set_soft_body_vertex_velocity(&mut self, soft_body_id: SoftBodyID, vertex: i32, velocity: Vec3)
pub fn set_soft_body_vertex_inverse_mass(&mut self, soft_body_id: SoftBodyID, vertex: i32, inv_mass: f32)
pub fn with_soft_body_vertex_positions<R, F>(&self, soft_body_id: SoftBodyID, body: F) -> Option<R>
where F: FnOnce(&[Pos]) -> R
pub fn soft_body_vertex_has_contact(&self, soft_body_id: SoftBodyID, vertex: i32) -> bool
```

整体属性与操作：

```rust
pub fn soft_body_mass(&self, soft_body_id: SoftBodyID) -> f32
pub fn soft_body_center_of_mass(&self, soft_body_id: SoftBodyID) -> Pos
pub fn soft_body_linear_velocity(&self, soft_body_id: SoftBodyID) -> Vec3
pub fn soft_body_bounds(&self, soft_body_id: SoftBodyID) -> AABB
pub fn soft_body_world_aabb(&self, id: SoftBodyID) -> Option<WorldAABB>
pub fn soft_body_volume(&self, soft_body_id: SoftBodyID) -> f32
pub fn soft_body_settings(&self, soft_body_id: SoftBodyID) -> Option<SoftBodySettings>
pub fn soft_body_contacts(&self, soft_body_id: SoftBodyID) -> Vec<SoftBodyContact>

pub fn set_soft_body_transform(&mut self, soft_body_id: SoftBodyID, position: Pos, rotation: Quat)
pub fn add_soft_body_velocity(&mut self, soft_body_id: SoftBodyID, velocity: Vec3)
pub fn apply_soft_body_linear_impulse(&mut self, soft_body_id: SoftBodyID, impulse: Vec3)
```

运行时调参：

```rust
pub fn set_soft_body_iteration_count(&mut self, soft_body_id: SoftBodyID, count: i32)
pub fn set_soft_body_pressure(&mut self, soft_body_id: SoftBodyID, pressure: f32)
pub fn soft_body_pressure(&self, soft_body_id: SoftBodyID) -> f32
pub fn set_soft_body_friction(&mut self, soft_body_id: SoftBodyID, friction: f32)
pub fn set_soft_body_restitution(&mut self, soft_body_id: SoftBodyID, restitution: f32)
pub fn set_soft_body_linear_damping(&mut self, soft_body_id: SoftBodyID, damping: f32)
pub fn set_soft_body_self_collision(&mut self, soft_body_id: SoftBodyID, enabled: bool)
pub fn set_soft_body_collide_with_soft_bodies(&mut self, soft_body_id: SoftBodyID, enabled: bool)
pub fn set_soft_body_edge_collision(&mut self, soft_body_id: SoftBodyID, enabled: bool)
pub fn set_soft_body_collision_thickness(&mut self, soft_body_id: SoftBodyID, thickness: f32)
pub fn soft_body_collision_thickness(&self, soft_body_id: SoftBodyID) -> f32
pub fn set_soft_body_gravity_scale(&mut self, soft_body_id: SoftBodyID, scale: f32)
pub fn set_soft_body_filter(&mut self, soft_body_id: SoftBodyID, filter: Filter)
pub fn soft_body_is_awake(&self, soft_body_id: SoftBodyID) -> bool
pub fn set_soft_body_awake(&mut self, soft_body_id: SoftBodyID, awake: bool)
pub fn set_soft_body_skin_transforms(&mut self, soft_body_id: SoftBodyID, transforms: &[WorldTransform], teleport: bool)
```

软体形状构造：

```rust
pub fn SoftBodySettings::make_cloth<F>(
    columns: i32, rows: i32, spacing: f32,
    attributes: SoftBodyVertexAttributes, bend_type: SoftBodyBendType, pinned: F,
) -> SoftBodySettings
pub fn SoftBodySettings::make_rope(count: i32, spacing: f32, compliance: f32, pinned: bool) -> SoftBodySettings
pub fn SoftBodySettings::make_cube(count: i32, spacing: f32, compliance: f32, volume_compliance: f32) -> SoftBodySettings
pub fn calculate_rest_values(&mut self)
pub fn optimize(&mut self)
```

`SoftBodyDef::default()` 的关键默认值：`iteration_count: 5`、`linear_damping: 0.1`、`friction: 0.2`、`restitution: 0.0`、`pressure: 0.0`、`gravity_scale: 1.0`、`vertex_radius: 0.0`、`collide_with_rigid_bodies: true`、`two_way_coupling: true`、`enable_sleep: true`、`enable_self_collision: false`、`enable_edge_collision: true`。

## 浮力

```rust
pub fn body_submerged_volume(
    &self, body_id: BodyID, surface_position: Pos, surface_normal: Vec3,
) -> SubmergedVolume

pub fn apply_buoyancy_impulse(
    &mut self, body_id: BodyID, surface_position: Pos, surface_normal: Vec3,
    buoyancy: f32, linear_drag: f32, angular_drag: f32,
    fluid_velocity: Vec3, time_step: f32,
) -> bool

pub fn apply_buoyancy(
    &mut self, volume: WorldAABB, surface_normal: Vec3,
    buoyancy: f32, linear_drag: f32, angular_drag: f32,
    fluid_velocity: Vec3, time_step: f32, filter: QueryFilter,
) -> i32

pub fn apply_buoyancy_in_aabb(
    &mut self, volume: AABB, surface_normal: Vec3,
    buoyancy: f32, linear_drag: f32, angular_drag: f32,
    fluid_velocity: Vec3, time_step: f32, filter: QueryFilter,
) -> i32
```

`SubmergedVolume`：`total_volume`、`submerged_volume`、`center_of_buoyancy`。

`apply_buoyancy` / `apply_buoyancy_in_aabb` 返回受影响刚体数量。必须在 `step` 之前调用，因为它们施加的是冲量。

## 爆炸

```rust
pub fn explode(&mut self, def: &ExplosionDef)

pub struct ExplosionDef {
    pub mask_bits: u64,          // 默认 u64::MAX
    pub position: Pos,           // 默认 ZERO
    pub radius: f32,             // 默认 0.0
    pub falloff: f32,            // 默认 0.0
    pub impulse_per_area: f32,   // 默认 0.0
}
```

## 快照

```rust
pub fn save_state(&self) -> WorldState
pub fn restore_state(&mut self, state: &WorldState) -> bool
pub fn WorldState::byte_count(&self) -> usize
pub fn WorldState::step_index       // 公开字段：保存时的步数
```

## 诊断

```rust
pub fn world_state_hash(&self) -> u64
pub fn body_state_dump(&self) -> Vec<String>
pub fn contact_dump(&self) -> Vec<String>

pub fn counters(&self) -> Counters
pub fn profile_data(&self) -> Profile

pub fn body_count(&self) -> i32
pub fn awake_body_count(&self) -> i32
pub fn contact_count(&self) -> i32
pub fn joint_count(&self) -> i32
pub fn touching_contact_count(&self) -> i32
pub fn hull_database_count(&self) -> usize
pub fn compound_shape_count_value(&self) -> i32
```

`Counters` 中被引擎填充的字段：

```rust
byte_count, body_count, shape_count, contact_count, joint_count,
island_count, static_tree_height, tree_height,
sat_call_count, sat_cache_hit_count, recycled_contact_count,
awake_contact_count, color_counts: [i32; 24]
```

`Counters` 里另有 `stack_used`、`arena_capacity`、`task_count`、`manifold_counts`、`distance_iterations`、`push_back_iterations`、`root_iterations` 字段存在，但 0.2.0 的 `counters()` 没有填充它们（保持默认 0）。

`Profile` 中被引擎填充的字段（毫秒）：

```rust
step, pairs, collide, prepare_constraints, store_impulses,
transforms, refit, bullets, soft_bodies, sleep_islands, vehicles, restitution
```

其余字段（`solve`、`solver_setup`、`constraints`、`integrate_velocities`、`warm_start`、`solve_impulses`、`integrate_positions`、`relax_impulses`、`split_islands`、`sensor_hits`、`joint_events`、`hit_events`、`sensors`）在本版本中保留为 0，用于阶段细分的未来扩展。

## 调试绘制

```rust
pub fn draw(&mut self, options: &DebugDraw, mask_bits: u64)   // 通常传 u64::MAX
```

`DebugDraw` 用回调交给你的渲染器，坐标全部是世界空间：

```rust
pub fn DebugDraw::new() -> Self    // 默认开启 shapes/joints/vehicles/soft_bodies

pub draw_shape:     Option<Arc<dyn Fn(*mut c_void, WorldTransform, HexColor)>>
pub draw_segment:   Option<Arc<dyn Fn(Pos, Pos, HexColor)>>
pub draw_transform: Option<Arc<dyn Fn(WorldTransform)>>
pub draw_point:     Option<Arc<dyn Fn(Pos, f32, HexColor)>>
pub draw_sphere:    Option<Arc<dyn Fn(Pos, f32, HexColor, f32)>>
pub draw_capsule:   Option<Arc<dyn Fn(Pos, Pos, f32, HexColor, f32)>>
pub draw_bounds:    Option<Arc<dyn Fn(AABB, HexColor)>>
pub draw_box:       Option<Arc<dyn Fn(Vec3, WorldTransform, HexColor)>>
pub draw_string:    Option<Arc<dyn Fn(Pos, &str, HexColor)>>

pub drawing_bounds: AABB
pub force_scale: f32
pub joint_scale: f32

// 开关
draw_shapes, draw_joints, draw_joint_extras, draw_bounds_option, draw_mass,
draw_sleep, draw_body_names, draw_contacts, draw_anchor_a, draw_graph_colors,
draw_contact_features, draw_contact_normals, draw_contact_forces, draw_islands,
draw_vehicles, draw_soft_bodies
```

## ID 类型

```rust
pub struct WorldID  { pub index1: u16, pub generation: u16 }
pub struct BodyID   { pub index1: i32, pub world0: u16, pub generation: u16 }
pub struct ShapeID  { pub index1: i32, pub world0: u16, pub generation: u16 }
pub struct JointID  { pub index1: i32, pub world0: u16, pub generation: u16 }
pub struct ContactID { pub index1: i32, pub world0: u16, pub padding: i16, pub generation: u32 }
```

共同 API：

```rust
pub const NULL: Self
pub fn is_null(&self) -> bool
pub fn stored(&self) -> u32        // WorldID
pub fn stored(&self) -> u64        // BodyID, ShapeID, JointID
pub fn stored(&self) -> (u32, u32, u32)   // ContactID
pub fn from_stored(...) -> Self
```

ID 是不透明句柄，`index1` 从 1 开始。**不要自己构造 ID**，只用 API 返回的值。

`SoftBodyID` 和 `VehicleID` 结构相同，但没有 `stored()` / `from_stored()`。

## 常用常量

```rust
use natsel::base::constants::{set_length_units_per_meter, length_units_per_meter, get_length_units_per_meter};
```

时间与容差由长度单位自动派生，一般不需要手调。可用 `length_units_per_meter()` 读取当前单位。

## 下一步

- [核心概念](/docs/concepts)：这些 API 背后的对象模型。
- [常见问题](/docs/faq)：调用返回 `None` / `false` / 静默无效时怎么排查。
