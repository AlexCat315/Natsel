# 软体

软体是由一组顶点、边、三角面与高阶级约束（弯曲、体积、长程附着）组成的可变形体。它使用 XPBD 求解：每个世界步内跑 `iteration_count` 个子迭代，每个子迭代都对所有约束做一次投影。

软体适合布料、绳索、气囊、果冻与可破坏的薄壳。它不参与刚体的约束求解，而是作为独立的记录在 `step` 中被推进，并与刚体形状做碰撞。

## 模型说明

软体的数据分成三层：

| 层次 | 类型 | 内容 |
|------|------|------|
| 配置 | `SoftBodySettings` | 静止配置下的顶点、面、边与约束 |
| 实例参数 | `SoftBodyDef` | 摆放位置、迭代次数、材质、碰撞开关 |
| 运行时 | 世界里的软体记录 | 当前位置、速度、包围盒、接触 |

约束有四类，都用 `compliance`（顺应性，即刚度的倒数，单位 m/N）描述软硬：

| 约束 | 类型 | 作用 |
|------|------|------|
| 距离 | `SoftBodyEdge` | 保持两个顶点在 `rest_length` 上 |
| 弯曲 | `SoftBodyDihedralBend` | 保持共享一条边的两个三角面之间的二面角 |
| 体积 | `SoftBodyVolume` | 保持四面体的有符号体积 |
| 长程附着 | `SoftBodyLRA` | 让顶点与某个运动学锚点保持不超过 `max_distance` |

`compliance = 0` 表示刚性，越大越软。所有约束的 `compliance` 取参与顶点的最大值。

顶点的 `inv_mass` 为 0 表示它是**运动学顶点**：不会被求解器移动，只能通过 `set_soft_body_vertex_position` 或蒙皮来驱动。`make_cloth` 的 `pinned` 闭包就是通过置零 `inv_mass` 来实现固定点的。

### SoftBodySettings 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `vertices` | `Vec<SoftBodyVertex>` | 静止配置的粒子 |
| `faces` | `Vec<SoftBodyFace>` | 三角面，从外侧看是逆时针 |
| `edges` | `Vec<SoftBodyEdge>` | 距离约束 |
| `dihedral_bends` | `Vec<SoftBodyDihedralBend>` | 弯曲约束 |
| `volumes` | `Vec<SoftBodyVolume>` | 体积约束 |
| `long_range_attachments` | `Vec<SoftBodyLRA>` | 长程附着 |
| `skinned` | `Vec<SoftBodySkinned>` | 绑定到骨骼的顶点 |
| `inverse_bind_matrices` | `Vec<Transform>` | 每个骨骼关节的逆绑定姿态 |
| `vertex_remap` | `Vec<u32>` | `optimize` 应用的置换（旧索引 → 新索引） |

`SoftBodyVertex { position, velocity, inv_mass }`，`SoftBodyVertex::new(position, velocity, inv_mass)`。默认 `inv_mass` 是 `1.0`。

`SoftBodyFace::new(v0, v1, v2)`，`is_degenerate()` 在任意两个索引相同时为真。

`SoftBodyEdge::new(v0, v1, compliance)`，`rest_length` 需要再调 `calculate_rest_values()` 才算出来。

`SoftBodyDihedralBend::new(v0, v1, v2, v3, compliance)` 约束边 `(v0, v1)` 两侧的三角面 `(v0, v1, v2)` 与 `(v1, v0, v3)`。

`SoftBodyVolume::new(v0, v1, v2, v3, compliance)` 的 `six_rest_volume` 是"六倍有符号体积"。

`SoftBodyLRA::new(anchor, vertex, max_distance)`。

### 构造流程

手工构造软体时，顺序是固定的：

1. 填 `vertices` 与 `faces`。
2. `create_constraints(&attributes, bend_type, angle_tolerance)` 由面自动生成边、剪切、弯曲与长程附着约束；`attributes` 可以只有一个元素（对所有顶点生效）或每顶点一个。
3. `calculate_rest_values()` 由顶点位置算出所有静止值。
4. `optimize()` 重排顶点与约束顺序以提升求解与碰撞效率。
5. `validate()` 检查索引与数值，返回 `Option<String>`（`None` 表示通过）。

`create_soft_body` 会先校验配置，不合法时返回空 ID 而不是 panic。

### 工厂函数

多数情况下不需要手工构造，用内置工厂更快：

| 函数 | 说明 |
|------|------|
| `make_cloth<F>(columns, rows, spacing, attributes, bend_type, pinned)` | 位于 xz 平面、以原点为中心的矩形布。`pinned(x, z) -> bool` 返回哪些网格坐标是运动学顶点 |
| `make_cube(count, spacing, compliance, volume_compliance)` | 每边 `count` 个顶点的四面体实心箱 |
| `make_sphere(radius, subdivisions, compliance, bend_compliance, bend_type)` | 闭合球面；配合 `pressure` 可做气球 |
| `make_rope(count, spacing, compliance, pinned)` | 从原点沿 -y 的顶点链；`pinned` 为真时首顶点运动学 |

`make_cloth` 的 `columns` 与 `rows` 都必须至少为 2，否则返回空配置。它会自动调用 `create_constraints` 与 `optimize`。

`SoftBodyVertexAttributes` 控制自动生成的约束：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `compliance` | `0.0` | 三角边约束的顺应性 |
| `shear_compliance` | `0.0` | 四边形对角线的顺应性；`f32::INFINITY` 表示不生成剪切约束 |
| `bend_compliance` | `f32::INFINITY` | 弯曲的顺应性；`f32::INFINITY` 表示不生成弯曲约束 |
| `lra_type` | `SoftBodyLRAType::None` | 长程附着类型 |
| `lra_max_distance_multiplier` | `1.0` | 缩放附着的静止距离，允许少量拉伸 |

`SoftBodyLRAType` 有 `None`、`Euclidean`（到最近运动学顶点的直线距离）、`Geodesic`（沿边的距离，适合折叠的布）。

`SoftBodyBendType` 有 `None`、`Distance`（相邻三角面远端顶点之间的距离约束）、`Dihedral`（相邻三角面之间的角度约束，能保持折痕形状）。

## 创建软体

```rust
pub fn create_soft_body(&mut self, def: &SoftBodyDef) -> SoftBodyID
```

`SoftBodyDef::with_settings(settings, position, rotation)` 保留其余默认值。

### SoftBodyDef 字段

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `settings` | `SoftBodySettings` | 空 | 静止配置 |
| `position` | `Pos` | 零 | 摆放位置（应用到静止顶点） |
| `rotation` | `Quat` | 单位 | 摆放旋转 |
| `iteration_count` | `i32` | `5` | 每个世界步的 XPBD 子迭代数；越大越硬 |
| `linear_damping` | `f32` | `0.1` | 线速度阻尼：`dv/dt = -c * v` |
| `max_linear_velocity` | `f32` | `500 * units` | 顶点速度上限 |
| `friction` | `f32` | `0.2` | 与刚体碰撞的摩擦 |
| `restitution` | `f32` | `0.0` | 弹性 |
| `pressure` | `f32` | `0.0` | 封闭曲面的 `n * R * T`（N·m）；正值使软体膨胀 |
| `gravity_scale` | `f32` | `1.0` | 相对世界重力的缩放 |
| `vertex_radius` | `f32` | `0.0` | 每个顶点的碰撞半径 |
| `filter` | `Filter` | 默认 | 与刚体形状的碰撞过滤 |
| `collide_with_rigid_bodies` | `bool` | `true` | 是否与刚体碰撞 |
| `two_way_coupling` | `bool` | `true` | 双向耦合；关掉后刚体只推软体、不感受反作用 |
| `enable_sleep` | `bool` | `true` | 是否允许睡眠 |
| `sleep_threshold` | `f32` | `0.03 * units` | 所有顶点低于此速度达 `time_to_sleep` 后入睡 |
| `faces_double_sided` | `bool` | `false` | 射线是否命中面的两侧 |
| `enable_self_collision` | `bool` | `false` | 软体自身表面是否互相碰撞（布折叠、绳自交） |
| `collide_with_soft_bodies` | `bool` | `true` | 是否与其他软体碰撞 |
| `enable_edge_collision` | `bool` | `true` | 顶点接触之外，是否启用边接触（细杆、刀刃、软体间的交叉） |
| `collision_thickness` | `f32` | `0.0` | 自碰撞与软-软接触保持的距离；`0` 表示自动（平均边长四分之一，或两倍顶点半径，取较大者） |
| `user_data` | `Option<*mut c_void>` | `None` | 用户指针 |

注意 `SoftBodyDef` 的过滤字段类型是 `Filter`（`category_bits` / `mask_bits` / `group_index`），不是 `QueryFilter`。

## 完整示例

```rust
use natsel::math::types::{Pos, Quat, Vec3};
use natsel::simulation::World;
use natsel::soft_body::soft_body_runtime::SoftBodyDef;
use natsel::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodyLRAType, SoftBodySettings, SoftBodyVertexAttributes,
};

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);

// 20x20 的布，左上与右上两个角固定
let mut attributes =
    SoftBodyVertexAttributes::new(0.0, 0.0, 1.0e-4, SoftBodyLRAType::Geodesic, 1.0);
let settings = SoftBodySettings::make_cloth(
    20,
    20,
    0.05,
    attributes,
    SoftBodyBendType::Dihedral,
    |x, z| z == 0 && (x == 0 || x == 19),
);

let mut def =
    SoftBodyDef::with_settings(settings.clone(), Pos::new(0.0, 2.0, 0.0), Quat::IDENTITY);
def.iteration_count = 10;

let cloth = world.create_soft_body(&def);
assert!(!cloth.is_null());
assert_eq!(world.soft_body_vertex_count(cloth), 400);

let dt: f32 = 1.0 / 60.0;
for _ in 0..240 {
    world.step(dt, 4);
}

// 读取顶点：Pos 是双精度的世界坐标
let positions = world.soft_body_vertex_positions(cloth);
let min_y = positions
    .iter()
    .fold(f64::MAX, |acc, p| if p.y < acc { p.y } else { acc });
println!("lowest vertex y = {min_y}");

// 固定点没有移动
for (i, v) in settings.vertices.iter().enumerate() {
    if v.inv_mass == 0.0 {
        assert!((positions[i].y - 2.0).abs() < 1.0e-5);
    }
}

println!("mass = {}", world.soft_body_mass(cloth));
// 注意：布料是开放曲面，没有体积。这里会得到一个接近 0（甚至为负）的数值，
// 不要用它判断布料状态。体积只对封闭曲面（如 make_cube）有意义。
```

`attributes` 需要写成 `mut` 再改字段是常见写法，也可以直接用 `SoftBodyVertexAttributes::new(compliance, shear_compliance, bend_compliance, lra_type, lra_max_distance_multiplier)` 一次给全。

如果不需要回读配置里的固定点信息，用 `world.soft_body_vertex_has_contact(cloth, vertex)` 之类按索引访问的接口即可，不必保存 `settings` 的克隆。

## 运行时 API

创建与生命周期：

| 方法 | 说明 |
|------|------|
| `create_soft_body(&def) -> SoftBodyID` | 配置非法时返回空 ID（`is_null()`） |
| `destroy_soft_body(id)` | 销毁 |
| `soft_body_is_valid(id) -> bool` | 有效性 |
| `soft_body_count() -> i32` | 存活数量 |
| `soft_body_user_data(id) -> Option<*mut c_void>` | 用户指针 |

顶点读取：

| 方法 | 返回 | 说明 |
|------|------|------|
| `soft_body_vertex_count(id)` | `i32` | 顶点数；无效 ID 返回 `0` |
| `soft_body_vertex_positions(id)` | `Vec<Pos>` | 全部世界坐标（分配） |
| `with_soft_body_vertex_positions(id, f)` | `Option<R>` | 通过回调交出 `&[Pos]`，仍然分配 |
| `with_soft_body_local_vertex_positions(id, f)` | `Option<R>` | 回调收到 `(origin, &[Vec3])`，**不分配**，适合每帧渲染 |
| `soft_body_vertex_velocities(id)` | `Vec<Vec3>` | 全部速度 |
| `soft_body_vertex_position(id, vertex)` | `Option<Pos>` | 单个顶点 |
| `soft_body_vertex_has_contact(id, vertex)` | `bool` | 上个时间步是否碰到刚体 |

每帧渲染时优先用 `with_soft_body_local_vertex_positions`，它在双精度里加 `origin`，既省一次分配又保精度：

```rust
world.with_soft_body_local_vertex_positions(cloth, |origin, local| {
    for p in local {
        // 世界坐标 = Pos::from_vec3(*p) + origin
        let world_x = origin.x + p.x as f64;
        let world_y = origin.y + p.y as f64;
        let world_z = origin.z + p.z as f64;
        let _ = (world_x, world_y, world_z);
    }
});
```

顶点写入：

| 方法 | 说明 |
|------|------|
| `set_soft_body_vertex_position(id, vertex, pos)` | 移动顶点；运动学顶点靠这个动画 |
| `set_soft_body_vertex_velocity(id, vertex, velocity)` | 设置单点速度 |
| `set_soft_body_vertex_inverse_mass(id, vertex, inv_mass)` | 置 0 固定该点，置正数释放 |
| `add_soft_body_velocity(id, velocity)` | 给所有动态顶点叠加速度 |
| `apply_soft_body_linear_impulse(id, impulse)` | 按质量分配冲量，使动量变化等于 `impulse` |
| `set_soft_body_transform(id, position, rotation)` | 绕质心整体刚体变换（瞬移） |

整体读取：

| 方法 | 返回 | 说明 |
|------|------|------|
| `soft_body_mass(id)` | `f32` | 动态顶点总质量 |
| `soft_body_center_of_mass(id)` | `Pos` | 质量加权质心；全运动学时退化为顶点平均 |
| `soft_body_linear_velocity(id)` | `Vec3` | 质量加权的平均速度 |
| `soft_body_bounds(id)` | `AABB` | 世界包围盒 |
| `soft_body_volume(id)` | `f32` | 封闭曲面在上个时间步的体积 |
| `soft_body_settings(id)` | `Option<SoftBodySettings>` | 配置的克隆（注意开销） |
| `soft_body_contacts(id)` | `Vec<SoftBodyContact>` | 上个时间步碰到的刚体形状 |

`SoftBodyContact { shape_id, body_id, vertex_count }`：`vertex_count` 是接触该形状的顶点数量。

调参：

| 方法 | 说明 |
|------|------|
| `set_soft_body_iteration_count(id, count)` | 子迭代数；传入小于 1 的值按 1 处理 |
| `set_soft_body_pressure(id, pressure)` / `soft_body_pressure(id)` | 读写压力 |
| `set_soft_body_friction(id, friction)` | 摩擦，负值被夹到 0 |
| `set_soft_body_restitution(id, restitution)` | 弹性，负值被夹到 0 |
| `set_soft_body_linear_damping(id, damping)` | 阻尼，负值被夹到 0 |
| `set_soft_body_self_collision(id, enabled)` | 自碰撞；开启时重建自接触排除表 |
| `set_soft_body_collide_with_soft_bodies(id, enabled)` | 与其他软体的碰撞 |
| `set_soft_body_edge_collision(id, enabled)` | 边接触 |
| `set_soft_body_collision_thickness(id, thickness)` / `soft_body_collision_thickness(id)` | 接触厚度；写入 0 恢复自动 |
| `set_soft_body_gravity_scale(id, scale)` | 重力缩放 |
| `set_soft_body_filter(id, filter)` | 碰撞过滤 |
| `soft_body_is_awake(id)` / `set_soft_body_awake(id, awake)` | 睡眠控制 |

蒙皮：

| 方法 | 说明 |
|------|------|
| `set_soft_body_skin_transforms(id, &[WorldTransform], teleport)` | 驱动蒙皮顶点的骨骼姿态；`teleport` 为真时运动学顶点直接吸附到蒙皮位置并清零速度 |
| `set_soft_body_skin_transforms_from_transforms(id, ..., teleport)` | 同上，但接收 `&[Transform]`（单精度） |

`transforms` 的长度必须不少于 `inverse_bind_matrices.len()`，否则调用被忽略。

## 压力与布料技巧

`pressure` 是 `n * R * T`（摩尔数 × 气体常数 × 温度），只对**封闭曲面**有意义。用它可以把一个 `make_sphere` 变成气球：

```rust
let settings = SoftBodySettings::make_sphere(
    0.5,
    2,
    1.0e-4,
    1.0e-4,
    SoftBodyBendType::Dihedral,
);
let mut def = SoftBodyDef::with_settings(settings, Pos::new(0.0, 2.0, 0.0), Quat::IDENTITY);
def.pressure = 300.0; // 正值向外膨胀
def.vertex_radius = 0.01;
let balloon = world.create_soft_body(&def);
```

`soft_body_volume` 可以用来监视是否"漏气"：封闭曲面在压力作用下体积应当稳定在一个大于零的值。

布料的关键参数是 `iteration_count` 与 `lra_type`。长程附着让长布条在不增加迭代次数的情况下也几乎不可拉伸——这正是 `make_cloth` 配合 `SoftBodyLRAType::Geodesic` 的用法。

## 注意事项

- **`soft_body_*` 前缀不是装饰**：所有软体接口都带前缀（`create_soft_body`、`soft_body_vertex_positions`、`set_soft_body_pressure`……），因为它们在 `World` 上与刚体方法共存。写错前缀会编译失败而不是静默失效。
- **`SoftBodyDef` 用 `Filter` 而不是 `QueryFilter`**：`filter` 字段的类型是 `Filter`（只有 `category_bits` / `mask_bits` / `group_index`），而角色与车辆的过滤用的是 `QueryFilter`。
- **构造失败返回空 ID**：`create_soft_body` 在 `validate()` 失败、位置非法或旋转非法时返回 `SoftBodyID::default()`。检查 `is_null()` 而不是假设成功。
- **`iteration_count` 是刚度的主要旋钮**：XPBD 下刚度与迭代次数耦合。同样的 `compliance`，迭代次数越多越硬。别只调 `compliance`。
- **`compliance` 是顺应性不是刚度**：数值越大越软，`0` 为刚性。弯曲与剪切的默认值是 `f32::INFINITY`，意味着默认**不生成**这类约束——需要弯曲刚度就必须显式给有限的 `bend_compliance`。
- **`make_cloth` 的 `pinned` 是网格坐标**：参数是 `(x, z)` 的网格索引，不是世界坐标或顶点索引。
- **`optimize()` 会重排顶点**：它按空间填充曲线排序顶点并记录 `vertex_remap`。工厂函数已经调用过它，所以 `settings.vertices` 的顺序是重排后的顺序——拿它和运行时顶点索引对齐时不要用原始构造顺序。
- **`two_way_coupling` 默认开启**：软体也会推动刚体。只想让刚体单方面推软体时把它设为 `false`。
- **`soft_body_settings()` 是深拷贝**：每帧调用会带来可观开销，缓存结果。
- **蒙皮长度必须够**：`set_soft_body_skin_transforms` 的 `transforms` 短于 `inverse_bind_matrices` 时直接返回，不报错。
- **`enable_edge_collision` 默认开启**：它带来额外的边-特征与边-边接触，对薄布料与绳索必要，但对体积型软体是纯开销。

## 相关页面

- [刚体](/docs/bodies)：软体与刚体的双向耦合通过刚体形状完成。
- [形状](/docs/shapes)：软体顶点接触的对象是刚体形状。
- [材质与过滤](/docs/materials)：`Filter` 与 `QueryFilter` 的区别在这一页。
- [骨骼与布娃娃](/docs/skeleton)：蒙皮软体由骨骼姿态驱动。
- [浮力与爆炸](/docs/buoyancy)：软体不参与爆炸冲量。
- [调试绘制](/docs/debug-draw)：`draw_soft_bodies` 可画出软体顶点与面。
