# 形状

形状（Shape）是挂在刚体上的碰撞几何。刚体本身只有运动状态，是形状赋予它体积、质量、摩擦、过滤规则和事件能力。

一个刚体可以有任意多个形状。所有形状共享刚体的变换，但各自拥有独立的局部几何、密度、材质与过滤设置。

本页讲**如何选用形状并配置 `ShapeDef`**。几何本身怎么生成（凸包顶点、网格三角形、高度场数据）见 [几何构建器](/docs/geometry)；摩擦、弹性、碰撞过滤的物理行为见 [材质与过滤](/docs/materials)。

## 六种形状类型

`ShapeType` 是一个运行时枚举，可以用 `world.shape_type(shape_id)` 查出来：

| `ShapeType` | 创建方法 | 几何参数 | 质量/惯性 | 能否放在静态体以外 |
|-------------|----------|----------|-----------|-------------------|
| `SphereShape` | `create_sphere_shape` | `Sphere { center, radius }` | 解析解 | 可以 |
| `CapsuleShape` | `create_capsule_shape` | `Capsule { center1, center2, radius }` | 解析解 | 可以 |
| `HullShape` | `create_hull_shape` | `&Hull` | 从凸包体积积分 | 可以 |
| `MeshShape` | `create_mesh_shape` | `TriangleMesh` + `Vec3` 缩放 | **不计质量** | **不可以，仅静态体** |
| `HeightShape` | `create_height_field_shape` | `HeightField` | **不计质量** | **不可以，仅静态体** |
| `CompoundShape` | `create_compound_shape` | `&Compound` | **不计质量** | **不可以，仅静态体** |

"仅静态体"是硬性限制：**高度场、复合形状、网格**这三种都不能挂在动态或运动学刚体上，调用会返回空 `ShapeID`（全零），形状不会被创建。

这条规则存在的原因就是下面的质量问题：这三种形状不计质量。如果一个动态体只挂它们，`inv_mass` 会是 0，物体永远不动 —— 而且看起来一切正常。引擎选择直接拒绝，而不是让你得到一个静默失效的刚体。

动态体需要复合几何时，用 `create_dynamic_compound`（它把复合拆成球/胶囊/凸包子形状，每个子形状正常计质量），或者手动创建多个独立形状。

**质量只来自球、胶囊、凸包。** 这三种形状把密度积分成质量；网格、高度场、复合形状不参与质量计算。因此动态物体必须至少挂一个球/胶囊/凸包。若确实想让某个形状不贡献质量（例如纯触发区域），把它的 `density` 设为 `0.0`。

`create_*_shape` 全部返回 `ShapeID`。空 ID 表示创建失败 —— 常见原因是刚体 ID 无效、或者把静态体专属形状挂到了动态体上。判断方法：

```rust
let shape = world.create_sphere_shape(body, &ShapeDef::default(), sphere);
if shape.is_null() {
    // 创建失败
}
```

### 球体

```rust
use natsel::math::types::Vec3;
use natsel::types::geometry::Sphere;

let sphere = Sphere::new(
    Vec3::new(0.0, 0.5, 0.0),   // 相对刚体原点的局部圆心
    0.5,                        // 半径
);
let shape = world.create_sphere_shape(body, &ShapeDef::default(), sphere);
```

`center` 是**局部坐标**，允许把球心偏离刚体原点。质量按 `4/3 * π * r³ * density` 计算，惯量是各向同性的。

### 胶囊

胶囊是两个半球加一个圆柱：

```rust
use natsel::types::geometry::Capsule;

let capsule = Capsule::new(
    Vec3::new(0.0, -0.5, 0.0),   // 内部线段起点
    Vec3::new(0.0, 0.5, 0.0),    // 内部线段终点
    0.25,                        // 半径
);
let shape = world.create_capsule_shape(body, &ShapeDef::default(), capsule);
```

`center1` 与 `center2` 定义中轴线段，半径是外部半径。胶囊总长是 `|center2 - center1| + 2 * radius`。

**陷阱：过短的胶囊会变成球。** 当 `|center2 - center1|` 小于引擎的最小胶囊长度时，`create_capsule_shape` 会静默地创建一个球体（球心取线段中点，半径不变），返回的 shape 类型是 `SphereShape` 而不是 `CapsuleShape`。所以不要用胶囊做"极扁的柱子"，那种情况直接用球或凸包。

```rust
// 会变成球
let flat = Capsule::new(Vec3::ZERO, Vec3::new(0.0, 1e-9, 0.0), 0.5);
let s = world.create_capsule_shape(body, &ShapeDef::default(), flat);
assert_eq!(world.shape_type(s), Some(ShapeType::SphereShape));
```

胶囊是角色胶囊体的标准选择：两段半球的形状让它在楼梯和斜坡上滑动比胶囊化的圆柱自然得多。

### 凸包

```rust
use natsel::collision::hull::make_box_hull;

let hull = make_box_hull(0.5, 0.5, 0.5);   // 半长 (hx, hy, hz)
let shape = world.create_hull_shape(body, &ShapeDef::default(), &hull);
```

`create_hull_shape` 接收引用并内部克隆。凸包是**内容共享**的：两个内容完全相同的凸包在引擎里只存一份，第二个形状只是增加引用计数。所以用 `make_box_hull` 批量造 1000 个方块，内存里只有一份盒子几何。

注意参数是半长（half extent），不是全尺寸。`make_box_hull(0.5, 0.5, 0.5)` 得到的是边长 1 的立方体。

### 三角网格

```rust
use natsel::collision::mesh::procedural::create_wave_mesh;
use natsel::math::types::Vec3;

let mesh = create_wave_mesh(32, 32, 1.0, 0.5, 2.0, 2.0).expect("wave mesh");
let shape = world.create_mesh_shape(ground, &ShapeDef::default(), mesh, Vec3::ONE);
```

网格适合静态的复杂地形与建筑。第三个参数是 `TriangleMesh`（按值传入，内部是 `Arc` 共享），最后一个是**非均匀缩放**，没有缩放需求就传 `Vec3::ONE`。

网格可以挂在动态体上，但有两个限制：网格不计质量（见上表），而且每个三角形对都要做窄相检测，代价明显。动态凸物体用凸包，动态凹物体用多个凸包拼。

### 高度场

```rust
use natsel::collision::height_field::factories::create_grid_height_field;

let field = create_grid_height_field(64, 64, Vec3::new(100.0, 1.0, 100.0), false)
    .expect("height field");
let shape = world.create_height_field_shape(ground, &ShapeDef::default(), field);
```

高度场是以规则网格采样高度值的地形，专为大范围地面设计：内存是 `rows * cols` 个 f32，而不是三角形列表。它**只能挂在静态刚体上**。

### 复合形状

复合形状把多个球、胶囊、凸包、网格打包成一个不可分割的几何体：

```rust
use natsel::collision::compound::compound::Compound;
use natsel::types::geometry::{CompoundDef, CompoundHullDef};
use natsel::math::types::Transform;

let mut cdef = CompoundDef::default();
let mut child = CompoundHullDef::default();
child.hull = Some(make_box_hull(1.0, 1.0, 1.0));
child.transform = Transform::IDENTITY;
cdef.hulls.push(child);

let compound = Compound::new(&cdef).expect("compound");
let shape = world.create_compound_shape(ground, &ShapeDef::default(), &compound);
```

复合形状在创建时会把子形状烘焙进一块连续内存，运行时不再解引用指针，因此比同样几何的一组独立形状更快、缓存更友好。代价是不可修改，也不能挂在动态体上。适合复杂静态场景（建筑、雕塑、赛道）。

构造细节见 [几何构建器](/docs/geometry)。

## 六个创建方法的签名

全部挂在 `World` 上：

| 方法 | 签名 |
|------|------|
| `create_sphere_shape` | `(&mut self, body: BodyID, def: &ShapeDef, sphere: Sphere) -> ShapeID` |
| `create_capsule_shape` | `(&mut self, body: BodyID, def: &ShapeDef, capsule: Capsule) -> ShapeID` |
| `create_hull_shape` | `(&mut self, body: BodyID, def: &ShapeDef, hull: &Hull) -> ShapeID` |
| `create_mesh_shape` | `(&mut self, body: BodyID, def: &ShapeDef, data: TriangleMesh, scale: Vec3) -> ShapeID` |
| `create_height_field_shape` | `(&mut self, body: BodyID, def: &ShapeDef, field: HeightField) -> ShapeID` |
| `create_compound_shape` | `(&mut self, body: BodyID, def: &ShapeDef, compound: &Compound) -> ShapeID` |

共同点：都借用 `&ShapeDef`（创建后 `ShapeDef` 可以丢弃），都返回 `ShapeID`，失败时返回空 ID。

## ShapeDef 字段参考

`ShapeDef` 里的每个字段都在创建时被复制进内部的形状记录。默认值取自 `impl Default for ShapeDef`：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `name` | `Option<String>` | `None` | 调试名，存入名字缓存 |
| `user_data` | `Option<*mut c_void>` | `None` | 透传给事件的裸指针 |
| `materials` | `Vec<SurfaceMaterial>` | 空 | 逐三角形材质（网格/高度场用）。空表示用 `base_material` |
| `base_material` | `SurfaceMaterial` | 摩擦 0.6 | 基础材质 |
| `density` | `f32` | `1000.0 / (单位³)` | 密度，kg/m³ 量级 |
| `explosion_scale` | `f32` | `1.0` | 爆炸冲量的缩放倍率 |
| `filter` | `Filter` | 全碰撞 | 类别位、掩码位、组索引 |
| `enable_custom_filtering` | `bool` | `false` | 是否调用自定义过滤回调 |
| `is_sensor` | `bool` | `false` | 传感器：只报告重叠，不产生物理响应 |
| `enable_sensor_events` | `bool` | `false` | 生成传感器进入/离开事件 |
| `enable_contact_events` | `bool` | `false` | 生成接触开始/结束事件 |
| `enable_hit_events` | `bool` | `false` | 生成高速撞击事件 |
| `enable_pre_solve_events` | `bool` | `false` | 每步调用 pre-solve 回调 |
| `invoke_contact_creation` | `bool` | `true` | 创建时立即为这个形状生成接触对 |
| `update_body_mass` | `bool` | `true` | 创建后重算刚体质量 |
| `enable_speculative_contact` | `bool` | `true` | 允许预测性接触，减少高速穿透 |

`density` 的默认值随长度单位缩放，默认单位下就是 1000。改长度单位后它会相应变化，见 [单位](/docs/units)。

## 密度与质量

`density` 是形状唯一的"重量"来源。刚体的总质量是所有形状质量之和，惯性张量也是累加的。

实心球的质量是 `4/3 * π * r³ * density`。半径 0.5、密度 1000 的球：

```rust
let mut sd = ShapeDef::default();
sd.density = 1000.0;
let shape = world.create_sphere_shape(body, &sd, Sphere::new(Vec3::ZERO, 0.5));

// 4/3 * π * 0.125 * 1000 ≈ 523.6
println!("mass = {:.1}", world.body_mass(body));
```

常见密度参考：

| 材质 | 密度 (kg/m³) |
|------|-------------|
| 木头 | 500 ~ 800 |
| 水 | 1000 |
| 混凝土 | 2200 ~ 2400 |
| 钢 | 7800 |

不过物理表现对不对，更取决于**相对**密度和重力、力的大小是否匹配。真实密度配上真实重力（-9.81）通常能得到手感自然的场景；但如果你的世界尺度不是米，就需要相应调整。

`density = 0` 会让形状不贡献质量。全为 0 的动态刚体 `inv_mass` 也是 0，行为像一个静态体但类型仍是动态 —— 这不是好状态，避免这么做。

运行时修改密度：

```rust
world.set_shape_density(shape, 500.0, true);   // 第三个参数：是否立即重算刚体质量
let d = world.shape_density(shape);
```

传 `false` 会把质量标记为待更新，下一次 `step` 时重算；传 `true` 立即算。

## 传感器

传感器只检测重叠，不产生任何推挤。常用作触发器、计分区域、AI 感知范围：

```rust
let mut sd = ShapeDef::default();
sd.is_sensor = true;
sd.enable_sensor_events = true;
let trigger = world.create_sphere_shape(zone_body, &sd, Sphere::new(Vec3::ZERO, 3.0));
```

要点：

- 传感器形状**仍按密度贡献质量**。想做"纯触发区域"而不影响刚体质量，就把它的 `density` 设为 `0.0`；默认密度（1000）下它会和普通形状一样计入质量。
- 传感器与其他形状之间不生成接触约束，因此不会产生任何推挤力 —— 物体可以自由穿过。
- `enable_sensor_events` 必须同时打开，否则 `is_sensor` 只意味着"不产生响应"，你收不到任何通知。
- 传感器事件在 `step` 之后读取，下一次 `step` 会清空。事件结构见 [事件](/docs/events)。

传感器与静态体搭配最省：静态体本身不动，传感器也不需要参与求解，只是纯查询。

## 事件开关

四个 `enable_*_events` 开关彼此独立，都需要显式打开：

| 开关 | 产生什么 | 何时用 |
|------|----------|--------|
| `enable_contact_events` | 接触开始/结束 | 脚步声、落地音效、接触点特效 |
| `enable_hit_events` | 高速撞击（带冲量） | 撞击伤害、破坏判定 |
| `enable_sensor_events` | 传感器进入/离开 | 触发区域 |
| `enable_pre_solve_events` | 每步接触前回调 | 单向平台、可穿透地板、动态摩擦调整 |

**只对需要的一方打开。** 事件生成不是零成本：每次接触创建都要填充事件缓冲，而且是逐对填充的。一个 500 个方块的场景如果全部打开接触事件，开销会明显上升。

运行时切换：

```rust
world.enable_shape_contact_events(shape, true);
world.enable_shape_hit_events(shape, true);
world.enable_shape_sensor_events(shape, true);
world.enable_shape_pre_solve_events(shape, true);
```

`enable_hit_events` 只在相对速度超过世界阈值时产生事件，阈值是 `world.hit_event_threshold`（默认 1.0）。调低会得到更多（也包括更多琐碎的）撞击事件。

`enable_pre_solve_events` 需要配合 `world.set_pre_solve_callback(...)` 才有意义 —— 回调返回 `false` 可以禁用该接触点当步的响应，这正是"从下往上穿过单向平台，从上往下站住"的实现方式。详见 [材质与过滤](/docs/materials)。

## 预测性接触

`enable_speculative_contact`（默认 `true`）让引擎在物体还没真正接触时就生成预测性接触约束，提前阻止穿透。对快速移动但不想开全量 CCD 的物体，这是便宜的替代方案。

需要两方形状都开启才会生效。世界级开关 `world.enable_speculative`（默认 `true`）是总闸，关掉它会让所有形状的预测性接触失效。

关掉它（`false`）会减少一些接触计算，但高速物体更容易穿模。除非你在做性能极紧的场景，否则保持默认。

## 形状查询

```rust
let t = world.shape_type(shape);              // Option<ShapeType>
let b = world.shape_body(shape);              // BodyID（无效 ID 返回空 ID）
let d = world.shape_density(shape);           // f32
let f = world.shape_filter(shape);            // Filter
let aabb = world.shape_aabb(shape);           // AABB（世界空间）
let m = world.shape_surface_material(shape);  // SurfaceMaterial
let n = world.body_shape_count(body);         // i32
```

销毁形状没有独立的公开方法；形状随刚体一起销毁（`destroy_body`）。需要"去掉一个形状"时，常见做法是禁用整个刚体（`disable_body`），或者重建刚体。

## 完整示例：多种形状同场

下面把所有形状类型放进一个场景，并打印每个形状的运行时类型与质量贡献：

```rust
use natsel::collision::compound::compound::Compound;
use natsel::collision::height_field::factories::create_wave_height_field;
use natsel::collision::hull::make_box_hull;
use natsel::collision::mesh::procedural::create_wave_mesh;
use natsel::math::types::{Pos, Transform, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::{Capsule, CompoundDef, CompoundHullDef, Sphere};
use natsel::types::shape_def::{ShapeDef, ShapeType};

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    // 静态地形：高度场
    let terrain_body = world.create_body(&BodyDef::default());
    let field = create_wave_height_field(64, 64, Vec3::new(100.0, 6.0, 100.0), 2.0, 2.0, false)
        .expect("height field");
    let terrain = world.create_height_field_shape(terrain_body, &ShapeDef::default(), field);
    assert_eq!(world.shape_type(terrain), Some(ShapeType::HeightShape));

    // 静态网格
    let mesh_body = world.create_body(&BodyDef::default());
    let mesh = create_wave_mesh(16, 16, 1.0, 0.3, 1.0, 1.0).expect("mesh");
    let mesh_shape = world.create_mesh_shape(mesh_body, &ShapeDef::default(), mesh, Vec3::ONE);
    assert_eq!(world.shape_type(mesh_shape), Some(ShapeType::MeshShape));

    // 静态复合体
    let compound_body = world.create_body(&BodyDef::default());
    let mut cdef = CompoundDef::default();
    let mut child = CompoundHullDef::default();
    child.hull = Some(make_box_hull(1.0, 0.5, 1.0));
    child.transform = Transform::IDENTITY;
    cdef.hulls.push(child);
    let compound = Compound::new(&cdef).expect("compound");
    let compound_shape =
        world.create_compound_shape(compound_body, &ShapeDef::default(), &compound);
    assert_eq!(world.shape_type(compound_shape), Some(ShapeType::CompoundShape));

    // 动态：球 + 胶囊 + 凸包，三个形状共用一个刚体
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 6.0, 0.0);
    let body = world.create_body(&def);

    let mut sd = ShapeDef::default();
    sd.density = 600.0;

    let s1 = world.create_sphere_shape(body, &sd, Sphere::new(Vec3::new(0.6, 0.0, 0.0), 0.3));
    let s2 = world.create_capsule_shape(
        body,
        &sd,
        Capsule::new(Vec3::new(-0.6, -0.3, 0.0), Vec3::new(-0.6, 0.3, 0.0), 0.3),
    );
    let s3 = world.create_hull_shape(body, &sd, &make_box_hull(0.3, 0.3, 0.3));

    println!(
        "types: {:?} {:?} {:?}",
        world.shape_type(s1),
        world.shape_type(s2),
        world.shape_type(s3)
    );
    println!("shapes on body = {}", world.body_shape_count(body));
    println!("total mass = {:.1}", world.body_mass(body));

    // 传感器区域
    let mut sensor_def = ShapeDef::default();
    sensor_def.is_sensor = true;
    sensor_def.enable_sensor_events = true;
    let _ = world.create_sphere_shape(
        terrain_body,
        &sensor_def,
        Sphere::new(Vec3::new(0.0, 3.0, 0.0), 5.0),
    );

    for _ in 0..240 {
        world.step(1.0f32 / 60.0, 4);
    }
    println!("settled at {:?}", world.body_position(body));
}
```

## 常见错误

**参数是半长而不是全尺寸。** `make_box_hull(10.0, 0.5, 10.0)` 得到的是 20×1×20 的地面。写成 `(20.0, 1.0, 20.0)` 会得到两倍大的地面 —— 通常只是看起来"地面比预想的大"，不容易发现。

**试图在动态刚体上挂网格/高度场/复合形状。** 这三种形状不计质量，只有静态体能挂。调用会返回空 `ShapeID`（用 `world.shape_is_valid(id)` 可判断），形状不会被创建。动态体需要复合几何时用 `create_dynamic_compound`。

**以为传感器不影响质量。** 传感器仍按 `density` 贡献质量。想让触发区域"无重量"，把 `density` 设为 `0.0`。

**在动态刚体上创建高度场或复合形状。** 返回空 `ShapeID`，没有断言也没有错误日志。检查返回值，或者干脆只在静态体上用这两种形状。

**忘了 `enable_*_events`。** `is_sensor = true` 只让形状不产生物理响应，但不会自动生成事件。要收到传感器事件必须同时设 `enable_sensor_events = true`。

**以为 `density` 直接设质量。** 密度是单位体积质量。同一密度下，半径翻倍质量变 8 倍。想让几十个物体质量一致，要么统一尺寸，要么用 `set_body_mass_data` 显式指定。

**用极扁的胶囊。** 会静默退化成球体。用 `world.shape_type(...)` 确认一下实际类型。

**给每个形状都开事件。** 事件开销逐对累加。只给需要响应的一方打开。

## 下一步

- [几何构建器](/docs/geometry)：凸包、网格、高度场、复合体怎么生成。
- [材质与过滤](/docs/materials)：摩擦、弹性、逐三角形材质、碰撞过滤。
- [刚体](/docs/bodies)：形状如何影响质量属性与睡眠。
- [事件](/docs/events)：接触、命中、传感器事件的完整流程。
- [空间查询](/docs/queries)：如何查询某个位置的形状。
