# 浮力与爆炸

这一页覆盖两个由**你主动调用**的力场功能：

- **浮力**：计算刚体在液面以下的排水体积，施加阿基米德浮力与流体阻力。
- **爆炸**：对球形范围内的动态刚体施加沿径向的冲量。

两者都**不是**引擎自动处理的。`world.step` 完全不涉及浮力与爆炸：如果你从不调用这些函数，刚体就只是普通刚体，水里也不会浮起来。这大概是最容易搞错的一点。

## 浮力模型说明

浮力被拆成两个可独立使用的层次：

| 层次 | 方法 | 用途 |
|------|------|------|
| 只测量 | `body_submerged_volume` | 查询排水体积与浮心，不改变任何状态 |
| 测量并施力 | `apply_buoyancy_impulse` | 对单个刚体施加一个时间步的浮力与阻力 |

液面由一个平面描述：`dot(surface_normal, x) = dot(surface_normal, surface_position)`。

**法线指向流体外部**。这是最关键的方向约定：水面在 `y = 0`、水在下方时，法线是 `(0, 1, 0)`。法线反了会让刚体被"往下推"。

排水体积对球、凸包与闭合网格是精确计算的（按液面裁剪），胶囊用数值积分近似。开放表面（高度场）没有体积，不参与浮力。

### body_submerged_volume

```rust
pub fn body_submerged_volume(
    &self,
    body_id: BodyID,
    surface_position: Pos,
    surface_normal: Vec3,
) -> SubmergedVolume
```

返回 `SubmergedVolume`：

| 字段 | 类型 | 说明 |
|------|------|------|
| `total_volume` | `f32` | 该刚体所有（非传感器）形状的总体积 |
| `submerged_volume` | `f32` | 液面以下的体积 |
| `center_of_buoyancy` | `Pos` | 排水体积的形心，世界坐标 |

刚体无效或 `surface_normal` 长度为零时返回全零结果（`center_of_buoyancy` 为零位置）。完全没有浸没时 `submerged_volume` 为 0。

这个函数只读，可以在任意时刻调用，用来做浮力指示、吃水深度显示、或者自己实现一套浮力。

### apply_buoyancy_impulse

```rust
pub fn apply_buoyancy_impulse(
    &mut self,
    body_id: BodyID,
    surface_position: Pos,
    surface_normal: Vec3,
    buoyancy: f32,
    linear_drag: f32,
    angular_drag: f32,
    fluid_velocity: Vec3,
    time_step: f32,
) -> bool
```

参数含义：

| 参数 | 说明 |
|------|------|
| `surface_position` / `surface_normal` | 液面；法线指向流体外部 |
| `buoyancy` | 流体密度相对于刚体平均密度的倍数。`1` 为中性浮力，大于 `1` 会浮起来 |
| `linear_drag` | 线性阻力系数，球体大约 `0.5` |
| `angular_drag` | 完全浸没时每秒移除的角速度比例 |
| `fluid_velocity` | 流体速度，用于水流 |
| `time_step` | 本次施力对应的时间步，应当与随后的 `step` 一致 |

返回值：**`false` 表示这个刚体当前没有浸在流体中**（或者刚体无效、非动态、已被禁用、时间步非正）。返回 `true` 表示这次施力生效。用返回值可以统计"当前有多少物体在水里"。

### 这是每步都要自己调用的

浮力不会累积也不会自动发生。正确用法是在每个 `step` 之前（或之后）对每个相关刚体调用一次：

```rust
world.apply_buoyancy_impulse(body, water_level, Vec3::AXIS_Y, 1.5, 0.5, 0.01, Vec3::ZERO, dt);
world.step(dt, 4);
```

`buoyancy` 的物理意义值得记一下：它是"流体密度 / 刚体平均密度"。所以 `buoyancy = 2.0` 的盒子最终会浮出约一半体积——稳态时浸没体积占比约等于 `1 / buoyancy`。

### 批量施力

遍历每个刚体比较麻烦，可以用体积版本：

```rust
pub fn apply_buoyancy(
    &mut self,
    volume: WorldAABB,
    surface_normal: Vec3,
    buoyancy: f32,
    linear_drag: f32,
    angular_drag: f32,
    fluid_velocity: Vec3,
    time_step: f32,
    filter: QueryFilter,
) -> i32
```

液面取这个盒子的**顶面**（沿 `surface_normal` 最高的一角），对所有与该盒子重叠的动态刚体施加浮力，返回受影响的刚体数量。

还有一个便利版本接收单精度包围盒：

```rust
pub fn apply_buoyancy_in_aabb(
    &mut self,
    volume: AABB,           // 单精度包围盒
    surface_normal: Vec3,
    buoyancy: f32,
    linear_drag: f32,
    angular_drag: f32,
    fluid_velocity: Vec3,
    time_step: f32,
    filter: QueryFilter,
) -> i32
```

`WorldAABB`（位于 `natsel::math::world_aabb`）是双精度包围盒（`lower_bound: Pos`、`upper_bound: Pos`），`AABB`（位于 `natsel::math::types`）是单精度（`Vec3`）。两者不能混用，`apply_buoyancy_in_aabb` 就是为单精度场景准备的。

## 浮力完整示例

一块木头浮在池子里，用每步手动调用的方式驱动：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::math::world_aabb::WorldAABB;
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::{QueryFilter, ShapeDef};

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);

// 水面在 y = 0，水在下方，所以法线朝上
let water_level = Pos::new(0.0, 0.0, 0.0);
let water_normal = Vec3::AXIS_Y;
let dt: f32 = 1.0 / 60.0;

// 一块密度只有水一半的木头
let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;
def.position = Pos::new(0.0, 2.0, 0.0);
let wood = world.create_body(&def);

let mut shape_def = ShapeDef::default();
shape_def.density = 500.0; // 水是 1000，所以 buoyancy 用 2.0
shape_def.base_material.friction = 0.4;
let _ = world.create_hull_shape(wood, &shape_def, &make_box_hull(0.5, 0.25, 0.5));

for _ in 0..600 {
    // 必须每步自己调用；返回值表示这一刻是否浸在水里
    let in_water = world.apply_buoyancy_impulse(
        wood,
        water_level,
        water_normal,
        2.0,          // buoyancy：水密度 / 木头密度
        0.5,          // linear_drag
        0.01,         // angular_drag
        Vec3::ZERO,   // fluid_velocity：静水
        dt,
    );

    if !in_water {
        // 还在空中，浮力为零，这一步只有重力
    }

    world.step(dt, 4);
}

// 稳态：浸没体积占比趋近 1 / buoyancy = 0.5。
// 600 步后实测约 0.46，跑到 3000 步收敛到约 0.48——浮动体在阻尼振荡，
// 不会精确等于 0.5。用这个比值判断浮力是否合理，不要要求精确相等。
let v = world.body_submerged_volume(wood, water_level, water_normal);
println!(
    "draft = {:.3} (expect ~0.5), volume = {}",
    v.submerged_volume / v.total_volume,
    v.total_volume
);
```

池子里有很多物体时，用批量版本省掉自己写循环：

```rust
let mut filter = QueryFilter::default_filter();
filter.category_bits = u64::MAX;

// 一个覆盖 20x20 水面、向下 10 米的流体盒子
let pool = WorldAABB::new(
    Pos::new(-10.0, -10.0, -10.0),
    Pos::new(10.0, 0.0, 10.0),  // 顶面 y = 0 就是水面
);

for _ in 0..600 {
    let affected = world.apply_buoyancy(
        pool,
        Vec3::AXIS_Y,
        1.2,
        0.5,
        0.01,
        Vec3::ZERO,
        dt,
        filter.clone(),
    );
    let _ = affected;
    world.step(dt, 4);
}
```

注意批量版本的液面取自盒子的顶面角点，所以盒子的高度范围只是用来筛选"谁可能在水里"，真正的液面位置由 `surface_normal` 方向上最高的那个角决定。

## 爆炸

```rust
pub fn explode(&mut self, def: &ExplosionDef)
```

`ExplosionDef`：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mask_bits` | `u64` | 全部位（`u64::MAX`） | 只影响匹配的形状 |
| `position` | `Pos` | 零 | 爆炸中心 |
| `radius` | `f32` | `0.0` | 完整冲量半径 |
| `falloff` | `f32` | `0.0` | 从 `radius` 到 `radius + falloff` 的衰减区间 |
| `impulse_per_area` | `f32` | `0.0` | 单位面积冲量 |

行为说明：

- 冲量**只施加给动态刚体**。静态与运动学刚体不受影响。
- 冲量大小与形状沿冲击方向的**投影面积**成正比，所以大平面比小方块受到更大的推力。
- 每个形状还有一个 `explosion_scale` 字段（在 `ShapeDef` 里），可以按形状缩放爆炸效果，设为 `0` 表示该形状完全免疫。
- 在 `radius` 以内是完整冲量；`radius` 到 `radius + falloff` 之间按距离线性衰减；再远没有效果。
- 命中的刚体会被唤醒。
- 冲量作用在形状上离爆炸中心最近的点，因此也会产生力矩。

```rust
use natsel::math::types::Pos;
use natsel::types::world_def::ExplosionDef;

let mut def = ExplosionDef::default();
def.position = Pos::new(0.0, 1.0, 0.0);
def.radius = 3.0;
def.falloff = 2.0;
def.impulse_per_area = 50.0;
world.explode(&def);
```

要让某类物体免疫爆炸，就用 `mask_bits` 收窄范围：

```rust
def.mask_bits = 1 << 1; // 只影响 category_bits 第 1 位匹配的形状
```

## 爆炸完整示例

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Quat, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;
use natsel::types::world_def::ExplosionDef;

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);

// 地面
let ground = world.create_body(&BodyDef::default());
let _ = world.create_hull_shape(
    ground,
    &ShapeDef::default(),
    &make_box_hull(20.0, 0.5, 20.0),
);

// 一圈动态箱子
let mut boxes = Vec::new();
for i in 0..8 {
    let angle = (i as f32) * std::f32::consts::TAU / 8.0;
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.5 * angle.cos(), 0.5, 1.5 * angle.sin());
    def.rotation = Quat::IDENTITY;
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.2, 0.2, 0.2));
    boxes.push(body);
}

// 先让它们落稳
for _ in 0..60 {
    world.step(1.0 / 60.0, 4);
}

// 在中心引爆
let mut explosion = ExplosionDef::default();
explosion.position = Pos::new(0.0, 0.3, 0.0);
explosion.radius = 2.0;
explosion.falloff = 1.5;
explosion.impulse_per_area = 250.0;
world.explode(&explosion);

// 爆炸后立刻能看到径向速度
for body in &boxes {
    let v = world.body_linear_velocity(*body);
    println!("speed = {}", (v.x * v.x + v.y * v.y + v.z * v.z).sqrt());
}

for _ in 0..120 {
    world.step(1.0 / 60.0, 4);
}
```

## 注意事项

- **浮力不会自动发生**：这是最重要的一条。`step` 不读液面、不做浮力。每个时间步都要自己调 `apply_buoyancy_impulse` 或 `apply_buoyancy`，漏调的那一步物体就等于在真空里自由落体。
- **液面法线指向流体外部**：水面在 `y = 0`、水在下方时法线是 `(0, 1, 0)`。反了会把物体往水里压。判断标准是"法线指向空气那一侧"。
- **`time_step` 要与 `step` 一致**：浮力冲量是按时间步缩放的，传错会让浮力过强或过弱，物体表现为缓慢下沉或猛烈弹出。
- **`buoyancy` 是密度比不是密度**：填 `1000.0`（水的密度）会让物体获得千倍浮力直接飞走。想要"木头浮在水上"，`buoyancy` 约等于 2.0。
- **爆炸只影响动态刚体**：静态与运动学刚体完全免疫，不需要额外排除地面。
- **`explosion_scale` 为 0 的形状免疫**：这是让特定物体忽略爆炸的正式手段，比调 `mask_bits` 更精确。
- **`explode` 的断言只在 debug 构建生效**：位置非法、`radius` 或 `falloff` 为负在 debug 下会 panic，在 release 下不检查。不要依赖 release 里的防御。
- **批量版本返回数量**：`apply_buoyancy` / `apply_buoyancy_in_aabb` 返回受影响的刚体数，但一个刚体有多个形状时只算一次，适合做统计而不适合逐个精确控制。
- **传感器形状不参与排水体积**：`body_submerged_volume` 会跳过传感器形状。给触发区域挂传感器不会凭空产生浮力。
- **高度场没有体积**：用高度场当地形时它不参与浮力计算，这是设计而非缺陷——高度场是开放表面。

## 相关页面

- [刚体](/docs/bodies)：浮力与爆炸都作用于动态刚体。
- [形状](/docs/shapes)：`ShapeDef::explosion_scale` 与形状体积决定受力。
- [材质与过滤](/docs/materials)：`mask_bits` 与 `category_bits` 控制爆炸与浮力的作用范围。
- [空间查询](/docs/queries)：`body_submerged_volume` 属于只读查询，可自由调用。
- [软体](/docs/soft-body)：软体不参与爆炸冲量，浮力也需要自行处理。
- [调试绘制](/docs/debug-draw)：可视化液面与爆炸半径。
