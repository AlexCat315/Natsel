# 坐标与单位

Natsel 同时使用 `f64` 世界坐标和 `f32` 局部坐标。理解这个分工，以及单位设置如何影响默认值，能避免绝大部分“物体抖动”“远处穿模”的问题。

## Pos 与 Vec3 的分工

| 类型 | 字段 | 精度 | 用途 |
|------|------|------|------|
| `Pos` | `x`, `y`, `z` | `f64` | 刚体在世界中的绝对位置 |
| `Vec3` | `x`, `y`, `z` | `f32` | 方向、速度、局部点、局部几何 |
| `Vec2` | `x`, `y` | `f32` | 二维量（极少用，如履带角速度） |
| `Transform` | `p: Vec3`, `q: Quat` | 混合 | **局部**变换：相对某个参考点 |
| `WorldTransform` | `p: Pos`, `q: Quat` | 混合 | **世界**变换：绝对位置 |

规则很简单：

- 说“在哪里”用 `Pos`。
- 说“往哪个方向”“移动多快”“形状有多大”用 `Vec3`。

```rust
use natsel::math::types::{Pos, Vec3};

let world_position = Pos::new(1.0e6, 12.5, -3.0);   // 世界里在哪
let velocity = Vec3::new(0.0, -2.0, 1.5);           // 速度用 f32
```

### 为什么世界位置用 f64

`f32` 只有 24 位有效位。数值到 1e6 量级时，最小可分辨间隔约 0.06；到 1e7 时约 0.95——已经比很多物体的尺寸还大。这意味着如果你用 `f32` 存世界坐标，物体在离原点几公里外就会开始抖动、互相穿透。

`f64` 有 53 位有效位，同样的 1e9 量级仍然精确到约 1e-7。Natsel 的碰撞与查询内部会构造一个 `f32` 的“以查询原点为中心”的局部坐标系，于是 `f32` 的精度只作用在小范围内，绝对坐标的精度由 `f64` 保证。

```rust
use natsel::math::types::Pos;

// 1e9 处的物体仍然稳定：位置本身是 f64
let far_away = Pos::new(1.0e9 + 0.125, 1.0e9, -1.0e9);

// 查询会自动重定心，命中点仍然是 f64 世界位置
let hit = world.shape_ray_cast(shape, far_away, Vec3::new(0.0, -10.0, 0.0));
```

所有查询 API 都遵循这一约定：`cast_ray`、`shape_cast`、`body_cast_ray`、`shape_ray_cast`、`collide_mover`、`body_submerged_volume` 等接收 `origin: Pos`，返回的命中点是 `Pos`。你不需要自己做任何补偿。

### 唯一的例外

`overlap_aabb` 接收 `AABB`（`f32`），因为它是宽相阶段的包围盒查询。如果你在极远处需要精确包围盒查询，用 `overlap_world_aabb(WorldAABB, ...)`——它接收 `Pos` 边界并在命中后做精确复核，过滤掉宽相 `f32` 树产生的假阳性。

## Quat 的布局与默认值

```rust
pub struct Quat {
    pub v: Vec3,   // 虚部 (x, y, z)
    pub s: f32,    // 实部 w
}
```

即 `v = (x, y, z)`、`s = w`，与世界坐标无关，始终是 `f32`。

**关键陷阱**：`Quat` 派生的是普通 `Default`，全零。全零四元数不是单位旋转，用它创建刚体会触发断言（`is_valid_quat` 要求四元数归一化）。

```rust
use natsel::math::types::Quat;

let mut def = natsel::types::body_def::BodyDef::default();
// def.rotation 已经是 Quat::IDENTITY，直接用默认值即可

// 需要显式写的时候：
def.rotation = Quat::IDENTITY;          // 正确
// def.rotation = Quat::default();      // 错误：全零
```

同理，`Transform::default()` 也是全零四元数；用 `Transform::IDENTITY`。

需要绕轴旋转时：

```rust
use natsel::math::functions::make_quat_from_axis_angle;
use natsel::math::types::Vec3;

let quarter_turn = make_quat_from_axis_angle(Vec3::new(0.0, 1.0, 0.0), std::f32::consts::FRAC_PI_2);
```

## 常量速查

| 常量 | 值 |
|------|-----|
| `Quat::IDENTITY` | `{ v: (0,0,0), s: 1 }` |
| `Transform::IDENTITY` | `{ p: (0,0,0), q: IDENTITY }` |
| `WorldTransform::IDENTITY` | `{ p: Pos::ZERO, q: IDENTITY }` |
| `Pos::ZERO` | `(0.0, 0.0, 0.0)` |
| `Vec3::ZERO` / `ONE` / `AXIS_X` / `AXIS_Y` / `AXIS_Z` | 对应向量 |
| `Matrix3::IDENTITY` / `ZERO` | 单位 / 零矩阵 |

`Matrix3` 按列存储：`{ cx, cy, cz }`。惯量张量相关的 API（`body_local_rotational_inertia`、`body_world_inverse_rotational_inertia`、`MassData::inertia`）都用它。

## 单位：长度单位每米

Natsel 内部只认“长度单位”，不认米。默认 1 个长度单位 = 1 米。你可以让 1 个单位等于任意长度：

```rust
use natsel::base::constants::set_length_units_per_meter;

// 1 单位 = 1 厘米
set_length_units_per_meter(100.0);

// 1 单位 = 1 英尺
set_length_units_per_meter(3.280_84);
```

读取当前值：

```rust
let units = natsel::base::constants::length_units_per_meter();  // 或 get_length_units_per_meter()
```

### 哪些默认值会随之缩放

单位不是只影响一个数字——一批默认值和容差都按它换算。**这些值在 `BodyDef::default()` / `ShapeDef::default()` 被调用的那一刻计算**，之后再改全局单位不会回溯修改已经创建的默认值。

| 量 | 默认值 | 来源 |
|----|--------|------|
| `BodyDef::sleep_threshold` | `0.05 * units` | 睡眠速度阈值 |
| `ShapeDef::density` | `1000 / units³` | 密度，使质量在任意单位下一致 |
| `JointDef::draw_scale` | `units` | 关节绘制比例 |
| `CharacterDef::radius` | `0.3 * units` | 角色半径 |
| `CharacterDef::height` | `1.8 * units` | 角色总高 |
| `CharacterDef::character_padding` | `0.02 * units` | 角色表皮厚度 |
| `CharacterDef::step_up_height` | `0.4 * units` | 可跨越台阶高度 |
| `CharacterDef::step_down_height` | `0.5 * units` | 下台阶贴地距离 |
| `SoftBodyDef::max_linear_velocity` | `500 * units` | 软体顶点速度上限 |
| `SoftBodyDef::sleep_threshold` | `0.03 * units` | 软体睡眠阈值 |
| `WorldDef::gravity` 相关阈值 | 见下 | 世界定义默认值 |

`WorldDef::default()`（描述默认世界设置的结构体）里的缩放值：

| 字段 | 默认值 |
|------|--------|
| `gravity` | `(0, -10, 0)` |
| `restitution_threshold` | `units` |
| `hit_event_threshold` | `units` |
| `contact_speed` | `3 * units` |
| `maximum_linear_speed` | `400 * units` |

碰撞容差同样按单位缩放（内部按 `linear_slop = 0.005 * units` 派生）：接触允许穿透量、推测接触距离、接触复用距离、胶囊最短长度、AABB 最大外边距都属于这一类。你不需要手动设置它们，但如果把单位改得很大或很小，要意识到这些容差也跟着变了。

### 推荐做法：先定单位，再建世界

因为默认值在构造时求值，正确顺序是：

```rust
use natsel::base::constants::set_length_units_per_meter;
use natsel::types::body_def::BodyDef;
use natsel::types::shape_def::ShapeDef;

// 1. 先设定单位
set_length_units_per_meter(1.0);

// 2. 再创建世界和定义
let mut world = natsel::simulation::World::default();
let def = BodyDef::default();          // sleep_threshold 已按新单位算好
let shape_def = ShapeDef::default();   // density 已按新单位算好
```

反过来做（先建定义、后改单位）会得到混合单位的世界：一部分对象用旧默认值、一部分用新默认值。这种不一致极难排查，也没有 API 能修正它。

### 单位与质量的连锁关系

`ShapeDef::density` 默认是 `1000 / units³`，目的是让“1 米立方、密度 1000”的质量恒为 1000 kg。如果你用厘米（`units = 100`），默认密度变成 `0.001`，1 单位³ 的物体质量是 0.001——看起来小得奇怪，但乘以体积（100³ 倍）后总质量是正确的。

所以：**要么全用默认密度，要么显式设 `density`，不要混着猜。**

## 常见误用

| 现象 | 原因 | 修法 |
|------|------|------|
| 远处物体抖动 | 用了 `Vec3` 存世界坐标 | 改用 `Pos`；查询传 `origin: Pos` |
| 刚体不动、断言失败 | `Quat::default()` 全零 | 用 `Quat::IDENTITY` |
| 物体轻得离谱或重得离谱 | 改了单位但沿用旧 `density` | 重新构造 `ShapeDef` 或显式设密度 |
| 角色浮在地面上方 | 改单位后 `character_padding` 未更新 | 重新构造 `CharacterDef` |
| 接触穿插严重 | 单位过大导致 `linear_slop` 相对物体过大 | 缩小单位，或放大物体尺寸 |

更多排障见 [常见问题](/docs/faq)。
