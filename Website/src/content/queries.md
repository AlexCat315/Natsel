# 空间查询

力量与碰撞之外，游戏还需要"读取"世界：鼠标点到了谁、角色脚下有没有地面、爆炸范围内有哪些可击碎的箱子。Natsel 提供三类查询——包围盒重叠、形状重叠、射线与形状扫掠。

## 查询模型

一次查询由三部分组成：

1. **查询区域**：一个 `AABB`、一个 `ShapeProxy`，或一条射线/一次扫掠
2. **`QueryFilter`**：按碰撞类别位筛掉不关心的形状
3. **回调**：每命中一个形状调用一次，由你决定记录、忽略还是提前收敛

查询是**只读的**：`&self` 借用世界，不会唤醒物体、不产生接触、不改变任何状态。可以在渲染帧里随意调用。

查询分两个层级：

| 层级 | 入口 | 用途 |
|------|------|------|
| 世界级 | `overlap_aabb`、`overlap_shape`、`cast_ray`、`shape_cast` 及其 `_closest` 变体 | 在全世界范围内查找 |
| 单体级 | `body_cast_ray`、`body_cast_shape`、`body_overlap_shape`、`shape_ray_cast` | 精确检查指定的一个刚体或形状 |

返回值分两种：收集型（`cast_ray_closest`、`shape_cast_closest`）直接返回一个结果结构体；回调型（`cast_ray`、`overlap_aabb` 等）返回 `TreeStats`，只包含遍历成本。

## 重叠查询

### `overlap_aabb`

```rust
pub fn overlap_aabb<F>(
    &self,
    aabb: AABB,
    filter: QueryFilter,
    callback: F,
) -> TreeStats
where
    F: FnMut(ShapeID) -> bool
```

回调返回 `true` 继续遍历，返回 `false` **提前终止**整次查询。只有形状的 AABB 与查询 AABB 相交才算命中——这是粗筛，可能包含实际并不重叠的形状。

### `overlap_shape`

```rust
pub fn overlap_shape<F>(
    &self,
    origin: Pos,
    proxy: ShapeProxy,
    filter: QueryFilter,
    callback: F,
) -> TreeStats
where
    F: FnMut(ShapeID) -> bool
```

比 `overlap_aabb` 精确：先用 AABB 粗筛，再做真实的形状重叠测试。`origin` 是 proxy 在世界中的摆放位置，proxy 自身的点是局部坐标。

`ShapeProxy` 表示任意凸体：

```rust
pub struct ShapeProxy {
    pub points: Option<*const Vec3>,
    pub count: i32,
    pub radius: f32,
}

impl ShapeProxy {
    pub fn new(points: Option<*const Vec3>, count: i32, radius: f32) -> Self;
    pub fn points(&self) -> &[Vec3];
}
```

`points` 是裸指针，因此 **`ShapeProxy` 借用了你的点数组**，调用期间点数组必须存活且不被移动。用 `radius` 表示球的半径，`count` 必须至少为 1——要表示球，传入一个点（通常是 `Vec3::ZERO`）加半径。

## 射线查询

### 回调式 `cast_ray`

```rust
pub fn cast_ray<F>(
    &self,
    origin: Pos,
    translation: Vec3,
    filter: QueryFilter,
    callback: F,
) -> TreeStats
where
    F: FnMut(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32
```

射线的方向与长度合并成 `translation`：从 `origin` 出发，终点是 `origin + translation`。`fraction` 是命中点在整段位移上的比例，`0.0` 是起点、`1.0` 是终点。

回调参数依次为：

| 位置 | 类型 | 含义 |
|------|------|------|
| 1 | `ShapeID` | 命中的形状 |
| 2 | `Pos` | 世界坐标命中点（f64） |
| 3 | `Vec3` | 命中处法线 |
| 4 | `f32` | 命中比例 `fraction` |
| 5 | `u64` | 命中三角形的 `user_material_id` |
| 6 | `i32` | 三角形索引（网格/高度场），否则为 0 |
| 7 | `i32` | 子形状索引（复合形状），否则为 0 |

**回调的返回值是核心约定**：

- 返回一个新的 `fraction`（通常取更小的值）= 收缩后续搜索范围，更近的命中会淘汰更远的
- 返回 `-1.0` = **跳过这个形状**，本次命中作废，继续搜索
- 返回 `0.0` = 立即结束整次查询

**回调顺序不是按距离排列的**，而是包围盒树的遍历顺序。因此不要假设第一个回调就是最近的命中，必须在回调里自己比较 `fraction` 并返回更小的值来收敛。

### 收集式 `cast_ray_closest`

```rust
pub fn cast_ray_closest(
    &self,
    origin: Pos,
    translation: Vec3,
    filter: QueryFilter,
) -> RayResult
```

不需要写回调。它内部忽略 `fraction == 0.0` 的初始重叠命中，因此**起点位于某形状内部时不会返回该形状**。

`RayResult` 字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `shape_id` | `ShapeID` | 命中的形状 |
| `point` | `Pos` | 世界坐标命中点 |
| `normal` | `Vec3` | 表面法线 |
| `user_material_id` | `u64` | 命中三角形/形状的材质 ID |
| `fraction` | `f32` | 命中比例 |
| `triangle_index` | `i32` | 网格三角形索引 |
| `child_index` | `i32` | 复合子形状索引 |
| `node_visits` | `i32` | 遍历节点数 |
| `leaf_visits` | `i32` | 遍历叶子数 |
| `hit` | `bool` | 是否命中 |

未命中时 `hit` 为 `false`，其余字段保持默认值。

## 形状扫掠

形状扫掠把一个凸体沿 `translation` 平移，找出它第一次碰到东西的位置——用于"胶囊体移动会不会撞墙"这类问题。

```rust
pub struct ShapeCastInput {
    pub proxy: ShapeProxy,
    pub translation: Vec3,
    pub max_fraction: f32,
    pub can_encroach: bool,
}
```

| 字段 | 说明 |
|------|------|
| `proxy` | 被扫掠的凸体 |
| `translation` | 扫掠位移 |
| `max_fraction` | 只关心这段比例内的命中 |
| `can_encroach` | 允许初始已重叠（入侵）的状态参与计算 |

```rust
pub fn shape_cast<F>(
    &self,
    origin: Pos,
    input: ShapeCastInput,
    filter: QueryFilter,
    callback: F,
) -> TreeStats
where
    F: FnMut(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32

pub fn shape_cast_closest(
    &self,
    origin: Pos,
    input: ShapeCastInput,
    filter: QueryFilter,
) -> ShapeCastResult
```

回调签名与返回值约定和 `cast_ray` **完全一致**，因此同样可以用 `-1.0` 跳过形状。

`ShapeCastResult` 与 `RayResult` 相同，只是少了 `node_visits` / `leaf_visits`。

> `shape_cast_closest` 同样忽略 `fraction == 0.0`，所以初始就重叠的形状不会被报告。

## 单体查询

想知道"这个射线会不会打到这个刚体"，用单体级接口可以避免全世界的遍历开销。

```rust
pub fn body_cast_ray(
    &self,
    body_id: BodyID,
    origin: Pos,
    translation: Vec3,
    filter: &QueryFilter,
    max_fraction: f32,
    body_transform: WorldTransform,
) -> BodyCastResult

pub fn body_cast_shape(
    &self,
    body_id: BodyID,
    origin: Pos,
    proxy: ShapeProxy,
    translation: Vec3,
    filter: &QueryFilter,
    max_fraction: f32,
    can_encroach: bool,
    body_transform: WorldTransform,
) -> BodyCastResult

pub fn body_overlap_shape(
    &self,
    body_id: BodyID,
    origin: Pos,
    proxy: ShapeProxy,
    filter: &QueryFilter,
    body_transform: WorldTransform,
) -> bool

pub fn shape_ray_cast(
    &self,
    shape_id: ShapeID,
    origin: Pos,
    translation: Vec3,
) -> WorldCastOutput
```

注意两点差异：`filter` 在这里是**引用**（`&QueryFilter`），并且需要显式传入刚体的 `WorldTransform`（一般就是 `world.body_transform(id)`）。射线起点 `origin` 用来做精度重定心，因此远离世界原点时依然准确。

`BodyCastResult` 字段：

| 字段 | 类型 |
|------|------|
| `shape_id` | `ShapeID` |
| `point` | `Pos` |
| `normal` | `Vec3` |
| `fraction` | `f32` |
| `triangle_index` | `i32` |
| `user_material_id` | `u64` |
| `iterations` | `i32` |
| `hit` | `bool` |

`WorldCastOutput` 类似，额外带 `material_index`，且没有 `shape_id`：

| 字段 | 类型 |
|------|------|
| `normal` | `Vec3` |
| `point` | `Pos` |
| `fraction` | `f32` |
| `iterations` | `i32` |
| `triangle_index` | `i32` |
| `child_index` | `i32` |
| `material_index` | `i32` |
| `hit` | `bool` |

## 过滤

`QueryFilter` 决定哪些形状会被回调。它和形状自身的 `Filter` 做双向位测试：

```rust
pub struct QueryFilter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub id: u64,
    pub name: Option<String>,
}
```

| 字段 | 说明 |
|------|------|
| `category_bits` | 本次查询"属于"哪些类别 |
| `mask_bits` | 本次查询想匹配哪些类别 |
| `id` | 用户标识，不参与过滤 |
| `name` | 用户标识，不参与过滤 |

命中条件为**双向匹配**：形状的 `category_bits` 必须与查询的 `mask_bits` 有交集，同时形状的 `mask_bits` 必须与查询的 `category_bits` 有交集。

`QueryFilter::default_filter()` 给出两侧全 1 的位，即不过滤任何形状。用 `QueryFilter::new(category_bits, mask_bits)` 构造自定义过滤器。

### 完整示例

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{AABB, Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::Sphere;
use natsel::types::queries::{ShapeCastInput, ShapeProxy};
use natsel::types::shape_def::{QueryFilter, ShapeDef};

fn main() {
    let mut world = World::default();
    world.enable_sleep = false;

    // 地面
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_box_hull(20.0, 0.5, 20.0),
    );

    // 一个球
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 4.0, 0.0);
    let ball = world.create_body(&def);
    let mut ball_def = ShapeDef::default();
    ball_def.density = 1000.0;
    let ball_shape = world.create_sphere_shape(ball, &ball_def, Sphere::new(Vec3::ZERO, 0.5));

    let filter = QueryFilter::default_filter();

    // 1. 包围盒重叠
    let region = AABB::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0));
    let mut overlaps = Vec::new();
    let stats = world.overlap_aabb(region, filter.clone(), |shape_id| {
        overlaps.push(shape_id.index1);
        true
    });
    println!("重叠 {} 个形状，访问 {} 个节点", overlaps.len(), stats.node_visits);

    // 2. 最近射线命中
    let ray = world.cast_ray_closest(
        Pos::new(0.0, 10.0, 0.0),
        Vec3::new(0.0, -20.0, 0.0),
        filter.clone(),
    );
    if ray.hit {
        println!("命中 shape {} 于 y={}", ray.shape_id.index1, ray.point.y);
    }

    // 3. 球扫掠：一个半径 0.25 的球自上而下扫过
    let center = Vec3::ZERO;
    let proxy = ShapeProxy::new(Some(&center as *const Vec3), 1, 0.25);
    let mut input = ShapeCastInput::default();
    input.proxy = proxy;
    input.translation = Vec3::new(0.0, -10.0, 0.0);
    input.max_fraction = 1.0;

    let cast = world.shape_cast_closest(Pos::new(0.0, 8.0, 0.0), input, filter.clone());
    println!("扫掠命中 {}，比例 {:.3}", cast.hit, cast.fraction);

    // 4. 回调式射线：跳过球，只找其余物体
    let mut best: f32 = 1.0;
    world.cast_ray(
        Pos::new(0.0, 10.0, 0.0),
        Vec3::new(0.0, -20.0, 0.0),
        filter,
        |shape_id, _point, _normal, fraction, _material, _tri, _child| {
            if shape_id == ball_shape {
                return -1.0; // 忽略球，继续搜索
            }
            if fraction < best {
                best = fraction;
            }
            best // 收缩搜索范围
        },
    );
    println!("排除球后最近比例 {:.3}", best);
}
```

## 常见问题

**回调必须声明为 `FnMut`。** 需要用闭包累积状态（计数、最近命中、结果列表）时，闭包会捕获可变引用。若变量在查询外还要用，注意查询结束后借用才归还：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::queries::ShapeProxy;
use natsel::types::shape_def::{QueryFilter, ShapeDef};

let mut world = World::default();
let mut def = BodyDef::default();
def.type_ = BodyType::StaticBody;
def.position = Pos::new(3.0, 0.0, 0.0);
let body = world.create_body(&def);
let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(1.0, 1.0, 1.0));

let mut nearest = f32::MAX;
world.cast_ray(
    Pos::ZERO,
    Vec3::new(10.0, 0.0, 0.0),
    QueryFilter::default_filter(),
    |_, _, _, fraction, _, _, _| {
        nearest = nearest.min(fraction);
        nearest // 返回更小的 fraction，后续更远的命中会被跳过
    },
);
// 此处 nearest 可正常读取
```

如果需要同时记录多个字段，把它们打包进一个结构体再用 `&mut` 捕获，比分散捕获更清晰。

**`-1.0` 与 `0.0` 的含义完全不同。** `-1.0` 只跳过当前形状；`0.0` 会立刻终止整次查询。想"排除某类物体后继续找"，返回 `-1.0`；想"找到第一个就停"，返回 `0.0`。

**查询过滤器与形状过滤器是两套机制。** `QueryFilter` 只影响查询，不影响物理碰撞；碰撞过滤用的是形状上的 `Filter`（见[材质与过滤](/docs/materials)）。想让某形状在物理上碰撞、但被查询忽略，只需在查询的 `mask_bits` 里清掉对应位。

**`cast_ray_closest` 不报告起点的初始重叠。** 若需要"起点在内部也算命中"，改用回调式 `cast_ray` 并自行保留 `fraction == 0.0` 的命中。

**`ShapeProxy` 至少需要一个点。** `count == 0` 或 `points == None` 会在内部计算包围盒时 panic。表示一个球时传一个点加半径：

```rust
let points = [Vec3::new(-0.5, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0)];
let proxy = ShapeProxy::new(Some(points.as_ptr()), 2, 0.1);
world.overlap_shape(Pos::ZERO, proxy, QueryFilter::default_filter(), |_| true);

let ball_center = [Vec3::ZERO];
let ball = ShapeProxy::new(Some(ball_center.as_ptr()), 1, 0.5);
```

不要让点数组在查询前被移动或释放。

**`TreeStats` 只反映成本。** `node_visits` 与 `leaf_visits` 用于诊断查询开销，与命中数量无关。大量查询导致 `node_visits` 偏高时，应优先缩小查询区域或用过滤器裁剪类别。

## 相关页面

- [形状](/docs/shapes) —— 形状与 `ShapeProxy` 的关系
- [材质与过滤](/docs/materials) —— 物理碰撞过滤（`Filter`）
- [连续碰撞检测](/docs/ccd) —— 高速物体的扫掠处理
- [角色控制器](/docs/character) —— 基于扫掠的角色移动
