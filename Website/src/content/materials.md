# 材质与过滤

[形状](/docs/shapes) 决定"碰撞几何长什么样"，本页决定两件事：**碰撞发生时的表面行为**（摩擦、弹性、滚动阻力、传送带速度）和**两两之间是否允许碰撞**（类别位与过滤）。

这两件事都在 `ShapeDef` 里配置，但语义完全不同：材质是"接触之后怎么算"，过滤是"要不要生成接触"。

## 表面材质 SurfaceMaterial

`SurfaceMaterial` 描述一个表面的物理属性：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `friction` | `f32` | `0.6` | 库仑（干）摩擦系数，通常 `0..1` |
| `restitution` | `f32` | `0.0` | 弹性（恢复）系数，通常 `0..1` |
| `rolling_resistance` | `f32` | `0.0` | 滚动阻力，通常 `0..1` |
| `tangent_velocity` | `Vec3` | `Vec3::ZERO` | 切向速度，局部坐标，用于传送带 |
| `user_material_id` | `u64` | `0` | 用户自定义材质 ID，会出现在事件与查询结果里 |
| `custom_color` | `u32` | `0` | 调试绘制颜色，`0` 表示使用默认 |
| `padding` | `u32` | `0` | 对齐填充，必须为 `0` |

注意 `SurfaceMaterial` 派生的 `Default` 是**全零**（摩擦也是 0），而 `SurfaceMaterial::default_material()` 才给出摩擦 0.6 的可用默认值。这两个容易混淆：

```rust
use natsel::types::shape_def::SurfaceMaterial;

let slippery = SurfaceMaterial::default();          // friction = 0.0，完全无摩擦
let normal = SurfaceMaterial::default_material();   // friction = 0.6
```

`ShapeDef::default()` 内部用的是 `default_material()`，所以不显式设置材质时摩擦是 0.6。但如果你写 `SurfaceMaterial { friction: 0.3, ..Default::default() }`，其余字段就是全零而不是 `default_material()` 的值 —— 这在逐三角形材质里很常见。

稳妥写法是显式指定每个要用的字段：

```rust
SurfaceMaterial {
    friction: 0.3,
    restitution: 0.1,
    ..SurfaceMaterial::default_material()
}
```

### 摩擦

摩擦决定物体沿表面滑动时受到的阻力。0 是无摩擦冰面，1 大致是橡胶。

```rust
use natsel::types::shape_def::ShapeDef;

let mut sd = ShapeDef::default();
sd.base_material.friction = 0.9;
```

常用参考：

| 场景 | `friction` |
|------|-----------|
| 冰面 | 0.0 ~ 0.1 |
| 抛光地板 | 0.2 ~ 0.4 |
| 普通地面 | 0.6 |
| 粗糙混凝土 | 0.8 ~ 0.9 |
| 橡胶、防滑垫 | 0.9 ~ 1.0 |

摩擦系数超过 1 是允许的（物理上也有对应材料），但它不会让物体"更粘"，只是让静摩擦阈值更高，可能造成求解器抖动。超过 2 通常说明你在用摩擦实现别的效果。

**摩擦不能让静止物体停住。** 静摩擦依赖于接触求解器的迭代收敛，极端堆叠或很高摩擦时可能需要更多子步。

### 弹性

弹性决定碰撞后法向速度恢复的比例。0 是完全非弹性（撞上就停），1 是完美弹性（同速反弹）。

```rust
sd.base_material.restitution = 0.8;
```

弹性最终表现为反弹高度比：

| `restitution` | 反弹高度 |
|---------------|----------|
| 0.0 | 不反弹 |
| 0.5 | 约 25% 原高度 |
| 0.8 | 约 64% 原高度 |
| 1.0 | 约 100%（理想，实际有能量损失） |

要看到真正的反弹，物体必须以一定速度撞上去 —— 一个缓慢放下的球即使 `restitution = 1.0` 也不会弹起来。

引擎有速度阈值 `world.restitution_threshold`（默认 1.0）：低于该速度的接触不做恢复计算。这是防止"静止物体因微小数值抖动而持续弹跳"的必要措施。想让低速碰撞也有弹性，可以调低它，但要接受随之而来的抖动。

```rust
world.restitution_threshold = 0.2;
```

相关字段还有 `world.restitution_iterations`（恢复迭代次数，默认 2）和 `world.enable_restitution_propagation`（弹性在堆叠中传播，默认关闭）。这两者通常不需要动，只有做弹球机、弹跳床这类高弹性场景时才值得调。

### 滚动阻力

滚动阻力专门针对"球和圆柱不该永远滚下去"的问题。摩擦对滚动的球几乎不消耗能量（接触点没有相对滑动），所以纯摩擦的球会一直滚。

```rust
sd.base_material.rolling_resistance = 0.05;
```

它的作用方式是给接触施加一个抵抗相对角速度的力矩，有效半径按形状估算（球和胶囊用自身半径，凸包用内切半径的四分之一）。所以同样的系数，大球比小球停得慢。

典型值：

| 场景 | `rolling_resistance` |
|------|---------------------|
| 光滑保龄球 | 0.0 ~ 0.01 |
| 台球 | 0.01 ~ 0.03 |
| 草地上的球 | 0.05 ~ 0.1 |
| 泥地、沙地 | 0.2 以上 |

### 传送带：切向速度

`tangent_velocity` 让表面像传送带一样"带着"接触物体移动：

```rust
let mut conveyor_def = ShapeDef::default();
conveyor_def.base_material = SurfaceMaterial::default_material();
conveyor_def.base_material.friction = 0.8;                        // 摩擦要够高才能带动
conveyor_def.base_material.tangent_velocity = Vec3::new(2.0, 0.0, 0.0);
```

几个要点：

- **速度是局部坐标**，会随刚体旋转一起转。世界空间的效果是 `rotate(shape_rotation, tangent_velocity)`。所以传送带跟着刚体转的时候，传送方向也转 —— 这对于旋转的转盘式传送带正是想要的。
- **必须配合足够的摩擦**。切向速度只是"表面想跑多快"，实际能拖动多少物体取决于摩擦力上限。摩擦太低，物体几乎不动。
- 两个都有切向速度的表面相接触时，效果是两者的**差**：`tangent_velocity_a - tangent_velocity_b`。所以两条同向同速的传送带之间不会有相对滑动。
- 传送带不需要是动态体，静态刚体上的形状完全够用，而且更省。

把传送带和传感器组合，就能做出"进入区域后被带着走"的传送带关卡。

### 用户材质 ID

`user_material_id` 是给你的游戏逻辑用的标记，引擎不解释它。它会被带进事件与查询结果：

```rust
sd.base_material.user_material_id = 42;
```

之后在接触事件、命中事件、射线查询结果里都能读到对应形状的 `user_material_id`。用它来区分"木头 42 / 金属 43 / 血肉 44"，就能在碰撞回调里统一播放对应音效，而不必对每个形状维护一张表。事件字段见 [事件](/docs/events)，查询见 [空间查询](/docs/queries)。

### 调试颜色

```rust
sd.base_material.custom_color = 0xFF8800;   // 0xRRGGBB 或引擎预设色
```

非零时，调试绘制会用这个颜色代替默认的体色。`0` 表示"用默认"。见 [调试绘制](/docs/debug-draw)。

## 材质混合规则

两个形状接触时，接触点用哪一方的摩擦和弹性？答案是**混合**，且摩擦与弹性的规则不同：

| 属性 | 混合方式 |
|------|----------|
| 摩擦 | 几何平均：`sqrt(friction_a * friction_b)` |
| 弹性 | 取最大值：`max(restitution_a, restitution_b)` |
| 滚动阻力 | 取最大值后乘以有效半径：`max(rr_a, rr_b) * radius` |
| 切向速度 | 相减：`tangent_velocity_a - tangent_velocity_b` |

这些规则有实际后果：

**几何平均意味着"有一方很滑，接触就很滑"。** 冰面（0.05）与橡胶（1.0）之间是 `sqrt(0.05) ≈ 0.22` —— 比你直觉的更接近冰。想让某处真的很滑，两侧材质都得设低。

**取最大值意味着"有一方弹，就会弹"。** 弹力球（0.9）撞上完全非弹性的地面（0.0）依然会弹，因为混合结果取 0.9。如果你想让某个区域"吸收所有弹跳"，需要显式把弹性设成 0 在那里并不能阻止对面的弹性 —— 这种情况应该用 pre-solve 回调改接触属性，或者干脆让落地的物体弹性为 0。

**滚动阻力也取最大值**，所以给地面设上滚动阻力，就能让所有球类物体在地面上自然停下，不需要逐个设置球。

## 逐三角形材质

网格和高度场可以给每个三角形指定不同材质 —— 一块地面同时有草地、泥地、冰面。

做法分两步：把材质表放进 `ShapeDef.materials`，让几何携带材质索引。

```rust
use natsel::collision::mesh::procedural::create_grid_mesh;
use natsel::math::types::Vec3;
use natsel::types::shape_def::{ShapeDef, SurfaceMaterial};

// 房间里三种材质：粗糙地面、冰面、弹跳板
let mut sd = ShapeDef::default();
sd.materials = vec![
    SurfaceMaterial { friction: 0.9, restitution: 0.0, ..Default::default() },
    SurfaceMaterial { friction: 0.05, restitution: 0.0, ..Default::default() },
    SurfaceMaterial { friction: 0.6, restitution: 0.9, ..Default::default() },
];

// material_count = 3：三角形会按顺序轮流使用第 0/1/2 号材质
let mesh = create_grid_mesh(8, 8, 2.0, 3, true).expect("grid");
let _ = world.create_mesh_shape(ground_body, &sd, mesh, Vec3::ONE);
```

索引链是这样工作的：

1. 几何（`TriangleMesh` 或 `HeightField`）为每个三角形存一个**槽索引**（`u8`）。
2. `ShapeDef.materials` 是材质数组，槽索引就是它的小标。
3. 接触时，引擎按三角形槽索引取到材质，再和对方形状的 `base_material` 混合。

所以 `ShapeDef.materials` 的长度必须**大于**几何里出现的最大槽索引。索引越界会被夹到最后一个材质，不会崩，但你会得到意料之外的摩擦力。

重要细节：**`ShapeDef.materials` 里只有一个元素时，它被当作 `base_material` 的替代**；长度大于 1 时才启用逐三角形查找。这也是为什么 `ShapeDef::default()` 的 `materials` 是空的：

| `materials` 长度 | 行为 |
|------------------|------|
| 0 | 用 `base_material` |
| 1 | 用 `materials[0]`（忽略 `base_material`） |
| 2 以上 | 按三角形槽索引查找 |

实测确认：`base_material.friction = 0.2` 配合 `materials` 只有一个摩擦 0.9 的元素时，形状的实际摩擦是 0.9。

复合形状是例外：它的材质来自 `CompoundDef` 里各子形状的 `material` 字段，`ShapeDef.materials` 会被完全忽略。要让复合体有材质，得在构造 `CompoundDef` 时给每个子形状设好。

`create_grid_mesh` 的 `material_count` 参数会自动把三角形轮流分配给各材质，是最快的"多材质地面"原型。

### 运行时修改材质

```rust
// 读/写基础材质
let m = world.shape_surface_material(shape);   // SurfaceMaterial
world.set_shape_surface_material(shape, new_material);

// 只改摩擦或弹性
world.set_shape_friction(shape, 0.4);
world.set_shape_restitution(shape, 0.7);
let f = world.shape_friction(shape);
let r = world.shape_restitution(shape);

// 逐三角形材质
let count = world.shape_mesh_material_count(shape);        // i32
if let Some(m) = world.shape_mesh_surface_material(shape, 0) {
    println!("槽 0 摩擦 = {}", m.friction);
}
world.set_shape_mesh_material(shape, 0, new_material);
```

`shape_surface_material` 读的是槽 0。修改基础材质不会影响逐三角形材质 —— 它们各自独立。

## 碰撞过滤

过滤决定两个形状是否**生成接触**。它比材质更早生效：过不了过滤，连接触都不会创建，也就谈不上摩擦。

`Filter` 有三个字段：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `category_bits` | `u64` | 全 1 | 这个形状属于哪些类别 |
| `mask_bits` | `u64` | 全 1 | 这个形状愿意和哪些类别碰撞 |
| `group_index` | `i32` | `0` | 正数强制碰撞，负数强制不碰撞，0 走位掩码 |

有两个构造入口：

```rust
use natsel::types::shape_def::Filter;

let everything = Filter::default_filter();          // 全 1，与一切碰撞
let custom = Filter::new(0b0010, 0b1100, 0);        // (category, mask, group)
```

### 位掩码语义

判定规则是**双向**的：两个形状必须互相"看得上"才碰撞。

```text
(a.mask & b.category) != 0  且  (b.mask & a.category) != 0
```

两个条件都要满足。这比"任一方允许就行"更严格，意味着你可以单方面屏蔽：只要把 A 的 `mask` 里清掉 B 的类别，A 和 B 就不碰，不需要改 B。

### 完整示例：玩家 / 敌人 / 子弹

```rust
use natsel::collision::hull::make_cube_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::{Filter, ShapeDef};

// 类别位：玩家 = 0b0001，敌人 = 0b0010，子弹 = 0b0100，场景 = 0b1000
const PLAYER: u64 = 0b0001;
const ENEMY: u64 = 0b0010;
const BULLET: u64 = 0b0100;
const WORLD: u64 = 0b1000;

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    // 场景：和所有东西碰撞
    let ground_body = world.create_body(&BodyDef::default());
    let mut ground_def = ShapeDef::default();
    ground_def.filter = Filter::new(WORLD, PLAYER | ENEMY | BULLET, 0);
    let _ = world.create_hull_shape(
        ground_body,
        &ground_def,
        &natsel::collision::hull::make_box_hull(50.0, 0.5, 50.0),
    );

    // 玩家：和场景、敌人碰，但不和自己碰
    let mut pdef = BodyDef::default();
    pdef.type_ = BodyType::DynamicBody;
    pdef.position = Pos::new(0.0, 2.0, 0.0);
    let player = world.create_body(&pdef);
    let mut psd = ShapeDef::default();
    psd.filter = Filter::new(PLAYER, WORLD | ENEMY, 0);
    let _ = world.create_hull_shape(player, &psd, &make_cube_hull(0.5));

    // 敌人：和场景、玩家碰，不和别的敌人碰
    let mut edef = BodyDef::default();
    edef.type_ = BodyType::DynamicBody;
    edef.position = Pos::new(2.0, 4.0, 0.0);
    let enemy = world.create_body(&edef);
    let mut esd = ShapeDef::default();
    esd.filter = Filter::new(ENEMY, WORLD | PLAYER, 0);
    let _ = world.create_hull_shape(enemy, &esd, &make_cube_hull(0.5));

    // 子弹：和场景、敌人碰，穿过玩家（友军伤害关闭）
    let mut bdef = BodyDef::default();
    bdef.type_ = BodyType::DynamicBody;
    bdef.position = Pos::new(0.0, 3.0, -3.0);
    bdef.linear_velocity = Vec3::new(0.0, 0.0, 20.0);
    let bullet = world.create_body(&bdef);
    let mut bsd = ShapeDef::default();
    bsd.filter = Filter::new(BULLET, WORLD | ENEMY, 0);
    let _ = world.create_hull_shape(bullet, &bsd, &make_cube_hull(0.1));

    for _ in 0..120 {
        world.step(1.0f32 / 60.0, 4);
    }

    println!("contacts = {}", world.contact_count());
    println!("player y = {:.2}", world.body_position(player).y);
    println!("enemy  y = {:.2}", world.body_position(enemy).y);
    println!("bullet y = {:.2}", world.body_position(bullet).y);
}
```

这个例子里，子弹的 `mask` 里没有 `PLAYER`，玩家的 `mask` 里也没有 `BULLET`，双向都不匹配，所以子弹穿过玩家而不产生任何力。

### 组索引

`group_index` 优先于位掩码，用来做"临时成组"的效果：

```text
if a.group == b.group && a.group != 0 {
    碰撞 = (a.group > 0)
} else {
    走位掩码判定
}
```

也就是说：

- 两个形状 `group_index` 相同且为正数 → **强制碰撞**，忽略掩码。
- 两个形状 `group_index` 相同且为负数 → **强制不碰撞**，忽略掩码。
- 不同（或都是 0）→ 回到位掩码。

负数组最实用的场景是布娃娃：同一个角色的所有部件设成同一个负数组，它们永远不会互相碰撞，省掉了大量无意义的接触求解。

```rust
let ragdoll_group = -7;
for part in &ragdoll_parts {
    let mut sd = ShapeDef::default();
    sd.filter.group_index = ragdoll_group;
    // ...
}
```

### 运行时修改过滤

```rust
let f = world.shape_filter(shape);     // Filter
world.set_shape_filter(shape, Filter::new(new_cat, new_mask, 0));
```

修改过滤不会立即重建已存在的接触 —— 它影响下一帧的接触创建判定。想让改动立刻生效，改完再 `step` 一次即可。

### 查询过滤

空间查询用 `QueryFilter`，字段与 `Filter` 类似但没有 `group_index`，多了 `id` 与 `name`：

| 字段 | 类型 | 说明 |
|------|------|------|
| `category_bits` | `u64` | 查询自身的类别 |
| `mask_bits` | `u64` | 愿意匹配哪些类别 |
| `id` | `u64` | 可选的形状 ID 过滤 |
| `name` | `Option<String>` | 可选的形状名过滤 |

```rust
use natsel::types::shape_def::QueryFilter;

let qf = QueryFilter::default_filter();            // 查询一切
let only_enemies = QueryFilter::new(0, ENEMY);     // 只查敌人类别
```

查询的匹配规则同样是双向的。详见 [空间查询](/docs/queries)。

## 自定义过滤回调

位掩码表达不了复杂的游戏规则（"只有玩家持钥匙时这扇门才开"）。这时用回调：

```rust
use std::sync::Arc;
use natsel::types::ids::ShapeID;

world.set_custom_filter(Some(Arc::new(|a: ShapeID, b: ShapeID| -> bool {
    // 返回 false 表示禁止这一对碰撞
    a != b
})));
```

回调签名是 `Fn(ShapeID, ShapeID) -> bool + Send + Sync`。返回 `false` 禁止接触，`true` 允许。

关键条件：**参与的两个形状中至少有一个的 `ShapeDef.enable_custom_filtering` 为 `true`，回调才会被调用。**

```rust
let mut sd = ShapeDef::default();
sd.enable_custom_filtering = true;
```

如果不设这个标志，即使注册了回调也完全不会触发 —— 这是最常见的"我的过滤函数没被调用"原因。

回调在接触创建时逐对调用，所以每帧的开销与候选接触对数量成正比。保持它简单、无锁，不要在里面做分配或 I/O。

## Pre-solve 回调

`set_pre_solve_callback` 在求解之前调用，让你按接触的动态条件决定这一步是否响应。

```rust
use std::sync::Arc;
use natsel::math::types::{Pos, Vec3};
use natsel::types::ids::ShapeID;

world.set_pre_solve_callback(Some(Arc::new(
    |a: ShapeID, b: ShapeID, point: Pos, normal: Vec3| -> bool {
        // 返回 false 禁用这个接触当步的响应
        true
    },
)));
```

签名是 `Fn(ShapeID, ShapeID, Pos, Vec3) -> bool + Send + Sync`，参数是形状 A、形状 B、世界空间的接触点、接触法线。

**返回 `false` 会禁用该接触在这一步的响应**，形状可以互相穿过。这正是单向平台的实现方式。

### 触发时机

回调**不是在每一个接触上、每一步都调用**。它只在接触的流形被重新计算时触发：引擎会缓存一对形状之间的相对位姿，当这个位姿没有实质变化、接触被直接复用时，会跳过整个窄相位更新，回调也就不会被调用。

实践中的表现是：一个静止在平台上的物体，回调在开始接触时触发一次，之后保持静止的每一步都不再触发；一旦它滑动或被撞击、接触需要重算，回调会再次出现。

这对单向平台的实现有一个直接影响：**不要在回调里做一次性的状态写入**，比如在回调里记录"已通过"。坚持写无状态的判断——只看当前的形状对和法线方向——这样无论回调触发几次，结果都一致。

```rust
// 好：无状态，任何时候调用都给出一致答案
world.set_pre_solve_callback(Some(Arc::new(move |a, _b, _point, normal| {
    if a == platform { normal.y > 0.0 } else { normal.y < 0.0 }
})));

// 坏：依赖调用次数，触发时机一变行为就变
world.set_pre_solve_callback(Some(Arc::new(move |_a, _b, _point, _normal| {
    if !passed_once.swap(true, Ordering::Relaxed) { false } else { true }
})));
```

### 法线方向

关键在于**法线方向是从 A 指向 B**，而 A、B 的顺序取决于形状创建顺序（先创建的通常是 A）。因此判断"从下往上穿"时要按自己的场景写，最稳妥的写法是按形状 ID 判断：

```rust
use std::sync::Arc;

// 记住平台的 ShapeID，然后在回调里按它分支
world.set_pre_solve_callback(Some(Arc::new(move |a, _b, _point, normal| {
    if a == platform {
        normal.y > 0.0    // 平台是 A：法线朝上说明对方在上方，拦截
    } else {
        normal.y < 0.0    // 平台是 B：法线由对方指向平台，方向相反
    }
})));
```

实测行为（平台先创建、因而是 A）：物体从下方冲上来时 `normal.y == -1.0`，回调返回 `false`，物体穿过；物体从上方落下时 `normal.y == 1.0`，回调返回 `true`，物体停在平台顶面。

与自定义过滤的区别：

| | 自定义过滤 | pre-solve |
|---|---|---|
| 调用时机 | 接触**创建**时 | 接触流形需要重算时 |
| 返回值含义 | 这一对是否允许有接触 | 这一步是否响应 |
| 开销 | 低（只在创建时） | 高于过滤（参与求解路径） |
| 典型用途 | 静态的规则（队伍、层） | 动态条件（单向平台、可开关的碰撞） |

同样需要形状上的 `enable_pre_solve_events = true` 作为开关（任一方的形状开启即可）：

```rust
let mut sd = ShapeDef::default();
sd.enable_pre_solve_events = true;
```

回调里也能读到 `world` 之外的游戏状态（通过 `Arc<Mutex<...>>` 或原子变量），所以在里面做"门是否解锁"这类判断是可行的。但注意它每步都会被调用，保持判断廉价。

## 摩擦与弹性回调

引擎的摩擦/弹性混合规则是固定的（几何平均 / 取最大），但外部无法通过公开接口替换它们 —— 混合逻辑是引擎内部的一部分。如果默认规则不满足需求，实际做法是：

- 调整两侧材质，让默认混合规则算出想要的结果。
- 或者用 pre-solve 回调在特定场景下禁用接触，改由你自己的逻辑施加力。

## 完整的材质配置示例

一辆在地面上的推车，轮子低摩擦、车身正常摩擦，地面有滚动阻力，并带一条传送带：

```rust
use natsel::collision::hull::{make_box_hull, make_cube_hull};
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::{Capsule, Sphere};
use natsel::types::shape_def::{Filter, ShapeDef, SurfaceMaterial};

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    // 地面：正常摩擦 + 滚动阻力（让轮子自然减速）
    let ground_body = world.create_body(&BodyDef::default());
    let mut ground_sd = ShapeDef::default();
    ground_sd.base_material = SurfaceMaterial {
        friction: 0.7,
        restitution: 0.0,
        rolling_resistance: 0.03,
        ..SurfaceMaterial::default_material()
    };
    ground_sd.base_material.user_material_id = 1;   // 1 = 水泥地
    let _ = world.create_hull_shape(ground_body, &ground_sd, &make_box_hull(50.0, 0.5, 50.0));

    // 传送带：切向速度朝 +X
    let mut belt_sd = ShapeDef::default();
    belt_sd.base_material = SurfaceMaterial {
        friction: 0.9,
        tangent_velocity: Vec3::new(3.0, 0.0, 0.0),
        user_material_id: 2,                         // 2 = 传送带
        ..SurfaceMaterial::default_material()
    };
    let _ = world.create_hull_shape(
        ground_body,
        &belt_sd,
        &make_box_hull(5.0, 0.25, 5.0),
    );

    // 推车：一个车身凸包 + 两个球体当"轮子"
    let mut cart_def = BodyDef::default();
    cart_def.type_ = BodyType::DynamicBody;
    cart_def.position = Pos::new(-8.0, 1.0, 0.0);
    let cart = world.create_body(&cart_def);

    // 车身：高摩擦、无弹性
    let mut body_sd = ShapeDef::default();
    body_sd.density = 400.0;
    body_sd.base_material = SurfaceMaterial {
        friction: 0.8,
        user_material_id: 3,                         // 3 = 木头
        ..SurfaceMaterial::default_material()
    };
    let _ = world.create_hull_shape(cart, &body_sd, &make_cube_hull(0.5));

    // 轮子：低摩擦，让车能滑起来
    let mut wheel_sd = ShapeDef::default();
    wheel_sd.density = 100.0;
    wheel_sd.base_material = SurfaceMaterial {
        friction: 0.1,
        user_material_id: 4,                         // 4 = 橡胶
        ..SurfaceMaterial::default_material()
    };
    let w1 = world.create_sphere_shape(
        cart,
        &wheel_sd,
        Sphere::new(Vec3::new(0.0, -0.5, 0.6), 0.3),
    );
    let _ = world.create_sphere_shape(
        cart,
        &wheel_sd,
        Sphere::new(Vec3::new(0.0, -0.5, -0.6), 0.3),
    );

    println!("cart mass = {:.1}", world.body_mass(cart));

    for _ in 0..300 {
        world.step(1.0f32 / 60.0, 4);
    }

    println!("cart x = {:.2}", world.body_position(cart).x);
    println!("wheel friction = {:.2}", world.shape_friction(w1));
}
```

## 常见错误

**`SurfaceMaterial::default()` 当作默认材质。** 它派生的 `Default` 是全零，摩擦为 0。默认材质是 `SurfaceMaterial::default_material()`（摩擦 0.6）。

**用 `..Default::default()` 改造材质。** 如果这样写 `SurfaceMaterial { friction: 0.3, ..Default::default() }`，弹性、滚动阻力等全部是 0 而不是 `default_material()` 的值。用 `..SurfaceMaterial::default_material()` 更安全。

**以为低摩擦的一侧能让接触变滑。** 混合是几何平均，冰面碰橡胶仍然有约 0.22 的摩擦。要真正滑，两侧都要低。

**以为 `restitution = 0` 能阻止对面的弹力球弹跳。** 混合取最大值，弹力球的 0.9 会胜出。要吸收弹跳，得让落地物的弹性也低，或者用 pre-solve 回调干预。

**注册了 `set_custom_filter` 但形状没开 `enable_custom_filtering`。** 回调完全不会调用。两个形状里至少一个要打开这个标志。

**只检查 `enable_custom_filtering` 而没检查 pre-solve 开关。** pre-solve 回调需要形状上的 `enable_pre_solve_events = true` 才会对该接触生效。

**`ShapeDef.materials` 长度和几何的材质索引对不上。** 索引越界会被夹到最后一个材质，你会得到"某些三角形摩擦力莫名一致"的现象。确保 `materials.len()` 大于几何里出现的最大槽索引。

**过滤条件写成单向。** 位掩码判定是双向的。只想让 A 不碰 B 时，清掉 A 的 `mask` 里 B 的类别就够了（因为双向要求已不满足），但要确认 B 的位没有意外把 A 又加回去。

## 下一步

- [形状](/docs/shapes)：`ShapeDef` 的其他字段，传感器与事件开关。
- [几何构建器](/docs/geometry)：如何为网格和高度场生成材质索引。
- [事件](/docs/events)：如何在事件里读 `user_material_id`。
- [空间查询](/docs/queries)：`QueryFilter` 的用法。
- [调试绘制](/docs/debug-draw)：`custom_color` 如何影响显示。
