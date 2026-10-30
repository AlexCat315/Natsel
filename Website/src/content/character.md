# 虚拟角色

`CharacterVirtual` 是一个运动学角色控制器：它不在刚体求解器里，而是在每帧用"碰撞与滑动"（collide and slide）把一枚胶囊体推过世界。它自己感知地面与坡度、上下楼梯、在斜坡上吸附地面、跟随移动平台、推动动态刚体，并且可以拥有一个运动学"内体"（inner body），让普通刚体与空间查询也能看见它。

角色适合做玩家与 NPC：手感可控、不会因为数值抖动而摔倒，又能与刚体世界双向交互。如果你需要的是被物理完全驱动的身体，请看 [骨骼与布娃娃](/docs/skeleton)。

## 模型说明

角色与世界的交互分成三层：

| 层次 | 内容 |
|------|------|
| 胶囊 | 由 `radius` 与 `height` 决定，胶囊底部就是 `position`（脚底） |
| 皮肤（padding） | 胶囊与世界的间隙，避免几何体互相嵌入；`character_padding` |
| 内体 | 可选的 `KinematicBody`，位置跟随角色，物理世界通过它感知角色 |

`update` 只做水平移动：把 `linear_velocity * dt` 作为目标位移，逐次收集接触平面、求解平面约束、用胶囊投射（shape cast）确认剩余行程，重复 `max_collision_iterations` 次。`extended_update` 在 `update` 之上再叠加三件事：

1. 楼梯行走：水平位移被陡峭接触挡住时，尝试"上升—前进—下降"，落在可站立平面上才算成功。
2. 地面吸附：走出台阶边缘时向下投射一小段 `step_down_height`，仍可站立就贴回地面。
3. 压住地面刚体：站在动态刚体上时按 `mass` 与重力给它一个向下的冲量。

## 创建角色

构造函数是可失败的，并且可变借用世界：

```rust
pub fn new(world: &'w mut World, def: &CharacterDef) -> Option<Self>
```

返回 `None` 的条件为：`radius <= 0`、`height < 2 * radius`、`position` 非法（NaN 或无穷），或 `up` 长度为零。`up` 会被归一化。

```rust
use natsel::character::character_virtual::{CharacterDef, CharacterVirtual};
use natsel::math::types::Pos;

let mut def = CharacterDef::new();
def.position = Pos::new(0.0, 1.0, 0.0); // 脚底
let mut character = CharacterVirtual::new(&mut world, &def).expect("character");
```

### CharacterDef 字段

`CharacterDef::new()` 按长度单位缩放库的默认值（`b3::length_units_per_meter()`）。

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `position` | `Pos` | `Pos::default()` | 脚底位置，即胶囊底部 |
| `up` | `Vec3` | `(0, 1, 0)` | 角色上方，构造时归一化 |
| `radius` | `f32` | `0.3 * units` | 胶囊半径 |
| `height` | `f32` | `1.8 * units` | 胶囊总高，必须 `>= 2 * radius` |
| `max_slope_angle` | `f32` | `50°` | 可站立的最陡坡度 |
| `mass` | `f32` | `70.0` | 用于推动刚体与压住地面 |
| `max_strength` | `f32` | `100.0` | 最大推力（N） |
| `character_padding` | `f32` | `0.02 * units` | 胶囊与世界之间的皮肤厚度 |
| `predictive_contact_distance` | `f32` | `0.1 * units` | 超出皮肤多远仍报告接触，用于提前感知地面 |
| `max_collision_iterations` | `i32` | `5` | 每次更新的碰撞与滑动轮数 |
| `step_up_height` | `f32` | `0.4 * units` | 可跨上的最高台阶 |
| `step_down_height` | `f32` | `0.5 * units` | 吸附地面时向下探测的距离 |
| `stair_step_forward_test` | `f32` | `0.15 * units` | 落在台阶边缘时向前复检的距离 |
| `enable_stair_stepping` | `bool` | `true` | 是否启用楼梯行走 |
| `enable_stick_to_floor` | `bool` | `true` | 是否启用地面吸附 |
| `inherit_ground_velocity` | `bool` | `true` | 是否跟随所站刚体一起移动 |
| `filter` | `QueryFilter` | `QueryFilter::default_filter()` | 参与碰撞的形状过滤 |
| `create_inner_body` | `bool` | `true` | 是否创建运动学内体 |
| `inner_body_shape` | `ShapeDef` | `ShapeDef::default()` | 内体的形状定义（密度、材质、过滤） |
| `user_data` | `Option<*mut c_void>` | `None` | 用户指针 |

`max_collision_iterations` 在构造时会被夹到至少 1。

## 每帧更新

设置 `linear_velocity`，然后调用 `update` 或 `extended_update`：

```rust
character.linear_velocity = Vec3::new(4.0, 0.0, 0.0);
character.extended_update(1.0 / 60.0, Vec3::new(0.0, -9.81, 0.0));
```

`linear_velocity` 是一个普通 `pub` 字段。`update` 结束后它会沿接触平面被裁剪（`clip_vector`），也就是说撞墙后不会保留指向墙内的分量。

`extended_update` 的 `gravity` 参数只用来确定"下方"，并用于压住地面刚体；重力对速度的积分仍由调用方负责——你需要自己把 `gravity * dt` 加到 `linear_velocity` 上。

## 地面状态与接触

`ground_state()` 返回四种状态：

| 状态 | 含义 |
|------|------|
| `CharacterGroundState::OnGround` | 站在可通行的地面上 |
| `CharacterGroundState::OnSteepGround` | 接触地面但坡度过陡 |
| `CharacterGroundState::NotSupported` | 有接触，但支撑力向上分量不足 |
| `CharacterGroundState::InAir` | 没有接触 |

与之配套的读取接口：

| 方法 | 返回 | 说明 |
|------|------|------|
| `ground_normal()` | `Vec3` | 地面法线（可能是多个支撑面的平均） |
| `ground_position()` | `Pos` | 地面接触点 |
| `ground_velocity()` | `Vec3` | 地面在该点相对世界的速度 |
| `ground_body_id()` | `BodyID` | 地面刚体，无支撑时为空 ID |
| `ground_shape_id()` | `ShapeID` | 地面形状 |
| `ground_user_material_id()` | `u64` | 地面材质 ID，用于脚步声与表面效果 |
| `contacts()` | `&[CharacterContact]` | 上次更新接触到的形状 |
| `is_supported()` | `bool` | `OnGround` 或 `OnSteepGround` |
| `is_slope_walkable(normal)` | `bool` | 该法线是否可站立 |

`CharacterContact` 字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `shape_id` | `ShapeID` | 被接触的形状 |
| `body_id` | `BodyID` | 所属刚体 |
| `position` | `Pos` | 接触点 |
| `normal` | `Vec3` | 由形状指向角色 |
| `depth` | `f32` | 相对"加了皮肤"的胶囊的穿深，负值表示留有间隙 |
| `user_material_id` | `u64` | 形状材质 ID |

`depth` 与 `-(character_padding + linear_slop)` 比较可以判断"是否真的接触"：`update` 之前它是预测性的，因此能提前报告即将落地的地面。

## 内体

当 `create_inner_body` 为真时，构造过程会创建一个 `KinematicBody`，并在其上挂一个与角色胶囊同形的形状。此后每次 `update` / `extended_update` 都会把该刚体的目标变换设为角色的位置与朝向（`set_body_target_transform`），因此：

- 刚体会与角色碰撞，角色自己则通过 `ignore_body` 忽略它；
- 点查询、射线、`overlaps` 等空间查询能命中这个胶囊；
- `InnerBody` 只反映位置，不反映角色语义，别把它当成物理身体使用。

`inner_body_id()` 返回这个刚体的 ID。

## 完整示例

角色与世界的借用关系是本页最容易出错的点：`CharacterVirtual<'w>` 在存活期间独占 `&'w mut World`。因此"更新角色"和"推进世界"必须通过一个显式的作用域或指针来分时进行。引擎提供的出路是显式分时：把 world 的可变借用通过裸指针交给角色，二者交替使用。这是长生命周期角色的标准写法：

```rust
use natsel::character::character_virtual::{
    CharacterDef, CharacterGroundState, CharacterVirtual,
};
use natsel::collision::hull::make_transformed_box_hull;
use natsel::math::functions::mul_add;
use natsel::math::types::{Pos, Quat, Transform, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::BodyDef;
use natsel::types::shape_def::ShapeDef;

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);

// 地面：静态刚体 + 盒形状
let ground = world.create_body(&BodyDef::default());
let _ = world.create_hull_shape(
    ground,
    &ShapeDef::default(),
    &make_transformed_box_hull(20.0, 0.5, 20.0, Transform::new(Vec3::ZERO, Quat::IDENTITY)),
);

let dt: f32 = 1.0 / 60.0;
let gravity = Vec3::new(0.0, -9.81, 0.0);

let mut def = CharacterDef::new();
def.position = Pos::new(0.0, 2.0, 0.0);

// 角色独占 world 的可变借用；用裸指针把两者分开访问，
// 保证任何时刻只有一方在使用 world。
let world_ptr: *mut World = &mut world;
// SAFETY: world 在 character 的全部使用期间保持存活，且没有任何访问重叠。
let mut character = unsafe { CharacterVirtual::new(&mut *world_ptr, &def) }.expect("character");

for frame in 0..240 {
    // 前 120 帧站立，之后向前走
    let walk = if frame < 120 {
        Vec3::ZERO
    } else {
        Vec3::new(2.0, 0.0, 0.0)
    };

    // 重力要累加到当前速度上（mul_add(v, s, b) 求 v + s * b）。
    // Vec3 没有运算符重载，所以用 mul_add 而不是 +=。
    let mut velocity = character.linear_velocity;
    if character.ground_state() == CharacterGroundState::OnGround {
        // 站在地面上时用行走速度，不再叠加重力，否则贴地会持续下沉
        velocity = walk;
    } else {
        // 空中：保留水平输入，垂直方向累积重力
        velocity.x = walk.x;
        velocity.z = walk.z;
        velocity = mul_add(velocity, dt, gravity);
    }
    character.linear_velocity = velocity;

    character.extended_update(dt, gravity);

    // SAFETY: extended_update 已返回，此刻没有任何借用在进行。
    unsafe { (*world_ptr).step(dt, 4) };
}

assert!(character.ground_state() == CharacterGroundState::OnGround);
println!("feet = {:?}", character.position());
println!("normal = {:?}", character.ground_normal());

// 世界还要继续用时，先移除内体，避免角色离开后刚体残留。
character.destroy();
```

如果角色只需要存活在当前帧之内，可以完全用安全代码写成作用域形式：

```rust
let dt: f32 = 1.0 / 60.0;
{
    let mut character = CharacterVirtual::new(&mut world, &def).expect("character");
    character.linear_velocity = Vec3::new(2.0, 0.0, 0.0);
    character.extended_update(dt, Vec3::new(0.0, -9.81, 0.0));
} // 借用在此结束
world.step(dt, 4);
```

这种写法每帧都会重建内体，只适合短生命周期的角色，或者 `create_inner_body = false` 的场景。

## 状态保存与恢复

`save_state()` 返回 `State`，`restore_state(state)` 写回。状态包含位置、朝向、线速度、`up`、胶囊尺寸、地面信息、接触列表与内体 ID。回滚整个场景时应当把它与 `World` 的快照一起处理，因为内体本身活在世界里。

## 方法一览

| 方法 | 说明 |
|------|------|
| `position()` | 脚底位置 |
| `up()` / `radius()` / `height()` | 胶囊参数 |
| `set_max_slope_angle(angle)` | 修改可站立坡度。必须用它而不是直接写 `max_slope_angle` 字段 |
| `ground_state()` 及各 `ground_*` | 地面信息 |
| `contacts()` | 上次更新的接触 |
| `inner_body_id()` | 内体刚体 ID |
| `set_shape(radius, height, max_penetration)` | 改变胶囊（例如蹲下），新形状与世界重叠超过 `max_penetration` 时返回 `false` 并保持原状 |
| `set_position(pos)` | 传送角色 |
| `is_supported()` / `is_slope_walkable(normal)` | 站立判定 |
| `update(dt)` | 碰撞与滑动 + 地面刷新 + 推动刚体 |
| `extended_update(dt, gravity)` | 上面的全部，再加楼梯、吸附、压地面 |
| `save_state()` / `restore_state(state)` | 状态快照 |
| `destroy()` | 移除内体 |
| `contact_filter` | `Option<Box<dyn Fn(ShapeID, BodyID) -> bool>>`，返回 `false` 表示忽略该形状 |

## 注意事项

- **借用与生命周期是首要约束**：`CharacterVirtual<'w>` 持有 `&'w mut World`。角色存活期间你无法再通过 `&mut world` 调用 `step`、`create_body` 等任何方法，也无法通过共享引用查询世界——角色没有公开的世界访问器。要么用作用域让借用在 `step` 之前结束，要么用裸指针分时访问。
- **构造函数是可失败的**：`new` 返回 `Option`，不要直接解引用。`radius <= 0`、`height < 2 * radius`、非法位置、零长度 `up` 都会返回 `None`。
- **`position` 是脚底而不是中心**：想让角色站在 `y = 0` 的地面上，`position.y` 需要约为 0，而不是 `height / 2`。
- **`linear_velocity` 会被改写**：`update` 结束后它沿接触平面被裁剪，撞墙后不要指望原值还在。不要用它做"输入缓存"，另存一份目标速度。
- **重力要自己积分**：`extended_update` 的 `gravity` 参数只决定向下方向并用于压地面刚体；速度积分由调用方完成。
- **`set_max_slope_angle` 而不是直接写字段**：`max_slope_angle` 虽然是 `pub` 字段，但直接赋值不会让 `is_slope_walkable` 生效——可站立判定依赖 setter 同步的状态。改坡度请一律走 setter。
- **内体需要显式清理**：世界比角色活得久时，离开前调用 `destroy()`，否则运动学胶囊会留在世界里。
- **`max_collision_iterations` 不是越大越好**：它影响每次更新的开销；构造时会被夹到至少 1，设成 0 或负数不会让角色穿过世界，只会退化成一次迭代。
- **接触是预测性的**：`depth` 为负且绝对值不超过 `predictive_contact_distance` 时，形状已经在接触列表里但尚未真正触碰——判断"落地"请用 `ground_state()`，不要只看 `contacts().is_empty()`。

## 相关页面

- [刚体](/docs/bodies)：内体是 `KinematicBody`，静态地面与动态道具都是刚体。
- [形状](/docs/shapes)：胶囊与盒形状是角色碰撞的基础。
- [空间查询](/docs/queries)：角色内部的碰撞与滑动依赖形状投射与重叠查询。
- [材质与过滤](/docs/materials)：`filter` 与 `user_material_id` 控制角色和谁碰撞、踩在什么表面上。
- [骨骼与布娃娃](/docs/skeleton)：需要被物理驱动的身体时改用布娃娃。
- [调试绘制](/docs/debug-draw)：把角色胶囊与接触可视化。
