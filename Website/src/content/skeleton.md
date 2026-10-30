# 骨骼与布娃娃

这一页覆盖三个互相衔接的模块：

- **骨骼**（`Skeleton` / `SkeletonPose`）：命名关节的层级结构，以及关节变换在"相对父节点"与"模型空间"之间的换算。
- **布娃娃**（`RagdollSettings` / `Ragdoll`）：为每个关节生成一个刚体，相邻关节用六自由度关节连接，让角色的身体被物理驱动。
- **骨骼映射**（`SkeletonMapper`）：在低精度游戏骨架与高精度布娃娃骨架之间搬运姿态。

`Skeleton` 与 `SkeletonPose` 也可以完全脱离刚体单独使用：它们是骨骼动画的底层数据结构。

## 骨骼模型说明

一个 `Skeleton` 就是一张有序的关节表，每个关节记录名字与父关节索引，父索引必须指向更早加入的关节。根关节的父索引是 `-1`。

姿态有两种表示，二者可以互相换算：

| 表示 | 字段 | 含义 |
|------|------|------|
| 关节状态 | `SkeletonPose::joint_states` | 相对父关节的变换 |
| 关节矩阵 | `SkeletonPose::joint_matrices` | 模型空间变换 |

`root_offset` 是模型空间原点在世界中的位置。因此关节的世界变换等于 `root_offset` 加上关节矩阵的平移分量。这个拆分让骨骼可以随角色整体移动而不必重算所有关节矩阵。

### Skeleton

| 方法 | 说明 |
|------|------|
| `Skeleton::new()` | 空骨架 |
| `add_joint(name, parent) -> i32` | 在父关节之后加入关节，返回索引。名字重复或父索引非法会 panic |
| `add_joint_with_parent_name(name, parent_name) -> i32` | 按父关节名字加入；名字未知会 panic |
| `joints() -> &[Joint]` | 关节列表，`Joint { name, parent_index }` |
| `joint_count() -> usize` | 关节数量 |
| `joint_index(name) -> Option<i32>` | 按名字查找索引 |
| `parent_index(joint) -> i32` | 父索引，根为 `-1` |
| `is_ancestor(ancestor, joint) -> bool` | `ancestor` 是否在 `joint` 的父链上（不含自身） |

```rust
use natsel::skeleton::skeleton::Skeleton;

let mut skeleton = Skeleton::new();
skeleton.add_joint("pelvis", -1);
skeleton.add_joint_with_parent_name("spine", "pelvis");
skeleton.add_joint_with_parent_name("head", "spine");
skeleton.add_joint_with_parent_name("thighL", "pelvis");
skeleton.add_joint_with_parent_name("calfL", "thighL");
```

### SkeletonPose

| 方法 | 说明 |
|------|------|
| `SkeletonPose::new(skeleton)` | 用骨架创建姿态；状态为零平移 + 单位旋转，矩阵为单位变换 |
| `joint_count() -> usize` | 关节数量（等于 `joint_states.len()`） |
| `calculate_joint_matrices()` | 由关节状态算出模型空间矩阵 |
| `calculate_joint_states()` | 由模型空间矩阵反算关节状态 |
| `world_transform(joint) -> WorldTransform` | 关节世界变换 |
| `set_world_transform(joint, WorldTransform)` | 写入关节矩阵（减去 `root_offset`） |

两个 `calculate_*` 都按关节顺序单趟遍历，因此要求父关节排在子关节之前——这正是 `add_joint` 强制"父必须先加入"的原因。

`SkeletonJointState` 相关：

```rust
SkeletonJointState::new(translation: Vec3, rotation: Quat)
SkeletonJointState::from_transform(transform: Transform)
state.transform() -> Transform
```

## 骨骼动画

`SkeletalAnimation` 用关键帧驱动姿态。每条 `Track` 按名字绑定一个关节，关键帧存局部平移与旋转，采样时线性插值平移、球面插值旋转。

| 类型/方法 | 说明 |
|-----------|------|
| `Keyframe::new(time, translation, rotation)` | 单个关键帧 |
| `Track::new(joint_name, keyframes)` | 构造时按时间排序；`keyframes()` 只读访问 |
| `Track::sample(time) -> Option<SkeletonJointState>` | 在给定时间采样，超出首尾关键帧时钳制 |
| `SkeletalAnimation::new(tracks, is_looping)` | 一个动画 |
| `duration() -> f32` | 最后关键帧的时间 |
| `sample(time, pose, weight)` | 采样并混合进 `pose` 的关节状态 |

`sample` 的 `weight` 会被夹到 `0..1`：权重 1 直接覆盖关节状态，更小的权重向采样值插值，因此可以叠加多层动画。没有轨道的关节保持原状。循环动画会把时间回绕到 `[0, duration)`。采样后需要调用 `calculate_joint_matrices()` 才能更新模型空间。

`weight` 没有默认值，完全覆盖时传 `1.0`。

## 布娃娃

`Ragdoll` 为每个关节生成一个刚体，刚体的位置与旋转就是该关节的模型变换。每个有父关节的部分都用六自由度关节连到父刚体，并按其 `RagdollJointKind` 配置摆动/扭转限制。

### RagdollSettings

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `skeleton` | `Skeleton` | — | 骨架 |
| `parts` | `Vec<RagdollPart>` | — | 每个关节一个部分，按骨架顺序 |
| `self_collision` | `RagdollSelfCollision` | `SkipOverlapping { margin: 0.02 }` | 部分之间的自碰撞策略 |
| `collision_group` | `i32` | `-1` | `None` 策略使用的负碰撞组 |
| `max_mass_ratio` | `f32` | `4.0` | 沿每个关节限制质量比；`0` 关闭 |

`RagdollSettings::new(skeleton, parts)` 会填好上面三个默认值。

`RagdollSelfCollision` 的四个变体：

| 变体 | 含义 |
|------|------|
| `None` | 部分之间永不碰撞（共用一个负碰撞组） |
| `SkipConnected` | 只跳过有直接关节连接的部分 |
| `SkipOverlapping { margin }` | 额外跳过创建姿态下包围盒重叠（并膨胀 `margin`）的部分 |
| `All` | 所有配对都碰撞，包括相连的 |

### RagdollPart

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `shapes` | `Vec<RagdollShape>` | 空 | 在关节空间里的碰撞几何 |
| `shape_def` | `ShapeDef` | 默认 | 密度、摩擦、过滤、事件 |
| `body_def` | `BodyDef` | 默认 | 阻尼、重力缩放、睡眠、bullet 等；类型/位置/旋转由布娃娃覆盖 |
| `joint` | `RagdollJointKind` | `SwingTwist { cone_angle: 0.5, twist_min: -0.3, twist_max: 0.3 }` | 与父关节的连接方式；根部分忽略 |
| `joint_frame` | `Transform` | 单位 | 关节在部分空间里的坐标系，其 x 轴是扭转轴 |
| `joint_friction` | `f32` | `0.0` | 抵抗关节转动的力矩（N·m） |
| `limit_hertz` / `limit_damping_ratio` | `f32` | `0.0` | 软限制；hertz 为 0 表示刚性 |

`RagdollPart::new()` 得到默认值，`RagdollPart::with_shapes(shapes, joint, joint_frame)` 是便捷构造。

`RagdollShape` 有 `Capsule`、`Sphere`、`Box { half_extents, transform }`、`Hull` 四种。

`RagdollJointKind` 有 `SwingTwist`、`Pyramid`、`Hinge { lower, upper }`、`Fixed` 四种（角度单位是弧度，扭转轴都是 `joint_frame` 的 x 轴）。

### 创建布娃娃

```rust
pub fn new(
    world: &'w mut World,
    settings: RagdollSettings,
    pose: &SkeletonPose,
    user_data: Option<*mut c_void>,
) -> Option<Ragdoll<'w>>
```

返回 `None` 的条件是：部分数量与骨架关节数不一致、姿态关节数与骨架关节数不一致，或关节数为 0。

创建过程会依次：按姿态的关节矩阵创建动态刚体、挂上形状、按 `max_mass_ratio` 稳定质量、为每个非根部分创建六自由度关节、在 `SkipOverlapping` 下为包围盒重叠的非相连部分创建过滤关节。

### Ragdoll 方法

| 方法 | 说明 |
|------|------|
| `bodies() -> &[BodyID]` | 每个关节对应的刚体 |
| `joints() -> &[JointID]` | 连接各部分的关节；根位置为默认 ID |
| `filter_joints() -> &[JointID]` | 仅用于禁用重叠部分间碰撞的关节 |
| `destroy()` | 销毁全部刚体与关节 |
| `root_body() -> BodyID` | 根刚体 |
| `pose() -> SkeletonPose` | 当前姿态，`root_offset` 取根刚体位置 |
| `set_pose(&pose, reset_velocities)` | 把各部分传送到位姿，可选清零速度 |
| `drive_to_pose_using_motors(&pose, max_torque, hertz, damping_ratio)` | 用位置马达驱动到目标姿态，仍是动态的 |
| `disable_motors()` | 关掉之前打开的马达 |
| `drive_to_pose_using_kinematics(&pose, time_step)` | 按速度驱动到位姿，每步之前调用 |
| `set_linear_velocity(velocity)` | 设置所有部分的速度 |
| `apply_linear_impulse(impulse)` | 按质量分配冲量，使每个部分获得相同速度变化 |
| `total_mass() -> f32` | 总质量 |
| `is_awake() -> bool` | 是否有部分醒着 |
| `wake()` | 唤醒全部部分 |
| `bounds() -> AABB` | 所有部分的包围盒 |

三种驱动方式的区别很重要：

- `set_pose` 是**瞬移**，会破坏连续性，适合初始化或传送。
- `drive_to_pose_using_motors` 保留物理性：身体仍会撞到东西、仍然受重力，只是关节主动追目标。
- `drive_to_pose_using_kinematics` 直接把速度设成"本步能到位姿"的值，身体不再有真实的动态响应。

## 完整示例

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Quat, Transform, Vec3};
use natsel::simulation::World;
use natsel::skeleton::ragdoll::{
    Ragdoll, RagdollJointKind, RagdollPart, RagdollSelfCollision, RagdollSettings, RagdollShape,
};
use natsel::skeleton::skeleton::{Skeleton, SkeletonPose};
use natsel::types::body_def::BodyDef;
use natsel::types::geometry::Capsule;
use natsel::types::shape_def::ShapeDef;

let mut world = World::default();
world.gravity = Vec3::new(0.0, -9.81, 0.0);

// 地面
let ground = world.create_body(&BodyDef::default());
let _ = world.create_hull_shape(
    ground,
    &ShapeDef::default(),
    &make_box_hull(20.0, 0.5, 20.0),
);

// 1. 三关节骨架：根 -> 中段 -> 末段
let mut skeleton = Skeleton::new();
skeleton.add_joint("root", -1);
skeleton.add_joint_with_parent_name("mid", "root");
skeleton.add_joint_with_parent_name("tip", "mid");

// 2. 绑定姿态：沿 x 轴排开
let mut bind = SkeletonPose::new(skeleton.clone());
bind.root_offset = Pos::new(0.0, 2.0, 0.0);
let set = |pose: &mut SkeletonPose, name: &str, p: Vec3| {
    let i = skeleton.joint_index(name).unwrap() as usize;
    pose.joint_matrices[i] = Transform::new(p, Quat::IDENTITY);
};
set(&mut bind, "root", Vec3::new(0.0, 0.0, 0.0));
set(&mut bind, "mid", Vec3::new(0.5, 0.0, 0.0));
set(&mut bind, "tip", Vec3::new(1.0, 0.0, 0.0));
bind.calculate_joint_states();

// 3. 每个关节一个胶囊部分
let mut parts: Vec<RagdollPart> = Vec::new();
for joint in skeleton.joints() {
    let mut part = RagdollPart::new();
    part.shapes = vec![RagdollShape::Capsule(Capsule::new(
        Vec3::new(0.08, 0.0, 0.0),
        Vec3::new(0.42, 0.0, 0.0),
        0.08,
    ))];
    part.joint = RagdollJointKind::SwingTwist {
        cone_angle: 0.5,
        twist_min: -0.3,
        twist_max: 0.3,
    };
    parts.push(part);
}

let mut settings = RagdollSettings::new(skeleton.clone(), parts);
settings.self_collision = RagdollSelfCollision::SkipConnected;

// 4. Ragdoll 同样独占 world 的可变借用
let world_ptr: *mut World = &mut world;
// SAFETY: world 在 ragdoll 全部使用期间保持存活；角色与世界的访问从不重叠。
let mut ragdoll = unsafe { Ragdoll::new(&mut *world_ptr, settings, &bind, None) }.expect("ragdoll");

ragdoll.apply_linear_impulse(Vec3::new(8.0, 0.0, 0.0));

let dt: f32 = 1.0 / 60.0;
for _ in 0..240 {
    // SAFETY: 此刻 ragdoll 没有进行中的世界访问。
    unsafe { (*world_ptr).step(dt, 4) };
}

let bounds = ragdoll.bounds();
println!("resting between y = {} and {}", bounds.lower_bound.y, bounds.upper_bound.y);

// 读取当前姿态
let pose = ragdoll.pose();
for i in 0..skeleton.joint_count() {
    println!("joint {i} at {:?}", pose.world_transform(i as i32).p);
}

// 世界要继续用时先销毁布娃娃
ragdoll.destroy();
```

与 `CharacterVirtual` 一样，`Ragdoll<'w>` 也会在你需要"驱动布娃娃然后推进世界"之间制造借用冲突。作用域形式在单帧内有效：

```rust
let dt: f32 = 1.0 / 60.0;
{
    let mut ragdoll = Ragdoll::new(&mut world, settings.clone(), &bind, None).expect("ragdoll");
    ragdoll.drive_to_pose_using_kinematics(&bind, dt);
} // 借用结束
world.step(dt, 4);
```

但这种写法每帧重建整只布娃娃，开销很大。持续存在的布娃娃应当用指针模式。

### 马达驱动

```rust
// 目标姿态：把 mid 关节转 45 度
let mut target = bind.clone();
target.joint_states[1].rotation =
    natsel::math::functions::make_quat_from_axis_angle(Vec3::AXIS_Y, 0.8);
target.calculate_joint_matrices();

// 注意：ragdoll 活着时不能写 world.step(...)，借用会冲突。
// 用裸指针分时访问，与上面的主示例同一套写法。
let world_ptr: *mut World = &mut world;

for _ in 0..120 {
    ragdoll.drive_to_pose_using_motors(&target, 500.0, 8.0, 1.0);
    // SAFETY: 此刻 ragdoll 没有进行中的世界访问，两个借用不重叠。
    unsafe { (*world_ptr).step(1.0 / 60.0, 4) };
}

// 想恢复纯物理行为时关掉马达
ragdoll.disable_motors();
```

这一步正是最容易卡住的地方：`drive_to_pose_using_motors` 之后**不能**紧跟一句 `world.step`，因为 `Ragdoll` 还持有 `&mut world`。要么像上面这样用指针，要么把整个驱动 + 步进序列都放进同一个持有 ragdoll 的作用域里。

`max_torque` 是每一轴的最大马达力，`hertz` 与 `damping_ratio` 控制关节的软硬。常用起点是 `hertz = 8.0`、`damping_ratio = 1.0`，三者都没有默认值。

## 骨骼映射

`SkeletonMapper` 在低精度骨架（通常是布娃娃）与高精度骨架（通常是动画）之间搬运姿态。两套骨架中**同名**的关节通过中性姿态下的偏移量建立映射；高精度骨架里位于两个已映射关节之间的关节组成"链"，保留其动画局部变换，并整体绕链起点旋转，使链末端对齐已映射的终点关节。

| 方法 | 说明 |
|------|------|
| `SkeletonMapper::new(low, low_neutral, high, high_neutral) -> Option<Self>` | 两个中性姿态必须是模型空间关节矩阵；没有任何同名关节时返回 `None` |
| `mappings() -> &[Mapping]` | 映射表 |
| `chains() -> &[Chain]` | 高精度骨架中的链 |
| `unmapped() -> &[i32]` | 既未映射也不在链中的高精度关节 |
| `map(&low_pose, &animated, &mut high_pose)` | 低精度 → 高精度 |
| `map_reverse(&high_pose, &mut low_pose)` | 高精度 → 低精度 |

`Mapping { low_index, high_index, offset }`，其中 `offset` 是中性姿态下高精度关节相对低精度关节的变换。`Chain { high_indices }` 是从映射起点到映射终点的高精度关节列表（两端都包含）。

`map` 会重算 `high_pose` 的关节状态；`map_reverse` 会让没有匹配的低精度关节保持其相对父关节的变换。

常见用法是：动画驱动高精度骨架，`map_reverse` 把结果压到布娃娃的低精度骨架上；或者布娃娃被物理扰动后，`map` 把姿态抬回高精度骨架用于渲染。

## 注意事项

- **借用与生命周期是首要约束**：`Ragdoll<'w>` 持有 `&'w mut World`，与 `CharacterVirtual` 完全一致。布娃娃存活期间无法通过 `&mut world` 调用 `step`。持续存在的布娃娃只能走指针模式或把世界访问全部挤进同一个作用域。
- **构造函数是可失败的**：`Ragdoll::new` 返回 `Option`，部分数量、姿态关节数、骨架关节数三者必须一致且非零。
- **`Ragdoll::new` 会消耗 `settings`**：需要复用配置时先 `clone()`，这也是示例里 `settings.clone()` 的原因。
- **`destroy()` 要显式调用**：世界比布娃娃活得久时，离开前销毁刚体，否则会有一堆孤儿刚体继续参与模拟。
- **`joint_frame` 的 x 轴是扭转轴**：`RagdollJointKind` 里所有角度都围绕 x 轴，摆动的锥角也不例外。部分形状也定义在关节空间，朝向错了会导致肢体互相穿插。
- **`max_mass_ratio` 会改写质量**：设成大于 1 的值时，创建过程会夹紧父子质量比并重新缩放总质量。如果你对质量有精确要求，把它设为 `0`。
- **`self_collision` 默认跳过重叠部分**：`SkipOverlapping { margin: 0.02 }` 会为包围盒重叠的非相连部分创建过滤关节，所以 `filter_joints()` 通常非空。
- **马达与运动学驱动的物理性差异**：`drive_to_pose_using_motors` 保留动态响应，`drive_to_pose_using_kinematics` 则放弃它去换稳定性。
- **`pose()` 的开销**：它会遍历所有刚体并 `calculate_joint_states()`，不要每帧多次调用。
- **`Skeleton::add_joint` 会 panic**：父索引越界或名字重复直接 panic，不会返回错误。构造骨架时务必按"父先于子"的顺序，并使用唯一名字。

## 相关页面

- [刚体](/docs/bodies)：布娃娃的每个部分都是 `DynamicBody`。
- [关节](/docs/joints)：布娃娃内部使用六自由度关节，理解其限制与马达有助于调参。
- [形状](/docs/shapes)：`RagdollShape` 对应到普通形状创建接口。
- [虚拟角色](/docs/character)：同一个借用约束的另一处体现。
- [调试绘制](/docs/debug-draw)：`draw_joints` 与 `draw_joint_extras` 可以画出布娃娃关节。
- [快照](/docs/snapshots)：布娃娃的刚体与关节活在世界里，会随世界快照一起保存。
