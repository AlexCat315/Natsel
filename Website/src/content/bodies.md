# 刚体

刚体（Body）是 Natsel 中最基本的运动单元：它是一个可以平移和旋转的坐标框架，持有速度、质量、阻尼、睡眠状态等运动学与动力学数据。刚体本身**不包含碰撞几何** —— 碰撞形状是挂在它上面的独立对象。

这种拆分让"一个物体多个形状"成为默认能力，也让形状可以被单独销毁重建而不影响刚体的运动状态。

## 三种刚体类型

`BodyType` 决定一个刚体如何被求解器对待：

| 类型 | 质量 | 速度来源 | 被求解器移动 | 典型用途 |
|------|------|----------|--------------|----------|
| `BodyType::StaticBody` | 0 | 恒为 0 | 否 | 地面、墙壁、静态场景 |
| `BodyType::KinematicBody` | 0 | 由用户设置 | 是（按设定速度） | 移动平台、电梯、玩家的运动学代理 |
| `BodyType::DynamicBody` | 由形状密度积分 | 由力、冲量与接触决定 | 是 | 一切受物理影响的物体 |

```rust
use natsel::types::body_def::{BodyDef, BodyType};

let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;   // 注意字段名是 type_，不是 type
```

`BodyType` 的默认值是 `StaticBody`（枚举上标了 `#[default]`）。这是一个容易踩的坑：`BodyDef::default()` 得到的是静态体，忘了改 `type_` 就会出现一个永远不会动的物体。

静态体与运动学体的 `inv_mass` 都是 0，因此求解器施加给它们的冲量无效。区别在于运动学体仍按你设置的 `linear_velocity` / `angular_velocity` 推进位置，静态体则完全不动。

### 运行时切换类型

```rust
use natsel::types::body_def::BodyType;

// 把一个已存在的动态刚体改成运动学体：之后它按你设置的 linear_velocity 移动，
// 不再受重力与接触冲量影响。
world.set_body_type(body, BodyType::KinematicBody);

// 查询当前类型
assert_eq!(world.body_type(body), BodyType::KinematicBody);
```

切换类型会重建质量、重算惯性、迁移求解器集合，并把物体从旧的岛屿里摘出来。有一个限制：**带复合形状或高度场的刚体不能改成非静态类型**。`set_body_type` 检测到这种组合会直接返回，不做任何修改。

## BodyDef 字段参考

`BodyDef` 是创建刚体的唯一输入。下表是全部字段与默认值（默认值取自 `impl Default for BodyDef`）：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `type_` | `BodyType` | `StaticBody` | 刚体类型。**必须显式改成 `DynamicBody` 才会下落** |
| `position` | `Pos` | `Pos::ZERO` | 世界坐标（f64） |
| `rotation` | `Quat` | `Quat::IDENTITY` | 朝向。默认值是单位四元数 |
| `linear_velocity` | `Vec3` | `Vec3::ZERO` | 初始线速度 |
| `angular_velocity` | `Vec3` | `Vec3::ZERO` | 初始角速度 |
| `linear_damping` | `f32` | `0.0` | 线性阻尼系数，非负 |
| `angular_damping` | `f32` | `0.0` | 角阻尼系数，非负 |
| `gravity_scale` | `f32` | `1.0` | 重力倍率；0 表示失重 |
| `sleep_threshold` | `f32` | `0.05 * 长度单位` | 低于该速度即可进入睡眠 |
| `safety_factor` | `f32` | `0.5` | 连续碰撞（CCD）触发比例 |
| `name` | `Option<String>` | `None` | 调试用名字，存入世界的名字缓存 |
| `user_data` | `Option<*mut c_void>` | `None` | 透传给事件回调的裸指针 |
| `motion_locks` | `MotionLocks` | 全 `false` | 逐轴锁定 |
| `enable_sleep` | `bool` | `true` | 是否允许该刚体睡眠 |
| `is_awake` | `bool` | `true` | 是否初始唤醒 |
| `is_bullet` | `bool` | `false` | 是否每步做连续碰撞检测 |
| `is_enabled` | `bool` | `true` | 是否初始启用 |
| `allow_fast_rotation` | `bool` | `false` | 允许每子步旋转超过 90°，避免快速旋转体穿透 |
| `enable_contact_recycling` | `bool` | `true` | 是否复用上一帧接触流形 |

`sleep_threshold` 的默认值随长度单位缩放：长度单位改变后，`0.05 * length_units_per_meter()` 会相应变化。见 [单位](/docs/units)。

`rotation` 必须传 `Quat::IDENTITY`。`Quat` 派生的 `Default` 是全零四元数（`v` 与 `s` 都是 0），模长为 0，不是合法旋转；`create_body` 会拒绝它并触发断言。

## 创建与销毁

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::Sphere;
use natsel::types::shape_def::ShapeDef;

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -20.0, 0.0);

    // 静态地面
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(20.0, 0.5, 20.0));

    // 动态球
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 3.0, 0.0);
    def.name = Some("ball".to_string());
    let ball = world.create_body(&def);
    let _ = world.create_sphere_shape(ball, &ShapeDef::default(), Sphere::new(Vec3::ZERO, 0.5));

    for _ in 0..120 {
        world.step(1.0f32 / 60.0, 4);
    }
    println!("{:?}", world.body_position(ball));

    world.destroy_body(ball);
    assert!(!world.body_is_valid(ball));
}
```

`create_body` 返回 `BodyID`，它是 `{ index1: i32, world0: u16, generation: u16 }`。之后所有针对该刚体的操作都要传这个 ID。销毁后 ID 失效，`body_is_valid` 返回 `false`；索引槽会被复用，但 generation 递增，所以陈旧 ID 不会被误认为新刚体。

`destroy_body` 会连带销毁刚体上的所有形状（并从凸包缓存里释放引用）、接触、关节，并把其他受影响刚体唤醒。

## 运动锁

`MotionLocks` 逐轴禁止平移或旋转，常用于角色控制器、2.5D 游戏，或者"只允许绕 Y 轴转"的载具：

```rust
use natsel::types::body_def::MotionLocks;

// 参数顺序：(linear_x, linear_y, linear_z, angular_x, angular_y, angular_z)
let locks = MotionLocks::new(false, false, false, true, false, true);
def.motion_locks = locks;
```

上面的配置锁住了 X 轴与 Z 轴旋转，物体只能绕 Y 轴自转 —— 典型的 2.5D 角色设置。

运行时修改：

```rust
world.set_body_motion_locks(ball, &MotionLocks::new(false, false, false, false, false, false));
let current = world.body_motion_locks(ball);
```

锁是按轴生效的：被锁的轴既不积累速度，也不响应接触求解器施加的冲量。解锁会使物体唤醒，因为用户能观察到状态变化。

一个常见组合是 `linear_y = true` 做"重力关掉但仍然能水平碰撞"的效果，比 `gravity_scale = 0` 更可控：后者只去掉重力，物体仍可被撞飞上天。

## 阻尼

阻尼采用双曲衰减而不是指数衰减，实际效果是**衰减量只取决于经过的总时长，与子步数无关**。改 `sub_step_count` 不会改变一个物体最终停在哪里。

```rust
def.linear_damping = 0.1;
def.angular_damping = 0.2;
```

经验值：

| 阻尼 | 效果 |
|------|------|
| `0.0` | 无阻尼，真空中抛物线 |
| `0.1` ~ `0.5` | 轻微空气阻力，适合多数学场景 |
| `1.0` ~ `5.0` | 明显减速，适合水下或粘稠介质 |
| 很大（> 100） | 近乎立即停止，通常不如直接设速度 |

角阻尼常用于让方块不要无限滚动；对球体想让它停下而不是永远滚，用 `rolling_resistance` 更符合物理（见 [材质与过滤](/docs/materials)）。

运行时读写：

```rust
world.set_body_linear_damping(ball, 0.3);
world.set_body_angular_damping(ball, 0.4);
let ld = world.body_linear_damping(ball);
let ad = world.body_angular_damping(ball);
```

## 重力倍率

`gravity_scale` 缩放世界重力对单个刚体的作用：

```rust
world.gravity = Vec3::new(0.0, -10.0, 0.0);

def.gravity_scale = 0.0;   // 失重，像是漂浮在太空
def.gravity_scale = 0.5;   // 半重力，像是月球
def.gravity_scale = -1.0;  // 反重力，向上飘
```

重要细节：**`gravity_scale` 只在 `inv_mass > 0` 时生效**。静态体与运动学体的 `inv_mass` 为 0，引擎会把它们的有效重力倍率强制视为 0。所以你不需要给静止平台设 `gravity_scale = 0`。

```rust
world.set_body_gravity_scale(ball, 0.5);
let gs = world.body_gravity_scale(ball);
```

## 睡眠

睡眠是性能优化的核心机制：静止的刚体不再参与求解，只在被接触、冲量或显式唤醒时恢复。

判定逻辑：引擎每一步会算出刚体的"睡眠速度"，它同时考虑线速度和物体最远点的旋转弧长 —— 所以一个原地高速自转的物体不会被误判为静止。当它低于 `sleep_threshold` 时开始累积静止时间；连续 0.5 秒都低于阈值，且该物体所在的整个接触组都满足条件，它们一起进入睡眠。

```rust
def.enable_sleep = false;   // 该刚体永不睡眠
def.is_awake = true;        // 初始唤醒

world.enable_body_sleep(ball, false);
world.set_body_sleep_threshold(ball, 0.1);
let awake = world.body_is_awake(ball);
```

要点：

- 睡眠是**成组**进行的。一个刚体保持清醒会让所有与它接触相连的刚体保持清醒，所以悬空的方块链不会一半睡一半醒。
- `world.enable_sleep = false` 关闭全局睡眠，常用于调试，因为睡眠会掩盖"物体其实在缓慢漂移"的问题。
- 场景初始化时可以全部 `is_awake = false`，让它们从睡眠开始，省掉第一帧的计算。
- 从外部修改变换或速度后，如果物体本该响应却不动，检查它是不是睡着 —— 用 `world.set_body_awake(ball, true)` 显式唤醒。

## 质量属性

质量不是直接设置的，而是由形状的密度积分得到。挂上形状时（`update_body_mass = true`）自动重算。

```rust
let mass = world.body_mass(ball);
let inv = world.body_inverse_mass(ball);
let md = world.body_mass_data(ball);   // MassData { mass, center, inertia }

world.apply_mass_from_shapes(ball);    // 强制按当前形状重算
world.set_body_mass_data(ball, md);    // 手动覆盖
```

`MassData` 有三个字段：

| 字段 | 类型 | 含义 |
|------|------|------|
| `mass` | `f32` | 总质量 |
| `center` | `Vec3` | 相对刚体原点的质心 |
| `inertia` | `Matrix3` | 绕质心的转动惯量 |

手动设置 `set_body_mass_data` 会同步更新引擎从质量推导出的所有量（逆惯性、质心位置、接触有效性），所以它是"我有解析解的质量属性"这种场景的入口。一般情况应该让引擎从形状算。

其他质量相关查询：

```rust
let local_c = world.body_local_center(ball);         // Vec3，局部质心
let world_c = world.body_world_center(ball);         // Pos，世界质心
let inertia = world.body_local_rotational_inertia(ball);
let inv_i = world.body_world_inverse_rotational_inertia(ball);
```

注意 `body_position` 返回的是**刚体原点**，不是质心。质心可能因为形状不对称而偏离原点，`body_world_center` 才是质心位置。

## 力与冲量

两者语义完全不同，混淆是常见 bug 来源。

### 力（force）

力在一段时间内持续作用。`apply_body_force` 把力累加到内部累加器，在下一个子步的积分中产生加速度。

```rust
// 在质心施加力，无扭矩
world.apply_body_force_to_center(ball, Vec3::new(10.0, 0.0, 0.0), true);

// 在世界点施加力，会产生扭矩
world.apply_body_force(ball, Vec3::new(10.0, 0.0, 0.0), Pos::new(0.0, 4.0, 0.0), true);

// 纯扭矩
world.apply_body_torque(ball, Vec3::new(0.0, 0.0, 1.0), true);
```

**关键语义：力和扭矩的累加器在每次 `step` 结束时清零。** 因此"持续推力"必须每帧重新施加：

```rust
for _ in 0..600 {
    world.apply_body_force_to_center(rocket, Vec3::new(0.0, 300.0, 0.0), true);
    world.step(1.0f32 / 60.0, 4);
}
```

而不是在循环外施加一次 —— 那样只影响一帧。

### 冲量（impulse）

冲量是瞬时动量变化，直接改速度，不经过时间。

```rust
// 在世界点施加冲量（含角速度变化）
world.apply_body_linear_impulse(ball, Vec3::new(0.0, 5.0, 0.0), Pos::new(0.0, 3.0, 0.0), true);

// 在质心施加冲量，只有平动
world.apply_body_linear_impulse_to_center(ball, Vec3::new(2.0, 0.0, 0.0), true);

// 角冲量
world.apply_body_angular_impulse(ball, Vec3::new(0.0, 0.0, 0.5), true);
```

冲量让速度立刻改变 `impulse * inv_mass`，所以对静态体施加冲量没有效果（`inv_mass` 为 0）。

速度和冲量结果都会被 `max_linear_speed` 限制（默认 400）。超限时速度向量被裁剪到该模长，不会被丢弃方向。

### wake 参数

每个施力函数的最后一个参数是 `wake: bool`。传 `true` 时，如果刚体正在睡眠会先被唤醒；传 `false` 则睡眠中的刚体直接忽略这次施力（因为睡眠刚体不在唤醒集合里）。做爆炸、撞击这类"应该惊动周围物体"的效果时务必传 `true`。

## 高速物体与连续碰撞

快速移动的物体会在两步之间"穿过"薄物体，这叫隧道效应。两种应对：

**`is_bullet`** —— 每个子步都对该刚体做扫掠检测：

```rust
def.is_bullet = true;
world.set_body_bullet(ball, true);
let is_bullet = world.is_body_bullet(ball);
```

子弹体开销高（要做扫掠与 TOI 求解），只给真正需要的物体开。典型是子弹、投掷物、高速弹射的角色。

**`safety_factor`** —— 自动触发连续碰撞的阈值。引擎每步检查物体的最大位移，当它超过 `safety_factor * min_extent`（自身最小尺寸乘以该系数）时，该物体在当步走扫掠检测：

```rust
def.safety_factor = 0.5;
world.set_body_safety_factor(ball, 0.5);
```

调小会让更多物体触发连续碰撞（更准、更慢），调大则相反。默认 0.5 意味着位移超过自身最小尺寸的一半时启用。

此外还有 `allow_fast_rotation`：每个子步旋转超过 90° 的物体会被特殊处理，避免旋转体穿透。细长物体快速旋转时要打开。

## 启用与禁用

禁用刚体是"从模拟中彻底摘除但保留数据"，比销毁更适合对象池：

```rust
world.disable_body(ball);
assert!(!world.is_body_enabled(ball));

world.enable_body(ball);
```

`disable_body` 会销毁该刚体上的所有接触（并唤醒相邻物体，避免留下悬空物）、把关节转移到禁用集合、移除宽相代理、从岛屿中摘除。重新启用时会按类型放回静态集合或唤醒集合，并重建代理。

与"禁用"相比：

- `enable_body_sleep(x, false)` 只是不让它睡，仍在模拟中。
- `gravity_scale = 0` 只是去掉重力，仍参与碰撞。
- `disable_body` 完全不参与，性价比最高但语义最重。

## 变换与速度的读写

```rust
let p = world.body_position(ball);       // Pos
let q = world.body_rotation(ball);       // Quat
let xf = world.body_transform(ball);     // WorldTransform { p: Pos, q: Quat }
let v = world.body_linear_velocity(ball);
let w = world.body_angular_velocity(ball);

world.set_body_transform(ball, Pos::new(0.0, 10.0, 0.0), Quat::IDENTITY);
world.set_body_linear_velocity(ball, Vec3::ZERO);
world.set_body_angular_velocity(ball, Vec3::ZERO);
```

`set_body_transform` 是瞬移：它把刚体直接放到目标位置与朝向，并相应修正派生状态，所以不会因为"一瞬间跨越很长距离"被连续碰撞误判为高速运动。

局部与世界空间的转换：

```rust
let local = world.body_local_point(ball, Pos::new(1.0, 2.0, 3.0));
let world_p = world.body_world_point(ball, Vec3::ZERO);
let local_vec = world.body_local_vector(ball, Vec3::AXIS_Y);
let world_vec = world.body_world_vector(ball, Vec3::AXIS_Y);

// 某点的速度（考虑旋转）
let pv = world.body_world_point_velocity(ball, Pos::new(0.0, 3.5, 0.0));
```

这些转换在施加"在物体的某个局部点施力"时很有用：先用 `body_world_point` 把局部点转成世界点，再传给 `apply_body_force`。

## 命名与诊断

```rust
def.name = Some("player".to_string());
world.set_body_name(ball, Some("player-2"));
let name = world.body_name(ball);        // Option<&str>

let n = world.body_shape_count(ball);
let joints = world.body_joints(ball);    // Vec<JointID>
let min_extent = world.body_min_extent(ball);
let max_extent = world.body_max_extent(ball);       // Vec3
let is_fast = world.is_body_fast(ball);             // 是否在走 CCD
```

名字存在世界的名字缓存里，相同的名字共享一份存储，所以给成百上千个刚体起同样的名字几乎不占内存。

`body_joints` 返回该刚体连接的所有关节，调试层级结构时很有用。

## 完整示例：带锁的滑动方块

把上面的东西串起来 —— 一个只能沿 X 轴滑动、受阻尼、由持续推力驱动的方块：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType, MotionLocks};
use natsel::types::shape_def::ShapeDef;

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    // 地面：中心 y=0、半高 0.5，所以顶面在 y=0.5
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(30.0, 0.5, 5.0));

    // 方块：半高 0.5，中心 y=1.0，底面正好贴在 y=0.5 的地面上
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(-10.0, 1.0, 0.0);
    def.linear_damping = 0.4;
    // 锁住 Y/Z 平动与全部转动，只留 X 轴
    def.motion_locks = MotionLocks::new(false, true, true, true, true, true);
    let slider = world.create_body(&def);

    let mut sd = ShapeDef::default();
    sd.density = 800.0;
    let _ = world.create_hull_shape(slider, &sd, &make_box_hull(0.5, 0.5, 0.5));
    println!("mass = {:.1}", world.body_mass(slider));   // 800

    // 推力必须超过静摩擦上限（约 0.6 * 800 * 10 = 4800 N）方块才会滑起来
    for _ in 0..300 {
        world.apply_body_force_to_center(slider, Vec3::new(8000.0, 0.0, 0.0), true);
        world.step(1.0f32 / 60.0, 4);
    }

    let p = world.body_position(slider);
    println!("with thrust: x = {:.3} y = {:.3}", p.x, p.y);   // x 增大，y 保持 1.0

    // 撤掉推力后由阻尼让它停下
    for _ in 0..600 {
        world.step(1.0f32 / 60.0, 4);
    }
    let p2 = world.body_position(slider);
    println!("coasting:   x = {:.3} vx = {:.3}", p2.x, world.body_linear_velocity(slider).x);
}
```

因为 Y/Z 被锁死，方块不会因为地面接触的数值误差而漂离轨道，`y` 始终保持在 1.0。

这个例子还演示了一个容易踩的坑：**推力必须超过摩擦上限。** 800 kg 的方块在地面上，静摩擦上限约为 `friction * mass * |gravity| = 0.6 * 800 * 10 = 4800 N`。推力只有 2000 N 时方块完全不动 —— 它没有被卡住，只是被摩擦力按住了。要么加大推力，要么降低摩擦系数。

另外注意方块位置：地面中心 `y = 0`、半高 0.5，顶面在 `y = 0.5`。方块半高也是 0.5，所以中心必须放在 `y = 1.0` 才刚好接触。把中心放在 `y = 0.5` 会让方块嵌进地面一半，产生巨大的法向力，摩擦力随之暴涨到根本推不动 —— 这是"我施加了很大的力但物体不动"的另一个常见原因。

## 常见错误

**忘记设置 `type_`。** `BodyDef::default()` 是 `StaticBody`，不受力也不下落。这是"物体悬在空中不动"最常见的原因。

**用 `Quat::default()` 当旋转。** 它是全零四元数，`create_body` 会拒绝。

**物体嵌进地面。** 把刚体中心放在地面上而不是"地面顶面 + 自身半高"，会让形状与地面深度重叠，产生极大的法向力。表现是物体不动、或者被猛地弹出、或者穿透。计算位置时先把两侧的半高加起来。

**推力小于摩擦上限。** 静止物体需要克服 `friction * mass * |gravity|` 才会滑动。力太小的话物体纹丝不动，看起来像是力没生效。

**在循环外施加一次力就期待持续加速。** 力的累加器在每个 `step` 结束时清零，持续推力必须每帧重新施加。

**对静态体或运动学体施加冲量。** 它们的 `inv_mass` 为 0，冲量不产生任何速度变化。

**给睡眠中的物体施力但 `wake` 传了 `false`。** 睡眠物体不在唤醒集合里，那次施力会被完全忽略。

**改完速度/位置后物体不动。** 检查它是不是在睡眠。`world.set_body_awake(body, true)` 显式唤醒。

## 下一步

- [形状](/docs/shapes)：`ShapeDef` 全字段、六种形状、传感器与事件。
- [材质与过滤](/docs/materials)：摩擦与弹性如何组合。
- [连续碰撞](/docs/ccd)：子弹体与 TOI 的细节。
- [空间查询](/docs/queries)：射线、形状投射与重叠查询。
- [事件](/docs/events)：接触、命中、传感器事件如何使用 `user_data`。
