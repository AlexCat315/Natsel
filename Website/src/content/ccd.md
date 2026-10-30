# 连续碰撞检测

每步物理推进的位移可能远大于物体尺寸。一颗 300 m/s 的子弹在 1/60 秒里走 5 米，而它只有 0.1 米厚——如果只检查"步末位置是否重叠"，它会在第 2 步直接出现在墙的另一侧。这就是穿透（tunneling）。

连续碰撞检测（CCD）解决这个问题：不再只看离散的两个位置，而是检查物体在这一步扫过的整段路径。

## 模型

Natsel 的 CCD 分两层：

1. **自动快速形状检测**：每步计算形状的实际位移，与该形状的最小尺寸比较。位移超过阈值（由 `safety_factor` 缩放）就做扫掠检测。这对所有动态物体默认生效。
2. **子弹标记**：`is_bullet = true` 的物体**跳过阈值判断**，每步无条件做扫掠检测。

因此默认情况下你已经受到保护，不需要为普通快速物体做任何事。子弹标记用于"绝对不允许穿透"的关键物体，代价是每步都付出扫掠开销。

CCD 生效时，求解器会在撞击时刻把物体**回退**到接触点，用剩余时间做反弹或停止。这就是为什么高速子弹会停在墙前而不是穿过去。

## 世界级开关

```rust
pub enable_continuous: bool;          // 默认 true
pub enable_dynamic_continuous: bool;  // 默认 false
pub enable_speculative: bool;         // 默认 true
```

| 字段 | 默认 | 说明 |
|------|------|------|
| `enable_continuous` | `true` | 关闭后全部 CCD 停止，高速物体将穿透 |
| `enable_dynamic_continuous` | `false` | 额外让**动态↔动态**的配对也做扫掠 |
| `enable_speculative` | `true` | 预测性接触的总闸；关闭后所有形状的预测性接触失效 |

预测性接触需要**两级开关同时开启**：世界级 `world.enable_speculative` 是总闸，形状级 `ShapeDef.enable_speculative_contact`（默认 `true`）是逐个形状的选择。只有参与接触的两个形状都开启了 `enable_speculative_contact`，且世界总闸也开着，接触才会带上预测性约束。

想整体关掉预测性接触，设置 `world.enable_speculative = false` 即可，不必逐个改形状。

对应的方法：

```rust
pub fn is_continuous_enabled(&self) -> bool
pub fn enable_continuous(&mut self, enable: bool)
```

注意字段与方法同名，方法只是个便捷包装。

`enable_dynamic_continuous` 只在 `enable_continuous` 也为 `true` 时才有效——前者是后者的补充，不是替代。

### 为什么动态↔动态默认关闭

默认的扫掠检测覆盖高速物体对**静态与运动学**物体的碰撞。两个都在飞的物体互相穿透需要双方都做扫掠，成本更高，且大多数游戏里高速物体撞的是地形和墙。需要"子弹打子弹"时再打开它。

## 单体控制

```rust
pub fn set_body_bullet(&mut self, body_id: BodyID, bullet: bool)
pub fn is_body_bullet(&self, body_id: BodyID) -> bool
pub fn is_body_fast(&self, body_id: BodyID) -> bool
```

`is_body_fast` 报告的是**本步被判定为需要扫掠**的状态，由引擎根据实际位移自动决定。可以用它做调试：如果它一直为 `true`，说明该物体每步都在触发 CCD，值得检查。

`BodyDef.is_bullet` 可在创建时直接设置：

```rust
let mut def = BodyDef::default();
def.type_ = BodyType::DynamicBody;
def.is_bullet = true;
```

## 安全系数

```rust
pub struct BodyDef { /* ... */ pub safety_factor: f32, /* ... */ }

pub fn set_body_safety_factor(&mut self, id: BodyID, safety_factor: f32)
pub fn body_safety_factor(&self, id: BodyID) -> f32
```

默认 `0.5`。它缩放"位移超过多少才需要扫掠"的判据：阈值约为 `safety_factor × 形状最小尺寸`。

- **调小**（如 `0.2`）：更早触发 CCD，更安全，开销更高
- **调大**（如 `1.0`）：只对真正极快的物体做 CCD，开销更低，但处于临界速度的物体可能穿透

改变它会改变状态哈希，因此录制回放的两端必须一致。

## 速度上限

```rust
pub max_linear_speed: f32  // 默认 400.0
```

超过这个速度的刚体被钳制。这是一个安全网：极端速度会让物理数值失去意义（也更容易穿透）。默认 400 m/s 已远高于常规游戏需求。

## 读取 CCD 冲击

```rust
pub fn continuous_impacts(&self) -> &[ContactHitEvent]
```

返回本步由 CCD 解决的瞬时冲击。与普通接触事件的关键区别：**它包含在持久接触建立前就已解决的冲击**。子弹撞墙后立即弹开时，接触流形可能从未稳定存在，普通 `contact_events()` 看不到任何东西，而 `continuous_impacts()` 能看到。

它由 `enable_contact_events` 或 `enable_hit_events` 任一开启触发，**不受 `hit_event_threshold` 过滤**，每步重置。

## 完整示例

下面的程序以 200 m/s 发射一颗半径 0.1 米的子弹射向 0.05 米厚的墙。没有 CCD 时它一定穿透。

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::Sphere;
use natsel::types::shape_def::ShapeDef;

fn main() {
    let mut world = World::default();
    world.enable_sleep = false;

    // 全局 CCD（默认就是 true，此处显式写出便于理解）
    world.enable_continuous = true;

    // 一面很薄的静态墙，位于 x = 0
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let wall = world.create_body(&def);
    let _ = world.create_hull_shape(
        wall,
        &ShapeDef::default(),
        &make_box_hull(0.05, 5.0, 5.0),
    );

    // 子弹：高速、无重力、标记为 bullet
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.gravity_scale = 0.0;
    def.position = Pos::new(5.0, 0.0, 0.0);
    def.linear_velocity = Vec3::new(-200.0, 0.0, 0.0);
    def.is_bullet = true;
    def.safety_factor = 0.5;
    let bullet = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.enable_hit_events = true;        // 让 CCD 冲击事件可读
    shape_def.enable_speculative_contact = true;
    let _ = world.create_sphere_shape(bullet, &shape_def, Sphere::new(Vec3::ZERO, 0.1));

    println!("是否子弹: {}", world.is_body_bullet(bullet));
    println!("速度上限: {}", world.max_linear_speed);

    for step in 0..120 {
        world.step(1.0 / 60.0, 4);

        // CCD 冲击：不受 hit_event_threshold 过滤
        for impact in world.continuous_impacts() {
            println!(
                "step {}: CCD 冲击，接近速度 {:.1}，点 {:?}",
                step, impact.approach_speed, impact.point
            );
        }

        let x = world.body_transform(bullet).p.x;
        if x < -1.0 {
            println!("step {}: 已穿过墙，x = {:.2}", step, x);
            break;
        }
    }

    let final_x = world.body_transform(bullet).p.x;
    println!("最终 x = {:.3}（应停在墙前约 0.15）", final_x);

    // 运行时切换
    world.set_body_bullet(bullet, false);
    println!("关闭 bullet 后: {}", world.is_body_bullet(bullet));

    world.set_body_safety_factor(bullet, 0.8);
    println!("安全系数: {}", world.body_safety_factor(bullet));
}
```

示例输出中子弹停在 `x ≈ 0.15`（墙面前表面 `0.05` 加子弹半径 `0.1`）。若把 `world.enable_continuous` 设为 `false`，同样的子弹会出现在 `x = -595` 附近——完全穿透。

## 常见问题

**关闭 `enable_continuous` 后一切都会穿透。** 这是全局开关，会影响所有物体，包括标记了 `is_bullet` 的。做性能对比时若发现"子弹穿墙了"，先检查这个字段。

**`is_bullet` 不是必须的。** 普通高速物体也会被自动检测。子弹标记只是免除阈值判断、无条件扫掠。滥用它会让每步都付出扫掠成本。

**两个高速动态物体互相穿透，默认不会避免。** 需要设置 `world.enable_dynamic_continuous = true`，同时 `enable_continuous` 保持 `true`。

**`continuous_impacts` 每步重置。** 在下一次 `step` 前读取。它不受 `hit_event_threshold` 影响，所以能看到细微撞击；若只想处理明显的撞击，自行按 `approach_speed` 过滤。

**改变 `safety_factor` 会改变状态哈希。** 录制与回放两端的这个值必须一致，否则回放会偏离。见[确定性](/docs/determinism)。

**CCD 不适用于网格与传感器。** 网格形状和高度场不做连续检测（它们自身作为被撞对象是可以的）；传感器形状也不参与 CCD。高速物体撞薄网格地形时仍可能穿透，此时应加厚地形或降低速度上限。

**`max_linear_speed` 会静默钳制速度。** 施加巨大冲量后速度没有增加，通常是撞上了这个上限（默认 400）。需要更高的速度就调大它，同时考虑穿透风险。

## 相关页面

- [刚体](/docs/bodies) —— `is_bullet`、`safety_factor` 等 `BodyDef` 字段
- [事件与传感器](/docs/events) —— `ContactHitEvent` 与命中阈值
- [空间查询](/docs/queries) —— 手动做形状扫掠
- [确定性](/docs/determinism) —— CCD 与状态哈希
- [调试绘制](/docs/debug-draw) —— 可视化扫掠结果
