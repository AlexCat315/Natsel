# 快照与状态哈希

快照让你把整个物理世界保存下来，之后原样恢复；状态哈希让你用一位数字证明两次运行真的完全一致。两者配合，是回滚网络同步、录像回放、回归测试的基础。

## 快照能做什么

`save_state()` 捕获所有会影响后续模拟的状态：

- 全部刚体、形状、质量属性、变换与速度
- 接触流形、接触点、warm start 冲量
- 岛屿、约束图着色、求解器集合
- 所有关节（含 6DOF、路径关节的路径数据）
- 传感器、车辆、软体
- 全部事件缓冲（含尚未发布的结束事件）
- 影响结果的模拟设置（重力、接触调优、睡眠/CCD 开关等）

恢复之后再推进，**得到的结果与当初保存后直接推进逐位相同**，即使期间创建或销毁过刚体、形状、关节。

## 保存与恢复

```rust
use natsel::simulation::World;

let mut world = World::default();

// ... 建立场景 ...

let snapshot = world.save_state();
let memory = snapshot.byte_count();   // 估算快照自身占用，用于预算

// 推进若干步
for _ in 0..60 {
    world.step(1.0 / 60.0, 4);
}

// 回到保存点
if world.restore_state(&snapshot) {
    println!("已回滚到保存点");
}
```

两个方法的签名：

| 方法 | 签名 | 说明 |
|------|------|------|
| `save_state` | `fn save_state(&self) -> WorldState` | 只读，不修改世界 |
| `restore_state` | `fn restore_state(&mut self, state: &WorldState) -> bool` | 成功返回 `true` |

`WorldState` 自身实现 `Drop`，在离开作用域时释放它持有的副本。`byte_count()` 返回 `usize`，是快照自身副本的近似堆占用。

## 失败条件

`restore_state` 会返回 `false`，原因只有两种：

- **快照属于另一个世界**。快照记录了保存它的世界身份，恢复时必须对同一个世界进行。跨世界恢复被拒绝。
- **世界正处于锁定状态**，即在 `step` 之中或某个回调之内。

同样，`save_state` 在锁定状态下会触发断言失败。实践规则只有一条：

> 在两次 `step` 之间保存和恢复，不要在回调里做。

```rust
// 正确
world.step(dt, 4);
let s = world.save_state();

// 错误：在自定义过滤器或预求解回调里调用 save_state 会断言失败
world.set_custom_filter(Some(std::sync::Arc::new(|_a, _b| {
    // let s = world.save_state();  // 编译不过（借用冲突），且语义上非法
    true
})));
```

注意 `restore_state` 需要 `&mut self`，而回调持有的是 `&self`，所以借用检查器会先拦住你——这是好事。

## 恢复后 id 的有效性

快照会同时恢复 id 池，因此：

- **在保存之后创建的 id 会失效**。它的槽位不存在了，`is_valid` 类查询返回 `false`。
- **在保存之后销毁的 id 会重新有效**。它回到了保存时的状态。
- 保存时就存在的 id **保持不变**，可以继续使用。

```rust
let before = world.save_state();
let extra = world.create_body(&natsel::types::body_def::BodyDef::default());

world.restore_state(&before);

// extra 是在保存之后创建的，现在已失效
assert!(!world.body_is_valid(extra));
```

这意味着你的游戏层对象表必须和快照一起回滚。典型做法是让实体 id（你的 ECS id）与 `BodyID` 分开管理，回滚时按 `step` 序号重新映射，或者干脆把“保存点之后的创建/销毁”记录成一份可撤销的操作日志。

## 什么不在快照里

以下内容有意不包含，恢复后不会改变：

| 项目 | 原因 |
|------|------|
| 形状几何对象（凸包、网格、高度场、复合、约束路径） | 按引用共享且不可就地修改，不需要复制 |
| 你注册的回调（过滤器、预求解、摩擦/弹性混合） | 属于宿主程序，不属于世界状态 |
| `user_data` 指针指向的对象 | 指针值本身会被恢复，但指向的内存由你管理 |
| `worker_count` | 结果与线程数无关，不需要回滚 |
| `CharacterVirtual` 控制器 | 它在世界之外，有自己的 `save_state` / `restore_state` |

角色控制器要单独存：

```rust
use natsel::character::character_virtual::{CharacterDef, CharacterVirtual};

let mut world = natsel::simulation::World::default();
let mut character = CharacterVirtual::new(&mut world, &CharacterDef::default()).unwrap();

let world_state = world.save_state();
let character_state = character.save_state();

// 回滚时两者都要恢复
world.restore_state(&world_state);
character.restore_state(character_state);
```

顺序很重要：先恢复世界（角色借用了它），再恢复角色状态。

`Ragdoll` 和车辆没有独立状态。前者的刚体与关节都在世界里，后者的状态记录在 `World` 内部，随世界快照一起回滚。

## 内存预算

每个快照都是完整副本，内存随场景规模线性增长。常见量级：

```rust
let snapshot = world.save_state();
println!("快照约 {} KB", snapshot.byte_count() / 1024);
```

回滚网络同步通常只需保留最近若干帧。用环形缓冲：

```rust
use std::collections::VecDeque;
use natsel::simulation::world_state::WorldState;

let mut history: VecDeque<WorldState> = VecDeque::with_capacity(32);
let mut total_bytes = 0usize;

let snapshot = world.save_state();
total_bytes += snapshot.byte_count();
history.push_back(snapshot);

// 超出预算就丢掉最旧的
while total_bytes > 8 * 1024 * 1024 && history.len() > 1 {
    if let Some(old) = history.pop_front() {
        total_bytes -= old.byte_count();
    }
}
```

注意 `WorldState` 不可 `Clone`，每个保存点必须单独保存，不能靠复制共享。

## 状态哈希

`world_state_hash()` 返回一个 `u64`，由每个活跃刚体的变换（醒着的刚体还包括速度）混合而成：

```rust
let hash = world.world_state_hash();
```

性质：

- **对位置极其敏感**。`f64` 位置变化一个最小可表示单位，哈希就会改变。
- **成本低**。只遍历刚体，不涉及接触和形状。
- **空世界有固定值**。没有刚体时返回 FNV-1a 的初始值，可以用作基线断言。

它**不是**完整状态校验：不覆盖接触流形、冲量、睡眠计时器。要逐项对比分叉原因，用 `body_state_dump()` 和 `contact_dump()`，它们把每个量的原始位输出成十六进制字符串。

## 用哈希验证确定性

最实用的用法是两个世界同步推进、逐步比对：

```rust
use natsel::collision::hull::make_box_hull;
use natsel::math::types::Pos;
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;

fn build() -> World {
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(20.0, 1.0, 20.0));

    for i in 0..5 {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(i as f64 * 1.1, 5.0, 0.0);
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));
    }
    world
}

let mut a = build();
let mut b = build();

assert_eq!(a.world_state_hash(), b.world_state_hash());

for step in 0..300 {
    a.step(1.0 / 60.0, 4);
    b.step(1.0 / 60.0, 4);
    assert_eq!(
        a.world_state_hash(),
        b.world_state_hash(),
        "第 {step} 步出现分叉"
    );
}
```

这个模式可以直接用来验证“改变线程数不影响结果”：

```rust
let mut serial = build();      // worker_count 默认 1
let mut parallel = build();
parallel.worker_count = 8;     // 8 个工作者

for _ in 0..300 {
    serial.step(1.0 / 60.0, 4);
    parallel.step(1.0 / 60.0, 4);
    assert_eq!(serial.world_state_hash(), parallel.world_state_hash());
}
```

如果这两个哈希不同，那是引擎的 bug，不是你的用法问题——引擎的确定性承诺包含“worker 数无关”。

## 用哈希验证快照往返

保存 → 推进 → 恢复 → 再推进，结果必须与第一次完全相同：

```rust
let mut world = build();
let saved = world.save_state();

for _ in 0..120 {
    world.step(1.0 / 60.0, 4);
}
let first_run_hash = world.world_state_hash();

assert!(world.restore_state(&saved));
for _ in 0..120 {
    world.step(1.0 / 60.0, 4);
}
assert_eq!(world.world_state_hash(), first_run_hash);
```

这是回滚系统正确性的最小验证，也是引擎自身测试所采用的方式。它同时证明了快照覆盖完整——漏掉任何一个影响结果的字段，第二次推进就会分叉。

## 常见用途

| 场景 | 做法 |
|------|------|
| 回滚网络同步 | 每帧保存快照，收到远端输入后回滚重放。只在本地输入变化时重算 |
| 录像与回放 | 保存初始快照 + 每帧输入序列，回放时重放输入 |
| 编辑器撤销 | 修改前保存，撤销时恢复；比重建场景快得多 |
| 确定性回归测试 | 固定场景 + 固定步数 + 哈希断言 |
| 分叉定位 | 两个哈希一旦不同，用 `body_state_dump()` 找出第一个不同的刚体 |

## 性能提示

- 快照是**深拷贝**，代价与场景规模成正比。每帧保存只适合中小场景。
- 大场景优先考虑“关键帧 + 输入日志”压缩，而不是逐帧快照。
- 恢复同样要重建接触与宽相结构，比保存略贵。
- `byte_count()` 只统计快照自身副本，不含共享的几何对象——共享部分不计入你的预算。

## 下一步

- [确定性](/docs/determinism)：哈希背后的求值顺序规则。
- [多线程](/docs/multithreading)：`worker_count` 与结果无关的原因。
- [常见问题](/docs/faq)：`restore_state` 返回 `false` 时的排查。
