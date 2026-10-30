# 确定性

Natsel 保证：**同一版本、同一份输入、同一份配置，无论跑多少线程，结果逐位相同。**

对使用者的意义很直接——录像回放不会漂移、锁步联机不会分叉、出 bug 时能稳定复现、加线程能提速而不改变物理表现。

## 你能依赖什么

| 保证 | 你能拿它做什么 |
|------|----------------|
| 重复运行结果一致 | 稳定复现 bug，回放录像与实时演算一致 |
| `worker_count` 不影响结果 | 放心开多线程提速，不必担心改变物理表现 |
| 运行中途改 `worker_count` 也不影响 | 可以按负载动态调整线程数 |
| 保存快照再恢复，接续结果不变 | 实现回滚、录像、断线重连 |

只有一种情况会破坏确定性：**你自己喂给物理的非确定输入**。比如用当前时间做随机种子、按未排序的 `HashMap` 顺序创建物体、每帧销毁重建刚体。规避办法见文末。

## 开线程不改变结果

`worker_count` 是 `World` 的普通字段，直接赋值：

```rust
use natsel::simulation::World;

let mut world = World::default();
world.worker_count = 8;
```

不需要额外的同步或配置。想让物理结果在同一台机器上完全可复现，就应该固定这个值——不是为了正确性，而是为了让性能对比有意义。

## 用状态哈希验证

`world_state_hash()` 把整个世界的状态压成一个 `u64`。两个世界哈希相同，就意味着所有刚体的位置、旋转（以及醒着刚体的速度）完全相同。

这是最实用的确定性检查工具：

```rust
use natsel::simulation::World;

fn simulate(workers: i32) -> u64 {
    let mut world = World::default();
    world.worker_count = workers;
    // ... 搭建你的场景 ...
    for _ in 0..300 {
        world.step(1.0 / 60.0, 4);
    }
    world.world_state_hash()
}

fn main() {
    let single = simulate(1);
    let multi = simulate(8);
    assert_eq!(single, multi, "多线程改变了物理结果");
}
```

什么时候该用它：

- **升级版本后**做回归对比，确认物理行为没有意外改变。
- **改并行相关代码后**，确认 `worker_count = 1` 与 `N` 结果一致。
- **写测试时**用它代替逐字段比较，一行断言覆盖整个场景。

注意哈希只覆盖刚体的变换和（醒着刚体的）速度。它不包含接触点缓存、事件缓冲、关节内部力矩这类中间量——所以它是"最终状态指纹"，不是完整序列化。

## 快照：回滚与回放

需要真正回到某个时刻，用快照而不是哈希：

```rust
use natsel::simulation::World;

let mut world = World::default();
// ... 搭建场景 ...
for _ in 0..60 {
    world.step(1.0 / 60.0, 4);
}

let checkpoint = world.save_state();   // 存档

for _ in 0..60 {
    world.step(1.0 / 60.0, 4);
}

world.restore_state(&checkpoint);      // 回到存档点
```

`restore_state` 返回 `bool`，两种情况会失败：

- 快照来自另一个 `World`；
- 当前处于 `step` 内部或回调中（此时世界是锁定状态）。

两条使用规则：

- 在 `step` 之外、回调之外调用 `save_state` / `restore_state`。
- 存档之后新建的 id 会失效；存档之后销毁的 id 会重新生效。跨存档持有的 `BodyID` 需要重新校验。

完整的快照内容与内存估算见 [快照与状态哈希](/docs/snapshots)。

## 写确定性代码

物理引擎本身是确定的，破坏它的是调用方。以下是实践中最常见的几个坑：

**不要在 `step` 之间使用不可复现的输入。** 时间戳、系统随机数、`HashMap` 的迭代顺序都会让两次运行分叉。需要随机就用固定种子的生成器。

```rust
// 坏：每次运行都不同
let impulse = Vec3::new(rand::random(), 0.0, 0.0);

// 好：固定种子，可复现
let mut rng = MyRng::new(42);
let impulse = Vec3::new(rng.next_f32(), 0.0, 0.0);
```

**不要每帧销毁重建物体。** 创建顺序决定 id 分配，id 决定接触遍历顺序。需要重置场景时用 `restore_state`，而不是清空重来。

```rust
// 坏：重建顺序一旦变化，结果就变
*self = Scene::new();

// 好：回到存档点
world.restore_state(&self.checkpoint);
```

**每次只改一个变量。** 调整求解参数、步长、子步数之后，用上面的哈希对比确认物理表现是否符合预期。有意改变求解行为的版本升级会改变哈希——这是正常的，但要把它当成一次需要验证的行为变更，而不是意外。

**步长与子步数保持一致。** `world.step(1.0 / 60.0, 4)` 和 `world.step(1.0 / 120.0, 4)` 是两套不同的物理行为。回放和联机时必须使用与录制时相同的组合。

## 浮点一致性

引擎在标量路径与宽 SIMD 路径上保持相同的求值顺序，所以开启或关闭宽求解器（`world.enable_wide_solver`）不会改变结果。这意味着你不需要为了确定性而关闭向量化。

反过来，这也意味着：如果你在**自己的**代码里对物理状态做浮点运算（比如手动积分一个自定义系统），那部分不受引擎保护——`f32` 的加法顺序、FMA 收缩、`fast-math` 都可能引入差异。

## 相关页面

- [快照与状态哈希](/docs/snapshots) —— 存档、回滚与内存开销
- [多线程与性能](/docs/multithreading) —— `worker_count` 的调优与性能剖析
- [坐标与单位](/docs/units) —— 精度模型为什么这样设计
