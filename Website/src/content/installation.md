# 安装与集成

Natsel 是一个纯 Rust 库 crate，没有第三方依赖，也没有构建脚本或 C 依赖。集成它的工作基本上就是写对 `Cargo.toml` 的一行。

## 依赖方式

### git 依赖

上游尚未发布到 crates.io，推荐直接引用仓库：

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main" }
```

如果需要锁定到某个具体提交（推荐用于生产，避免上游 `main` 变动导致构建漂移）：

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", rev = "a1b2c3d" }
```

### 本地 path 依赖

在同一个 workspace 里开发，或需要边改引擎边调试游戏逻辑时，指针指向本地目录最方便：

```toml
[dependencies]
natsel = { path = "../Natsel" }
```

`path` 依赖不会拷贝源码，改动即时生效；把两者结合可以做到"本地用 `path`、CI 用 git"：

```toml
[dependencies]
natsel = { path = "../Natsel", optional = false }
```

### cargo add

也可以让 Cargo 写这行：

```sh
cargo add --git https://github.com/AlexCat315/Natsel natsel
```

## 工具链要求

| 项目 | 要求 |
|------|------|
| Rust 版本 | 1.85 及以上（`rust-version = "1.85"`） |
| edition | 2024 |
| 外部依赖 | 无 |
| 平台 API | 仅用 `std`，不需要链接任何 C 库 |

crate 的 `rust-version` 是 1.85，因为 edition 2024 从该版本开始稳定。低于这个版本 Cargo 会直接拒绝构建，而不是给出难懂的编译错误。

```sh
rustup update stable
rustc --version   # 需要 >= 1.85
```

引擎本身不需要 nightly，也不需要任何 feature flag 才能编译。

## 特性

只有一个可选特性：

```toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main", features = ["validate"] }
```

`validate` 打开额外的内部一致性校验。这些校验只在调试断言同时开启时生效，也就是 `cargo build` 或 `cargo test` 的默认 profile，而在 `--release` 下完全编译掉。

它检查的是引擎自身的不变量，例如序列化布局、岛屿划分一致性、接触流形合法区间。代价是明显变慢，所以：

- 开发与测试阶段打开，能更早暴露引擎或调用方的错误用法。
- 发布构建不要打开：release 下这些校验本来就被编译掉，打开也只是无意义的配置。

注意这两个条件必须同时满足。单独用 `cargo build --release --features validate` 不会得到任何额外检查。

## 构建与测试

```sh
cargo build --release --all-targets
cargo test
```

第一条会连同 `benchmarks` 二进制一起构建，这也是 CI 实际执行的命令。`cargo test` 运行 crate 内所有 `#[cfg(test)]` 模块。

带校验断言跑测试：

```sh
cargo test --features validate
```

运行基准（需要 release，否则数字没有意义）：

```sh
cargo run --release --bin benchmarks -- --all -r=1
```

`-r=1` 是重复次数；省略参数会跑默认的两组场景。

## 支持的平台

CI 在三个平台上跑完整的构建与测试：

| 平台 | runner |
|------|--------|
| Linux | `ubuntu-latest` |
| Windows | `windows-latest` |
| macOS | `macos-latest` |

因为只依赖 `std`，其他提供标准库的 target（例如各架构的 Linux、`wasm32` 之外的嵌入式目标若提供 `std` 与分配器）原则上也能编译。但确定性只在上述三平台上有回归测试覆盖，跨平台二进制级别的浮点一致性不做承诺 —— 详见 [确定性](/docs/determinism)。

## 版本信息

运行时可读取版本号与精度模式：

```rust
use natsel::{get_version, is_double_precision, get_max_manifold_points};

fn main() {
    let v = get_version();
    println!("natsel {}.{}.{}", v.major, v.minor, v.revision);
    println!("double precision: {}", is_double_precision());
    println!("max manifold points: {}", get_max_manifold_points());
}
```

`Version` 是 `{ major, minor, revision }` 三个 `i32`。当前是 `0.2.0`。`is_double_precision()` 目前恒为 `true`：世界坐标是 f64，局部几何与速度是 f32。`get_max_manifold_points()` 返回编译时的流形点数上限，当前为 4。

## 无依赖意味着什么

`cargo tree` 只会看到 `natsel` 自己：

```text
natsel v0.2.0 (/path/to/Natsel)
```

这对集成有几点实际影响：

- **不会引入传递依赖冲突**。引擎不会锁死 `glam`、`nalgebra` 之类的数学库版本，你可以自由使用任何数学库，只在边界处转换成 `natsel::math::types` 的类型。
- **不会有 `-sys` crate 与构建脚本**，交叉编译到新 target 时不需要额外准备工具链。
- **编译时间随引擎规模线性增长**，与依赖数量无关。release 构建比 debug 慢，这是为运行性能付的代价（crate 自身已配置好优化选项，你不需要额外设置）。

## 建议的 Cargo 配置

引擎对 release 性能敏感，游戏本体通常一起受益于同样的设置：

```toml
[profile.release]
opt-level = 3
lto = "thin"
codegen-units = 1
overflow-checks = false
```

打开 `overflow-checks` 会让物理求解变慢，通常不值得为发行版开启。

调试物理问题时则相反：用 debug 构建配合 `validate` 特性，让内部校验尽早暴露问题。

## 下一步

- [快速开始](/docs/getting-started)：写出第一个可运行场景。
- [核心概念](/docs/concepts)：公共类型的设计与取舍。
- [架构](/docs/architecture)：模块划分与一次 `step` 内部发生了什么。
