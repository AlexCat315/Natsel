# 快速开始

本指南带你通过 SwiftPM 集成 Natsel，并运行第一个物理世界。

## 环境要求

- Swift 6.4 工具链
- macOS 26 / iOS 26 及以上（需要 `InlineArray`）

## 添加依赖

在你的 `Package.swift` 中加入：

```swift
dependencies: [
    .package(url: "https://github.com/AlexCat315/Natsel", branch: "main"),
],
targets: [
    .target(name: "MyApp", dependencies: ["Natsel"]),
]
```

## 创建一个世界

```swift
import Natsel

let world = World()
```

## 添加地面

地面是一个静态刚体：

```swift
world.addBox(
    position: Vec3(0, -1, 0),
    halfExtents: Vec3(20, 1, 20),
    isStatic: true)
```

## 添加一个动态盒子

```swift
let body = world.addBody(BodyDef(
    type: .dynamic,
    position: Vec3(0, 5, 0),
    shape: .box(halfExtents: Vec3(0.5, 0.5, 0.5))))
```

## 推进模拟

```swift
for _ in 0 ..< 60 {
    world.step(1.0 / 60.0)
}
```

## 读取位置

```swift
let position = world.position(body)
print(position)
```

## 运行测试与基准

```bash
swift test
swift run Benchmarks
```

## 下一步

- 阅读 [核心概念](/docs/concepts)，理解 `BodyDef`、形状与世界服务。
- 学习 [确定性](/docs/determinism)，了解多线程下的可重复模拟。
- 查看 [基准性能](/benchmarks)，了解性能数据。
