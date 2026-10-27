# 核心概念

本页介绍 Natsel 的核心类型与设计。

## Vec3 与 Transform

公开 API 是 Swift 风格的：`Vec3`、`Quat`、`Transform`、`BodyID`、`World`，以及 `makeBoxHull`、`computeCosSin` 等自由函数，没有 C 风格的前缀。

```swift
let position = Vec3(0, 5, 0)
let transform = Transform(position: position, rotation: Quat.identity)
```

## BodyDef

`BodyDef` 描述一个刚体的静态配置：类型、位置、旋转、形状、摩擦、恢复系数等。

```swift
let def = BodyDef(
    type: .dynamic,
    position: Vec3(0, 5, 0),
    shape: .sphere(radius: 0.5),
    friction: 0.6,
    restitution: 0.2)
let id = world.addBody(def)
```

## 形状

Natsel 支持多种碰撞形状：

| 形状 | 说明 |
|------|------|
| 凸包 Hull | 由 `makeBoxHull` 等构建器生成，内容键共享 |
| 网格 Mesh | 三角形网格，带 BVH |
| 高度场 HeightField | 地形 |
| 复合 Compound | 多个子形状组合 |
| 球 / 胶囊 / 盒 | 常用基本体 |

相同内容的凸包会被世界"内化"为同一个实例，避免重复存储。

## BodyID

`addBody` 返回一个 `BodyID`，后续查询与操作都通过它进行，避免直接持有内部数据。

```swift
let position = world.position(id)
let velocity = world.linearVelocity(id)
```

## 关节

关节通过 `JointDef` 描述，包括旋转、6DOF、轮子、路径等类型。

```swift
let hinge = world.addJoint(RevoluteJointDef(
    bodyA: frame,
    bodyB: door,
    anchor: Vec3(0, 1, 0),
    axis: Vec3(0, 1, 0)))
```

## 传感器与事件

传感器不产生物理响应，只报告重叠。接触、命中与连续冲击通过事件 API 暴露：

```swift
world.continuousImpacts()  // 本步 CCD 冲击，每步重置
```

## 世界服务

- **录制 / 回放**：记录世界状态，用于复现
- **快照**：保存与恢复世界
- **调度器**：多线程按岛屿并行求解

## 特殊系统

- **车辆**：传动、差速、轮胎与碰撞
- **虚拟角色**：角色控制器
- **骨骼**：骨骼映射、布娃娃、骨骼动画
- **软体**：可变形体
