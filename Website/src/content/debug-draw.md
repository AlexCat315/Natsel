# 调试绘制

`world.draw(&DebugDraw, mask_bits)` 把世界当前的形状、包围盒、接触、关节、车辆与软体交给一组回调，由你决定怎么画。引擎不假设渲染后端：它只调用你提供的 `Arc<dyn Fn>`，你把收到的线段、点、变换塞进自己的缓冲，再提交给渲染器。

这套接口是排障的主力工具：接触法线不对、关节锚点错位、轮子悬空、软体穿模，几乎都能一眼看出来。

## 模型说明

`DebugDraw` 是两部分的组合：

| 部分 | 内容 |
|------|------|
| 回调 | 8 个可选的 `Arc<dyn Fn>`，分别处理形状、线段、变换、点、球、胶囊、包围盒、文字 |
| 开关 | 一组 `bool`，决定哪些内容会被提交；另有裁剪包围盒与缩放系数 |

`draw` 从不自己保存几何：如果某个回调是 `None`，对应的内容就被直接跳过。

### 回调字段

| 字段 | 类型 | 语义 |
|------|------|------|
| `draw_shape` | `Arc<dyn Fn(*mut c_void, WorldTransform, HexColor)>` | 绘制用户形状；第一个参数是 `create_debug_shape` 返回的句柄 |
| `draw_segment` | `Arc<dyn Fn(Pos, Pos, HexColor)>` | 一条线段（两点世界坐标 + 颜色） |
| `draw_transform` | `Arc<dyn Fn(WorldTransform)>` | 一个坐标系 |
| `draw_point` | `Arc<dyn Fn(Pos, f32, HexColor)>` | 一个点，第二个参数是尺寸提示 |
| `draw_sphere` | `Arc<dyn Fn(Pos, f32, HexColor, f32)>` | 球心、半径、颜色、透明度 |
| `draw_capsule` | `Arc<dyn Fn(Pos, Pos, f32, HexColor, f32)>` | 两个端点、半径、颜色、透明度 |
| `draw_bounds` | `Arc<dyn Fn(AABB, HexColor)>` | 轴对齐包围盒 |
| `draw_box` | `Arc<dyn Fn(Vec3, WorldTransform, HexColor)>` | 有向盒（半尺寸 + 变换） |
| `draw_string` | `Arc<dyn Fn(Pos, &str, HexColor)>` | 世界空间文字 |

### 开关字段

| 字段 | `DebugDraw::new()` 的默认 | 说明 |
|------|---------------------------|------|
| `draw_shapes` | `true` | 绘制形状（需要 `create_debug_shape`） |
| `draw_joints` | `true` | 绘制关节 |
| `draw_joint_extras` | `false` | 绘制关节的额外信息 |
| `draw_bounds_option` | `false` | 绘制形状包围盒（`fat_aabbs`，金色） |
| `draw_mass` | `false` | 标注质量与质心坐标系 |
| `draw_sleep` | `false` | 标注睡眠状态（蓝 → 红） |
| `draw_body_names` | `false` | 绘制刚体名字 |
| `draw_contacts` | `false` | 绘制接触点 |
| `draw_anchor_a` | `false` | 绘制锚点 A |
| `draw_graph_colors` | `false` | 按约束图着色 |
| `draw_contact_features` | `false` | 绘制接触特征 |
| `draw_contact_normals` | `false` | 绘制接触法线（浅灰线段） |
| `draw_contact_forces` | `false` | 绘制接触力（品红线段） |
| `draw_islands` | `false` | 按岛屿着色 |
| `draw_vehicles` | `true` | 绘制车辆轮子与防倾杆 |
| `draw_soft_bodies` | `true` | 绘制软体的边 |

`draw_shapes`、`draw_joints`、`draw_vehicles`、`draw_soft_bodies` 默认开启，其余默认关闭。

### 其它字段

| 字段 | 默认 | 说明 |
|------|------|------|
| `drawing_bounds` | `±1e6` 的 `AABB` | 裁剪范围；只绘制与它重叠的内容 |
| `force_scale` | `1.0` | 绘制力时的缩放 |
| `joint_scale` | `1.0` | 绘制关节的全局缩放 |
| `create_debug_shape` | `None` | `Arc<dyn Fn(&DebugShape) -> Option<*mut c_void>>`，首次遇到形状时调用 |

`DebugDraw::new()` 设置上面这些默认值；`DebugDraw::default()` 则全部为关闭/零值，两者不同，通常你要的是 `new()`。

## 关键前置条件：create_debug_shape

这是最容易踩的坑：**`draw_shape` 只有在你提供了 `create_debug_shape` 并且它返回了 `Some` 时才会被调用。**

原因是引擎需要一个渲染资源句柄来代表这个形状。流程是：

1. 引擎遇到一个还没有句柄的形状，调用 `create_debug_shape(&DebugShape)`。
2. 你返回一个 `Some(句柄)`（比如网格缓冲区的裸指针），引擎把它缓存在形状上。
3. 之后每一帧都调用 `draw_shape(句柄, transform, color)`。
4. 句柄会被缓存，所以同一形状的工厂只调用一次。

```rust
options.create_debug_shape = Some(Arc::new(|_shape| Some(0x1 as *mut c_void)));
options.draw_shape = Some(Arc::new(|_handle, _xf, _color| { /* 提交绘制 */ }));
```

返回 `None` 的形状会被静默跳过，不报错——表现就是"什么都没画出来"。

## 完整可运行示例

回调是 `Fn`（不是 `FnMut`）且要求 `'static`，所以不能用普通的 `&mut Vec` 捕获。标准做法是共享一个 `Arc<Mutex<Vec<Line>>>`，回调里加锁推入，`draw` 返回后再取出来交给渲染器。

```rust
use std::ffi::c_void;
use std::sync::{Arc, Mutex};

use natsel::collision::hull::make_box_hull;
use natsel::math::types::{Pos, Quat, Vec3, WorldTransform};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::debug_draw::{make_debug_color, DebugDraw, DebugMaterial, HexColor};
use natsel::types::shape_def::ShapeDef;

/// 一条待渲染的线段，颜色已解开为 RGB。
#[derive(Clone, Copy, Debug)]
struct Line {
    a: Pos,
    b: Pos,
    rgb: u32,
}

/// 一个待渲染的点。
#[derive(Clone, Copy, Debug)]
struct Point {
    p: Pos,
    size: f32,
    rgb: u32,
}

/// 把 HexColor 的低 24 位拆成 RGB；高字节是材质预设，渲染器可以忽略。
fn rgb_of(color: HexColor) -> u32 {
    color & 0x00FF_FFFF
}

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -9.81, 0.0);

    // 静态地面
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_box_hull(10.0, 0.5, 10.0),
    );

    // 一个动态盒子
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 3.0, 0.0);
    def.rotation = Quat::IDENTITY;
    let faller = world.create_body(&def);
    let _ = world.create_hull_shape(faller, &ShapeDef::default(), &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..90 {
        world.step(1.0 / 60.0, 4);
    }

    // ---- 收集缓冲：闭包要 Fn + 'static，所以用 Arc<Mutex<..>> 共享 ----
    let lines: Arc<Mutex<Vec<Line>>> = Arc::new(Mutex::new(Vec::new()));
    let points: Arc<Mutex<Vec<Point>>> = Arc::new(Mutex::new(Vec::new()));
    let labels: Arc<Mutex<Vec<(Pos, String)>>> = Arc::new(Mutex::new(Vec::new()));

    let mut options = DebugDraw::new();

    // 1) 形状句柄工厂：返回非 None，draw_shape 才会被调用
    options.create_debug_shape = Some(Arc::new(|shape| {
        // 这里本应创建/查找网格资源；示例用递增的假句柄代替。
        let _ = shape.type_;
        Some(0x1000 as *mut c_void)
    }));

    // 2) 形状绘制
    {
        let lines = Arc::clone(&lines);
        options.draw_shape = Some(Arc::new(
            move |_handle: *mut c_void, xf: WorldTransform, color: HexColor| {
                // 用坐标系的三根轴代替真正的网格，能看位置与朝向。
                let origin = xf.p;
                let axes = [
                    (Vec3::AXIS_X, 0xFF0000u32),
                    (Vec3::AXIS_Y, 0x00FF00u32),
                    (Vec3::AXIS_Z, 0x0000FFu32),
                ];
                let mut out = lines.lock().unwrap();
                let _ = color;
                for (axis, rgb) in axes {
                    out.push(Line {
                        a: origin,
                        b: Pos::new(
                            origin.x + axis.x as f64 * 0.5,
                            origin.y + axis.y as f64 * 0.5,
                            origin.z + axis.z as f64 * 0.5,
                        ),
                        rgb,
                    });
                }
            },
        ));
    }

    // 3) 线段：接触法线、软体边、车辆轮子都会走这里
    {
        let lines = Arc::clone(&lines);
        options.draw_segment = Some(Arc::new(move |a: Pos, b: Pos, color: HexColor| {
            lines.lock().unwrap().push(Line {
                a,
                b,
                rgb: rgb_of(color),
            });
        }));
    }

    // 4) 点：睡眠状态标记等
    {
        let points = Arc::clone(&points);
        options.draw_point = Some(Arc::new(move |p: Pos, size: f32, color: HexColor| {
            points.lock().unwrap().push(Point {
                p,
                size,
                rgb: rgb_of(color),
            });
        }));
    }

    // 5) 文字
    {
        let labels = Arc::clone(&labels);
        options.draw_string = Some(Arc::new(move |p: Pos, text: &str, _color: HexColor| {
            labels.lock().unwrap().push((p, text.to_owned()));
        }));
    }

    // 打开想看的调试图层
    options.draw_shapes = true;
    options.draw_bounds_option = true;
    options.draw_contacts = true;
    options.draw_contact_normals = true;
    options.draw_mass = true;
    options.draw_sleep = true;

    // ---- 执行绘制（u64::MAX = 所有类别都参与裁剪查询）----
    world.draw(&options, u64::MAX);

    // ---- 取出结果交给渲染器 ----
    let lines = lines.lock().unwrap();
    let points = points.lock().unwrap();
    let labels = labels.lock().unwrap();

    println!("lines: {}, points: {}, labels: {}", lines.len(), points.len(), labels.len());

    for line in lines.iter() {
        // renderer.draw_line(line.a, line.b, line.rgb);
        let _ = (line.a, line.b, line.rgb);
    }
    for point in points.iter() {
        // renderer.draw_point(point.p, point.size, point.rgb);
        let _ = (point.p, point.size, point.rgb);
    }
    for (p, text) in labels.iter() {
        // renderer.draw_text(*p, text);
        let _ = (p, text);
    }

    // 每帧重复：清空缓冲 → world.draw → 提交
}
```

每帧的节奏就是三行：

```rust
lines.lock().unwrap().clear();
world.draw(&options, u64::MAX);
renderer.submit(&lines.lock().unwrap());
```

把 `options` 建在循环外，只重建一次；`create_debug_shape` 的句柄会被形状缓存，所以工厂不会每帧被调用。

## 颜色

颜色是打包的 `u32`：低 24 位是 RGB，高 8 位可选携带材质预设。

```rust
pub type HexColor = u32;

pub fn make_debug_color(rgb: HexColor, material: DebugMaterial) -> HexColor
```

`DebugMaterial` 是渲染材质的提示：

| 变体 | 值 |
|------|-----|
| `DebugMaterial::Default` | 0 |
| `DebugMaterial::Matte` | 1 |
| `DebugMaterial::Soft` | 2 |
| `DebugMaterial::Dead` | 3 |
| `DebugMaterial::Glossy` | 4 |
| `DebugMaterial::Metallic` | 5 |

```rust
use natsel::types::debug_draw::{debug_colors, make_debug_color, DebugMaterial};

let red = make_debug_color(debug_colors::RED, DebugMaterial::Glossy);
// 只想用纯色时，直接写 RGB 也行：
let blue: u32 = 0x0000FF;
```

`debug_colors` 模块提供了一组具名 CSS 颜色常量，例如 `RED`、`GREEN`、`BLUE`、`YELLOW`、`CYAN`、`MAGENTA`、`WHITE`、`BLACK`、`ORANGE`、`GOLD`、`LIGHT_GRAY`、`DARK_GRAY`、`LIGHT_STEEL_BLUE`、`TEAL`、`SALMON` 等。引擎自己在绘制不同图层时用的就是这些常量（接触法线浅灰、接触力品红、包围盒金色、软体边浅钢蓝或灰色）。

渲染器只需要取 `color & 0x00FF_FFFF` 当 RGB，忽略高字节即可。

## 形状几何

`create_debug_shape` 收到 `DebugShape`：

| 字段 | 类型 | 说明 |
|------|------|------|
| `shape_id` | `ShapeID` | 形状 ID |
| `type_` | `ShapeType` | 形状类型 |
| `geometry` | `DebugShapeGeometry` | 几何数据 |

`DebugShapeGeometry` 的变体：

| 变体 | 载荷 |
|------|------|
| `Capsule(*const Capsule)` | 胶囊指针 |
| `Compound(Compound)` | 复合形状 |
| `HeightField(HeightField)` | 高度场 |
| `Hull(*const HullData)` | 凸包数据指针 |
| `Mesh(*const Mesh)` | 网格指针 |
| `Sphere(*const Sphere)` | 球指针 |

指针变体指的是引擎内部存储，用于构建渲染网格。如果你只关心位置（像上面的示例那样画坐标系），可以完全不读它们。

## 裁剪

`world.draw` 的第二个参数是 `mask_bits`，与形状的碰撞过滤一起决定哪些形状参与裁剪查询；`options.drawing_bounds` 再按空间范围裁剪。只调试某个区域时：

```rust
options.drawing_bounds = AABB::new(
    Vec3::new(-10.0, -1.0, -10.0),
    Vec3::new(10.0, 10.0, 10.0),
);
world.draw(&options, u64::MAX);
```

车辆与软体不参与 `mask_bits` 过滤，它们只受 `drawing_bounds` 与各自的开关控制。

## 注意事项

- **`draw_shape` 依赖 `create_debug_shape`**：不提供工厂、或工厂返回 `None`，`draw_shape` 一次都不会被调用。这是"打开了 `draw_shapes` 却什么都没画"的首要原因。
- **`draw_segment` 是万能入口**：接触法线、接触力、关节、车辆、软体边全部通过它提交。想让这些东西出现，`draw_segment` 必须设置。
- **`DebugDraw::new()` 与 `default()` 不同**：`new()` 打开 `draw_shapes` / `draw_joints` / `draw_vehicles` / `draw_soft_bodies`，`drawing_bounds` 为 `±1e6`；`default()` 全是关闭与零值。用 `new()`。
- **回调是 `Fn` 不是 `FnMut`**：不能直接捕获 `&mut Vec`。用 `Arc<Mutex<Vec<_>>>`（或多线程下用无锁队列），这是本页示例的写法。
- **不要在回调里调用 `world`**：`draw` 持有 `&mut World`，闭包又要求 `'static`，捕获世界引用无法通过编译。需要世界数据就在 `draw` 之前取好。
- **`mask_bits` 传 `u64::MAX`**：传 `0` 会让裁剪查询匹配不到任何形状，表现为完全空白。
- **`drawing_bounds` 是裁剪而非相机**：超出范围的形状不提交，但它不会帮你做视锥剔除以外的任何变换。
- **形状句柄会被缓存且随形状销毁**：工厂对同一形状只调用一次，句柄存放在形状上；形状随刚体一起销毁时缓存一并消失。不要在刚体销毁后继续引用旧句柄。
- **`DebugDraw` 是 `Clone`**：可以预置多套配置（例如"只看接触"和"只看软体"）轮流使用。
- **力与关节的量级靠缩放系数**：`force_scale` 与 `joint_scale` 默认 1.0，接触力或关节尺寸在真实尺度下可能太小看不见，按需放大。

## 相关页面

- [刚体](/docs/bodies)：`draw_mass`、`draw_sleep`、`draw_body_names` 标注的都是刚体。
- [形状](/docs/shapes)：`DebugShape` 的几何变体对应各类形状。
- [材质与过滤](/docs/materials)：`mask_bits` 与形状过滤共同决定裁剪范围。
- [关节](/docs/joints)：`draw_joints` 与 `draw_joint_extras` 可视化关节。
- [车辆](/docs/vehicle)：`draw_vehicles` 画出轮子与防倾杆。
- [软体](/docs/soft-body)：`draw_soft_bodies` 画出软体的边。
- [浮力与爆炸](/docs/buoyancy)：用线段画出液面与爆炸半径。
