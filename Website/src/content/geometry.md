# 几何构建器

[形状](/docs/shapes) 讲的是"挂什么形状、怎么配置"；本页讲**几何本身怎么造出来** —— 凸包的顶点、网格的三角形、高度场的高度采样、复合体的子形状。

Natsel 不要求你把几何序列化成文件再用工具导入。所有几何都可以用普通 Rust 代码生成，这对程序化关卡、体素世界、地形生成非常方便。

## 凸包（Hull）

凸包是最常用的动态碰撞几何：任意凸多面体，用半空间表示，不要求是盒子。

### 盒子族构建器

| 函数 | 签名 | 说明 |
|------|------|------|
| `make_box_hull` | `(hx: f32, hy: f32, hz: f32) -> Hull` | 轴对齐盒子，三轴半长 |
| `make_cube_hull` | `(half_width: f32) -> Hull` | 立方体，等价于三轴同值 |
| `make_offset_box_hull` | `(hx, hy, hz, offset: Vec3) -> Hull` | 中心偏移到 `offset` 的盒子 |
| `make_transformed_box_hull` | `(hx, hy, hz, transform: Transform) -> Hull` | 应用旋转变换的盒子 |
| `make_scaled_box_hull` | `(half_widths: Vec3, transform: Transform, post_scale: Vec3) -> Hull` | 盒子经变换后再缩放，缩放被烘焙进顶点 |

这些函数全部返回值类型 `Hull`（不是 `Option`），不会失败。参数都是**半长**：

```rust
use natsel::collision::hull::{make_box_hull, make_cube_hull, make_transformed_box_hull};
use natsel::math::functions::make_quat_from_axis_angle;
use natsel::math::types::{Transform, Vec3};

// 边长 1 的立方体
let cube = make_cube_hull(0.5);

// 20 x 1 x 20 的地面
let ground = make_box_hull(10.0, 0.5, 10.0);

// 绕 Y 轴转 45 度的盒子（旋转烘焙进几何）
let rot = make_quat_from_axis_angle(Vec3::AXIS_Y, std::f32::consts::FRAC_PI_4);
let rotated = make_transformed_box_hull(0.5, 0.5, 0.5, Transform::new(Vec3::ZERO, rot));
```

`make_scaled_box_hull` 会先把缩放合并到顶点里，同时保持旋转矩阵正交。非均匀缩放会产生非正交的盒子框架，这个函数会处理掉这个问题，所以要用"拉伸的盒子"时应该用它，而不是自己乘顶点。

### 从点云生成凸包

任意点集都能算出其凸包：

```rust
use natsel::collision::hull_builder::create_hull;

let points = vec![
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(0.0, 0.0, 1.0),
    Vec3::new(0.6, 0.6, 0.6),
];

let hull = create_hull(&points, 32).expect("hull");
```

`max_vertex_count` 是顶点数上限（会被夹到 4 到 128 之间）。点数超过上限时，算法会尝试简化。

返回 `Option<Hull>`，失败条件：

- 点数少于 4（构不成三维体积）。
- 所有点共面或共线（退化，没有体积）。
- 结果超出顶点/面/边上限。

所以从外部数据（用户上传的模型、随机点）造凸包时，必须处理 `None`：

```rust
match create_hull(&points, 32) {
    Some(hull) => { world.create_hull_shape(body, &ShapeDef::default(), &hull); }
    None => eprintln!("点云退化，无法生成凸包"),
}
```

### 显式指定面

想精确控制拓扑（例如导入模型后需要保持原始三角面）时用 `hull_from_faces`：

```rust
use natsel::collision::hull::hull_from_faces;

let points = vec![
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(0.0, 0.0, 1.0),
];

// 每个面是一个顶点索引多边形，索引指向 points
let faces = vec![
    vec![0u32, 2, 1],
    vec![0u32, 1, 3],
    vec![0u32, 3, 2],
    vec![1u32, 2, 3],
];

let hull = hull_from_faces(&points, &faces).expect("hull");
```

要求：至少 4 个点、至少 4 个面、点不超过 128、面不超过 128。返回 `Option`，拓扑不闭合或退化时返回 `None`。

### 程序化图元

几个现成的凸包图元，适合快速搭场景：

| 函数 | 签名 | 约束 |
|------|------|------|
| `create_cylinder` | `(height, radius, y_offset, sides) -> Option<Hull>` | `height > 0`, `radius > 0`, `3 <= sides <= 32` |
| `create_cone` | `(height, radius1, radius2, slices) -> Option<Hull>` | `height > 0`, 两半径均 `> 0`, `4 <= slices <= 32` |
| `create_rock` | `(radius) -> Option<Hull>` | 生成不规则石块，10 个顶点 |
| `create_complex_hull` | `(radius) -> Option<Hull>` | 更复杂的测试用外形，32 个顶点 |

```rust
use natsel::collision::hull_builder::{create_cylinder, create_rock, create_cone};

let pillar = create_cylinder(3.0, 0.4, 1.5, 16).expect("cylinder");
let boulder = create_rock(0.8).expect("rock");
let cone = create_cone(1.5, 0.7, 0.1, 12).expect("cone");
```

`create_cylinder` 的 `y_offset` 是圆柱底面在局部 Y 轴的起点，所以 `create_cylinder(2.0, 0.5, 0.0, 16)` 得到的是从 `y=0` 到 `y=2` 的柱体。想让建模原点在中心，传 `-height / 2`。

### 凸包的属性查询

```rust
let hull = make_box_hull(1.0, 0.5, 2.0);

let n = hull.vertex_count();        // 顶点数
let f = hull.face_count();          // 面数
let area = hull.surface_area();     // f32
let vol = hull.volume();            // f32
let r = hull.inner_radius();        // 内切球半径
let c = hull.center();              // Vec3，质心
let inertia = hull.central_inertia(); // Matrix3
let aabb = hull.aabb();             // AABB
let hash = hull.hash();             // 内容哈希
```

`volume()` 与 `central_inertia()` 正是质量计算的输入：质量 = `density * volume()`，所以 `make_box_hull(0.5, 0.5, 0.5)` 配默认密度 1000 得到质量 1000。

## 三角网格（TriangleMesh）

网格是任意三角形集合，带一棵 BVH 加速结构。适合静态复杂几何。

### 手工构造

```rust
use natsel::collision::mesh::build::MeshBuildOptions;
use natsel::collision::mesh::storage::TriangleMesh;
use natsel::math::types::Vec3;

// 一个 2x2 的正方形地面（四个顶点、两个三角形）
let vertices = [
    Vec3::new(-1.0, 0.0, -1.0),
    Vec3::new(1.0, 0.0, -1.0),
    Vec3::new(1.0, 0.0, 1.0),
    Vec3::new(-1.0, 0.0, 1.0),
];
let indices: [i32; 6] = [0, 1, 2, 0, 2, 3];

let options = MeshBuildOptions {
    weld_tolerance: 1e-4,
    use_median_split: true,
    identify_edges: true,
    clockwise_winding: false,
};

let mesh = TriangleMesh::new(&vertices, &indices, None, options).expect("mesh");
```

`TriangleMesh::new` 签名：

```rust
fn new(
    vertices: &[Vec3],
    indices: &[i32],
    materials: Option<&[u8]>,
    options: MeshBuildOptions,
) -> Option<Self>
```

失败条件（都返回 `None`）：

- 顶点少于 3 个。
- 索引为空，或索引数不是 3 的倍数。
- 有索引越界（`< 0` 或 `>= vertices.len()`）。
- 提供了材质数组但长度不等于三角形数（`indices.len() / 3`）。

`materials` 是**逐三角形的材质槽索引**，长度必须精确等于三角形数。它存的是"第几个材质"，不是材质本身 —— 材质本身放在 `ShapeDef.materials` 里。

### MeshBuildOptions

| 字段 | 类型 | 默认 | 含义 |
|------|------|------|------|
| `weld_tolerance` | `f32` | `0.0` | 顶点合并容差。`0.0` 表示保留所有输入顶点 |
| `use_median_split` | `bool` | `false` | BVH 用中位数分割（更平衡）而非空间中位分割 |
| `identify_edges` | `bool` | `false` | 预计算边分类（凹/凸边），改善接触稳定性 |
| `clockwise_winding` | `bool` | `false` | 三角形绕序是顺时针（默认逆时针） |

几个实用建议：

- **`weld_tolerance` 只在你怀疑顶点重复时设正数。** 导入的模型常有重合顶点，合并后 BVH 更小、射线检测更准。程序化生成的网格通常不需要。
- **`identify_edges = true` 值得开。** 它让引擎知道哪些边是凹边，物体沿网格滑动时不会在三角形接缝处被"卡住"。代价是构建慢一点、内存多一点。
- **`use_median_split = true` 对不均匀分布的网格更稳。** 官方提供的程序化网格生成器基本都开这个。
- **绕序决定了法线方向。** 如果你的网格看起来"从内侧碰撞"，就是绕序搞反了，改 `clockwise_winding`。

### 程序化网格生成器

`natsel::collision::mesh::procedural` 提供六个生成器，全部返回 `Option<TriangleMesh>`：

| 函数 | 签名 |
|------|------|
| `create_wave_mesh` | `(x_count, z_count, cell_width, amplitude, row_frequency, column_frequency)` |
| `create_torus_mesh` | `(radial_resolution, tubular_resolution, radius, thickness)` |
| `create_box_mesh` | `(center: Vec3, extent: Vec3, identify_edges: bool)` |
| `create_hollow_box_mesh` | `(center: Vec3, extent: Vec3)` |
| `create_platform_mesh` | `(center: Vec3, height, top_width, bottom_width)` |
| `create_grid_mesh` | `(x_count, z_count, cell_width, material_count, identify_edges)` |

```rust
use natsel::collision::mesh::procedural::{
    create_box_mesh, create_grid_mesh, create_hollow_box_mesh,
    create_platform_mesh, create_torus_mesh, create_wave_mesh,
};

let wave = create_wave_mesh(16, 16, 1.0, 0.5, 2.0, 2.0).expect("wave");
let torus = create_torus_mesh(24, 16, 2.0, 0.5).expect("torus");
let box_mesh = create_box_mesh(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), true).expect("box");
let hollow = create_hollow_box_mesh(Vec3::ZERO, Vec3::new(2.0, 2.0, 2.0)).expect("hollow");
let platform = create_platform_mesh(Vec3::ZERO, 2.0, 1.0, 3.0).expect("platform");
let grid = create_grid_mesh(8, 8, 1.0, 4, true).expect("grid");
```

要点：

- `create_box_mesh` 的 `extent` 是**半长**，与 `make_box_hull` 一致。
- `create_hollow_box_mesh` 的三角形朝向反转，得到"从内侧可见、外侧不可见"的盒子 —— 用于把摄像机装进去的环境盒。
- `create_platform_mesh` 是上窄下宽的梯形柱台，适合做跳台。
- `create_grid_mesh` 的 `material_count` 在 `1..=255` 内，会按顺序把三角形轮流分配给不同材质槽，配合 `ShapeDef.materials` 就能做出"草地/泥地/冰面"交错的地面。传 `0` 则不带材质索引。
- `create_wave_mesh` 生成正弦乘积的起伏平面，是最快的地形原型。

这些都是**静态**几何的起点：造好之后用 `create_mesh_shape` 挂到静态刚体上。

### 网格的属性查询

```rust
let n = mesh.vertex_count();          // usize
let t = mesh.triangle_count();        // usize
let mats = mesh.material_indices();   // &[u8]
let bounds = mesh.bounds();           // AABB
let area = mesh.surface_area();       // f32
let hash = mesh.hash();               // u64
let deg = mesh.degenerate_triangle_count(); // usize，退化三角形数

// 取某个三角形的三个顶点
let verts = mesh.triangle_vertices(0); // [Vec3; 3]
```

`degenerate_triangle_count()` 是个有用的体检指标：如果它不为 0，说明网格里有零面积三角形，通常是导入数据的重复顶点造成的，会降低碰撞质量。用 `weld_tolerance` 可以消掉大部分。

## 高度场（HeightField）

高度场是规则网格采样的地形：存 `count_x * count_z` 个高度值，而不是三角形列表。内存远小于等效网格，适合大范围地面。

### 两个工厂函数

```rust
use natsel::collision::height_field::factories::{
    create_grid_height_field, create_wave_height_field,
};

// 平地，所有高度为 0
let flat = create_grid_height_field(
    64,                          // 行数
    64,                          // 列数
    Vec3::new(100.0, 1.0, 100.0),// 整体缩放（x 宽、y 高度、z 深）
    false,                       // 是否挖洞
).expect("flat field");

// 正弦乘积的丘陵
let hills = create_wave_height_field(
    64, 64,
    Vec3::new(100.0, 6.0, 100.0),
    2.0,     // 行方向频率
    3.0,     // 列方向频率
    false,   // 是否挖洞
).expect("wave field");
```

| 函数 | 签名 |
|------|------|
| `create_grid_height_field` | `(rows: usize, cols: usize, scale: Vec3, make_holes: bool) -> Option<HeightField>` |
| `create_wave_height_field` | `(rows, cols, scale: Vec3, row_frequency: f32, column_frequency: f32, make_holes: bool) -> Option<HeightField>` |

两者都要求 `rows >= 2 && cols >= 2`，否则返回 `None`。

`scale` 的三个分量各有含义：`x` 与 `z` 是地形在世界中的横向尺寸，`y` 是高度方向的缩放。所以 `Vec3::new(100.0, 1.0, 100.0)` 是 100x100 米的平地，而 `Vec3::new(100.0, 6.0, 100.0)` 是同样大小、起伏最多 6 米的丘陵。

`make_holes = true` 会把每第 16 个格子挖空，用来测试物体掉落或做地洞。它主要用于调试，正式关卡很少用。

### 手工构造 HeightFieldDef

需要从自己的高度数据（噪声函数、高度图文件、体素世界）生成地形时：

```rust
use natsel::collision::height_field::HeightField;
use natsel::types::geometry::HeightFieldDef;
use natsel::math::types::Vec3;

const COLS: usize = 32;
const ROWS: usize = 32;

let mut def = HeightFieldDef::default();
def.count_x = COLS as i32;
def.count_z = ROWS as i32;
def.scale = Vec3::new(64.0, 8.0, 64.0);
def.global_minimum_height = -1.0;
def.global_maximum_height = 1.0;

// 高度值，长度必须正好是 count_x * count_z
def.heights = (0..ROWS * COLS)
    .map(|i| {
        let (x, z) = (i % COLS, i / COLS);
        ((x as f32 * 0.5).sin() * (z as f32 * 0.5).cos()) * 0.5
    })
    .collect();

// 可选的逐格材质索引，长度必须是 (count_x - 1) * (count_z - 1)
def.material_indices = vec![0u8; (COLS - 1) * (ROWS - 1)];

let field = HeightField::new(&def).expect("height field");
```

`HeightFieldDef` 字段：

| 字段 | 类型 | 说明 |
|------|------|------|
| `heights` | `Vec<f32>` | 高度采样，长度必须等于 `count_x * count_z` |
| `material_indices` | `Vec<u8>` | 逐格材质索引，长度必须是 `(count_x-1) * (count_z-1)`；空表示不用 |
| `scale` | `Vec3` | 三轴缩放，三个分量都必须 `> 0` |
| `count_x` | `i32` | 列数，必须 `>= 2` |
| `count_z` | `i32` | 行数，必须 `>= 2` |
| `global_minimum_height` | `f32` | 高度下界，必须有限且 `<= 上界` |
| `global_maximum_height` | `f32` | 高度上界 |
| `clockwise_winding` | `bool` | 三角形绕序，默认 `false` |

`HeightField::new` 返回 `Option`，校验相当严格 —— 长度不匹配、缩放非正、高度含 `NaN`/无穷、上下界反了，全部返回 `None`。这是好事：地形数据错误会在创建时暴露，而不是在模拟中变成幽灵穿透。

高度值的实际世界高度是 `height * scale.y`。`global_minimum_height` 与 `global_maximum_height` 应该包住所有采样值的范围（乘以 `scale.y` 之前还是之后取决于你的数据约定，最稳的做法是把实际极值先算出来再填）。

## 复合形状（Compound）

复合形状把多个子形状烘焙成一个整体。它在创建时把所有子几何复制进一块连续内存，运行时不再有指针跳转。

### CompoundDef

```rust
use natsel::collision::compound::compound::Compound;
use natsel::math::types::{Quat, Transform, Vec3};
use natsel::types::geometry::{
    Capsule, CompoundCapsuleDef, CompoundDef, CompoundHullDef,
    CompoundSphereDef, Sphere,
};
use natsel::types::shape_def::SurfaceMaterial;

let mut def = CompoundDef::default();

// 一个偏移的盒子
let mut hull_child = CompoundHullDef::default();
hull_child.hull = Some(make_box_hull(1.0, 1.0, 1.0));
hull_child.transform = Transform::new(Vec3::new(2.0, 0.0, 0.0), Quat::IDENTITY);
hull_child.material = SurfaceMaterial::default_material();
def.hulls.push(hull_child);

// 一个球
let mut sphere_child = CompoundSphereDef::default();
sphere_child.sphere = Sphere::new(Vec3::new(-2.0, 0.0, 0.0), 0.5);
sphere_child.material = SurfaceMaterial::default_material();
def.spheres.push(sphere_child);

// 一根胶囊
let mut capsule_child = CompoundCapsuleDef::default();
capsule_child.capsule = Capsule::new(
    Vec3::new(0.0, -1.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
    0.3,
);
capsule_child.material = SurfaceMaterial::default_material();
def.capsules.push(capsule_child);

let compound = Compound::new(&def).expect("compound");
```

`CompoundDef` 字段：

| 字段 | 类型 | 子定义结构 |
|------|------|------------|
| `capsules` | `Vec<CompoundCapsuleDef>` | `{ capsule: Capsule, material: SurfaceMaterial }` |
| `hulls` | `Vec<CompoundHullDef>` | `{ hull: Option<Hull>, transform: Transform, material: SurfaceMaterial }` |
| `meshes` | `Vec<CompoundMeshDef>` | `{ mesh_data: Option<TriangleMesh>, transform: Transform, scale: Vec3, materials: Vec<SurfaceMaterial> }` |
| `spheres` | `Vec<CompoundSphereDef>` | `{ sphere: Sphere, material: SurfaceMaterial }` |

差别在于：球与胶囊的几何自带位置（`Sphere.center`、`Capsule.center1/center2`），所以子定义里没有 `transform`；凸包与网格则是"原点在几何中心"的，所以需要 `transform` 来摆放。

每个子形状都能带自己的 `material`，因此复合体天然支持"不同部位不同摩擦" —— 例如一个复合的推车，轮子用低摩擦、车厢用高摩擦。

`Compound::new` 返回 `Option<Compound>`，以下情况为 `None`：所有列表都为空、某个 `hull` 或 `mesh_data` 是 `None`、子形状超出数量限制、烘焙后超出字节上限。

### 复合体的属性

```rust
let n = compound.child_count();       // usize
let bounds = compound.bounds();       // AABB
let mats = compound.materials();      // Vec<SurfaceMaterial>
let bytes = compound.byte_count();    // i32
```

`materials()` 按子形状顺序返回材质列表，这个顺序就是子形状的 material index 映射。

### 从字节恢复

复合体可以序列化后再读回来，适合预烘焙关卡：

```rust
let bytes: Vec<u8> = compound.serialized_bytes();
// ...存盘、传输...
let restored = Compound::from_serialized_bytes(&bytes).expect("compound");
```

## 内容共享

凸包与网格都按**内容哈希**共享存储。这带来两个重要行为：

**相同的凸包只存一份。** 用 `make_box_hull(0.5, 0.5, 0.5)` 造 1000 次，然后创建 1000 个形状，内存里只有一份盒子几何：

```rust
let cube = make_cube_hull(0.5);
for i in 0..1000 {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(i as f64, 5.0, 0.0);
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &cube);
}
// 1000 个形状，但只有一份立方体几何
```

代价是每次 `create_hull_shape` 都要做一次内容比对（哈希 + 字节比较），所以"造很多不同形状"时这个开销是线性的。但"重复用同一个形状"几乎免费 —— 这是体素世界、平台跳跃关卡的关键优化。

**销毁形状会释放引用。** 引用计数归零时几何被真正释放，所以循环创建/销毁不会泄漏。

网格同理：`TriangleMesh` 内部是 `Arc`，`create_mesh_shape` 按值传入时只是增加引用计数。下面的写法是安全的，`mesh` 依然可以复用：

```rust
let mesh = create_wave_mesh(16, 16, 1.0, 0.3, 1.0, 1.0).expect("wave");

let a = world.create_mesh_shape(ground_a, &ShapeDef::default(), mesh.clone(), Vec3::ONE);
let b = world.create_mesh_shape(ground_b, &ShapeDef::default(), mesh.clone(), Vec3::new(2.0, 1.0, 2.0));
```

同一份网格配不同缩放，是"一块地形重复铺开"的标准做法。

## 完整示例

把凸包、网格、高度场、复合体全部生成并放进一个场景：

```rust
use natsel::collision::compound::compound::Compound;
use natsel::collision::height_field::factories::create_wave_height_field;
use natsel::collision::hull::{make_box_hull, make_cube_hull};
use natsel::collision::hull_builder::{create_cone, create_cylinder, create_hull, create_rock};
use natsel::collision::mesh::build::MeshBuildOptions;
use natsel::collision::mesh::procedural::{create_grid_mesh, create_platform_mesh};
use natsel::collision::mesh::storage::TriangleMesh;
use natsel::math::types::{Pos, Quat, Transform, Vec3};
use natsel::simulation::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::{CompoundDef, CompoundHullDef};
use natsel::types::shape_def::{ShapeDef, SurfaceMaterial};

fn main() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    // --- 静态地形：高度场 ---
    let terrain_body = world.create_body(&BodyDef::default());
    let terrain = create_wave_height_field(64, 64, Vec3::new(200.0, 4.0, 200.0), 1.5, 1.5, false)
        .expect("terrain");
    let _ = world.create_height_field_shape(terrain_body, &ShapeDef::default(), terrain);

    // --- 静态网格：从零手工构造一个斜坡 ---
    let verts = [
        Vec3::new(-10.0, 0.0, -5.0),
        Vec3::new(10.0, 0.0, -5.0),
        Vec3::new(10.0, 3.0, 5.0),
        Vec3::new(-10.0, 3.0, 5.0),
    ];
    let idx: [i32; 6] = [0, 1, 2, 0, 2, 3];
    let opts = MeshBuildOptions {
        weld_tolerance: 0.0,
        use_median_split: true,
        identify_edges: true,
        clockwise_winding: false,
    };
    let ramp = TriangleMesh::new(&verts, &idx, None, opts).expect("ramp");

    let ramp_body = world.create_body(&BodyDef::default());
    let _ = world.create_mesh_shape(ramp_body, &ShapeDef::default(), ramp, Vec3::ONE);

    // --- 静态复合体：一个带柱子的平台 ---
    let mut cdef = CompoundDef::default();
    let mut deck = CompoundHullDef::default();
    deck.hull = Some(make_box_hull(4.0, 0.25, 4.0));
    deck.transform = Transform::IDENTITY;
    deck.material = SurfaceMaterial::default_material();
    cdef.hulls.push(deck);

    let mut pillar = CompoundHullDef::default();
    pillar.hull = create_cylinder(4.0, 0.4, -4.0, 16);
    pillar.transform = Transform::new(Vec3::new(-3.0, 0.0, -3.0), Quat::IDENTITY);
    pillar.material = SurfaceMaterial::default_material();
    cdef.hulls.push(pillar);

    let compound = Compound::new(&cdef).expect("compound");
    let compound_body = world.create_body(&BodyDef::default());
    let _ = world.create_compound_shape(compound_body, &ShapeDef::default(), &compound);

    // --- 动态刚体：各种凸包 ---
    let shapes = [
        make_cube_hull(0.5),
        create_rock(0.6).expect("rock"),
        create_cone(1.2, 0.6, 0.05, 12).expect("cone"),
    ];

    for (i, hull) in shapes.iter().enumerate() {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(i as f64 * 2.0 - 2.0, 12.0, 0.0);
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), hull);
    }

    // 从点云生成的凸包
    let cloud = vec![
        Vec3::new(-0.5, -0.5, -0.5),
        Vec3::new(0.5, -0.5, -0.5),
        Vec3::new(-0.5, 0.5, -0.5),
        Vec3::new(-0.5, -0.5, 0.5),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(0.3, 0.1, 0.7),
    ];
    if let Some(hull) = create_hull(&cloud, 16) {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 16.0, 0.0);
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), &hull);
    }

    // --- 带逐三角形材质的网格 ---
    let grid_mesh = create_grid_mesh(8, 8, 2.0, 3, true).expect("grid");
    let mut md = ShapeDef::default();
    md.materials = vec![
        SurfaceMaterial { friction: 0.9, ..Default::default() },  // 0: 粗糙
        SurfaceMaterial { friction: 0.1, ..Default::default() },  // 1: 冰面
        SurfaceMaterial { friction: 0.6, restitution: 0.7, ..Default::default() }, // 2: 弹跳
    ];
    let grid_body = world.create_body(&BodyDef::default());
    let _ = world.create_mesh_shape(grid_body, &md, grid_mesh, Vec3::ONE);

    let platform_mesh = create_platform_mesh(Vec3::new(15.0, 1.0, 0.0), 2.0, 1.5, 4.0)
        .expect("platform");
    let plat_body = world.create_body(&BodyDef::default());
    let _ = world.create_mesh_shape(plat_body, &ShapeDef::default(), platform_mesh, Vec3::ONE);

    for _ in 0..300 {
        world.step(1.0f32 / 60.0, 4);
    }

    println!("bodies = {}", world.body_count());
    println!("contacts = {}", world.contact_count());
}
```

## 常见错误与性能提示

**半长 vs 全尺寸。** `make_box_hull`、`create_box_mesh` 的 `extent` 都是半长。`create_cylinder(height, ...)` 的 `height` 是全高。这是最容易混的一处，建场景前先确认。

**从点云造凸包忘了处理 `None`。** 共面点集（比如所有点都在 `y = 0`）构不成三维凸包。用 `?` 或 `match` 处理，否则要么 panic 要么静默丢物体。

**网格材质数组长度对不上。** `TriangleMesh::new` 的 `materials` 长度必须精确等于三角形数，不是顶点数。而 `ShapeDef.materials` 的长度要和网格里出现的最大材质索引匹配。

**以为凸包会复制顶点数据。** 相同内容的凸包是共享的。如果你"改了原始 hull 想让已有形状变化"，那是不会生效的 —— 形状持有自己的（共享的）几何，几何不可变。要换几何只能重建形状。

**高度场行列数太小。** `rows`/`cols` 只有 2 时整个地形就是一个格子，物体在上面会显得像踩在棱锥上。地形至少要 16x16，视觉上合理则要 64x64 以上。

**给大范围平坦地面用网格而不是高度场。** 1000x1000 米的网格地面需要两个巨大三角形（BVH 效率差、接触精度低），而高度场用很小的内存就能达到更好的精度。静态大地面优先考虑高度场，或者干脆用一个大盒子凸包。

**动态物体用网格。** 网格不计质量（见 [形状](/docs/shapes)），动态体不会动。而且网格的窄相检测比凸包贵得多。动态物体用凸包。

## 下一步

- [形状](/docs/shapes)：把造好的几何挂上去，配置密度与事件。
- [材质与过滤](/docs/materials)：逐三角形材质、摩擦与碰撞过滤。
- [空间查询](/docs/queries)：对网格和高度场做射线检测。
- [调试绘制](/docs/debug-draw)：把几何可视化出来检查。
