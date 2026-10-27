# Natsel Website

Natsel 官方网站，使用 **Vue 3 + TypeScript + Vite** 构建。

> 上游项目：[Natsel](../) — 用 Swift 编写的快速、确定性物理引擎：刚体、车辆、角色、骨骼、软体、关节、CCD 与多线程宽 SIMD 求解器。

## 技术栈

| 类别 | 选型 |
|------|------|
| 框架 | Vue 3（`<script setup>`） |
| 语言 | TypeScript 5（strict） |
| 构建 | Vite 5 |
| 路由 | Vue Router 4 |
| 状态 | Pinia |
| 动效 | GSAP、Canvas 2D、CSS 动画 |
| 文档 | Markdown（`?raw` 导入）+ marked 渲染 |

## 快速开始

```bash
npm install
npm run dev        # 开发服务器 http://localhost:5175
```

## 构建与预览

```bash
npm run build      # 类型检查 + 生产构建（输出到 dist/）
npm run preview    # 预览生产构建
```

## 目录结构

```
Website/
├── public/
│   └── favicon.svg
├── src/
│   ├── components/
│   │   ├── TheNav.vue          # 顶部导航
│   │   ├── TheFooter.vue       # 页脚
│   │   ├── PhysicsCanvas.vue   # 交互式物理演示画布
│   │   └── CodeBlock.vue       # 代码块（高亮 + 复制）
│   ├── content/                # 文档 Markdown
│   │   ├── getting-started.md
│   │   ├── concepts.md
│   │   └── determinism.md
│   ├── directives/
│   │   └── reveal.ts           # v-reveal 滚动入场
│   ├── physics/
│   │   └── engine.ts           # 迷你 2D 刚体物理引擎（冲量 + SAT）
│   ├── router/index.ts
│   ├── styles/main.css         # Engineering HUD 主题
│   ├── utils/
│   │   ├── docs.ts             # 文档目录清单
│   │   ├── highlight.ts        # 语法高亮
│   │   └── markdown.ts         # marked 渲染
│   ├── views/
│   │   ├── HomeView.vue        # 首页（含实时物理演示）
│   │   ├── FeaturesView.vue    # 功能特性清单
│   │   ├── BenchmarksView.vue   # 基准性能（真实数据图表）
│   │   ├── DownloadView.vue    # 获取方式
│   │   ├── DocsView.vue        # 文档站
│   │   └── NotFoundView.vue    # 404
│   ├── App.vue
│   ├── env.d.ts
│   └── main.ts
├── index.html
├── package.json
├── tsconfig.json
└── vite.config.ts
```

## 页面与功能

- **首页**：Hero、数字滚动、实时物理演示（四场景 + 拖拽发射）、能力概览、代码示例、CTA。
- **特性**：碰撞 / 模拟 / 关节 / 角色与生物 / 世界服务 / 基础设施六大类清单。
- **基准**：十个标准场景下 C 参考 vs Swift 的真实耗时对比条形图（数据来自 `PERFORMANCE.md`），含测量条件与如实说明。
- **获取**：SwiftPM 集成、本地路径引用、四步引导、相关链接。
- **文档站**：三栏布局、Markdown 渲染、代码块复制、翻页导航、移动端抽屉。

## 实时物理演示

`src/physics/engine.ts` 是一个独立的迷你 2D 刚体引擎：

- 圆形与盒子（OBB），重力、摩擦、恢复
- 圆-圆、圆-盒、盒-盒（SAT）碰撞与冲量求解
- 距离关节（链条）、位置修正、多次迭代
- 场景：金字塔、球堆、链条、粒子雨；点击或拖拽发射物体

它是演示用途，与 Natsel 3D 引擎采用相同的冲量求解思路，但并非其移植。

## 设计与动效

- **主题：Engineering HUD（工程仪表）**——近黑绿底、终端绿/黄绿/琥珀色、HUD 边角装饰与工程网格。
- 物理世界实时渲染、碰撞发光、拖拽瞄准、数字滚动、滚动入场。
- 全站响应式。

## 数据说明

基准页数据为单线程（`workerCount=1`）下 Apple M1 的测量结果，整体差距约 4.15 倍，尚未达到 5% 以内目标，页面已如实标注。详细方法见上游 `PERFORMANCE.md`。

## 如何修改文档

编辑 `src/content/*.md`，并在 `src/utils/docs.ts` 中登记侧边栏条目即可。

## 许可

与上游 Natsel 一致。
