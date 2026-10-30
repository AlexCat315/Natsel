# Natsel Website

Natsel 官方网站，使用 Vue 3、TypeScript 与 Vite 构建。

## 开发

```sh
npm install
npm run dev
npm run build
npm run preview
```

开发服务器默认使用 http://localhost:5175，生产构建输出到 `dist/`。

## 页面

- 首页：引擎介绍、能力概览、代码示例与集成入口。
- 特性：碰撞、模拟、关节、角色、世界服务和基础设施。
- 基准：Natsel 自身优化前后的历史耗时，数据与测量条件见 [性能记录](../PERFORMANCE.md)。
- 获取：Cargo 集成、本地路径引用与开发说明。
- 文档：快速开始、核心概念与确定性设计。

文档位于 `src/content/*.md`，侧边栏条目在 `src/utils/docs.ts` 登记。
共用组件位于 `src/components/`，页面位于 `src/views/`。

## 设计

近黑绿底、终端绿与 HUD 边角装饰，配合数字滚动和滚动入场动效。
全站支持响应式布局。

## 许可

与 Natsel 项目一致。
