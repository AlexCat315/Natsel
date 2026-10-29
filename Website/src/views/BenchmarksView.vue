<script setup lang="ts">
import { RouterLink } from 'vue-router'

// Historical Natsel measurements, also recorded in PERFORMANCE.md.
const rows = [
  { scene: 'large_pyramid', before: 6185, after: 4286 },
  { scene: 'many_pyramids', before: 6312, after: 4121 },
  { scene: 'joint_grid', before: 7534, after: 4323 },
  { scene: 'convex_pile', before: 56280, after: 22622 },
  { scene: 'spinner', before: 75562, after: 12453 },
  { scene: 'washer', before: 83038, after: 79219 },
  { scene: 'trees25', before: 1464, after: 1420 },
]

const conditions = [
  { label: '机器', value: 'Apple M1 · 8 核 · macOS 27.0.1' },
  { label: '构建', value: 'swift build -c release' },
  { label: '线程', value: 'workerCount = 1' },
  { label: '步长', value: 'dt = 1/60 · 4 子步' },
  { label: '取值', value: '每场景 3 次，取最小值' },
]
</script>

<template>
  <div class="bench-page">
    <section class="b-hero">
      <div class="container">
        <span class="eyebrow reveal">基准性能</span>
        <h1 class="reveal" style="font-size: clamp(32px, 4.5vw, 50px)">记录每一次进步</h1>
        <p class="section-sub reveal">比较 Natsel 优化前后的耗时，在相同场景与配置下追踪自身变化。</p>
      </div>
    </section>

    <section class="container b-body">
      <div class="results glass hud reveal">
        <table>
          <caption>历史优化记录，耗时单位：毫秒。此记录不代表当前版本的性能。</caption>
          <thead>
            <tr><th scope="col">场景</th><th scope="col">优化前</th><th scope="col">优化后</th><th scope="col">提速</th></tr>
          </thead>
          <tbody>
            <tr v-for="row in rows" :key="row.scene">
              <th scope="row">{{ row.scene }}</th>
              <td>{{ row.before.toLocaleString() }}</td>
              <td>{{ row.after.toLocaleString() }}</td>
              <td class="speedup">{{ (row.before / row.after).toFixed(2) }}x</td>
            </tr>
          </tbody>
        </table>
      </div>

      <div class="below-grid">
        <div class="conditions glass reveal">
          <h2>测量条件</h2>
          <dl>
            <div v-for="condition in conditions" :key="condition.label" class="cond-row">
              <dt>{{ condition.label }}</dt><dd>{{ condition.value }}</dd>
            </div>
          </dl>
        </div>
        <div class="note-card glass reveal">
          <h2>如何验证</h2>
          <p>使用相同机器、构建配置、步长和 worker 数，交替运行两个 Natsel 版本，保存每轮耗时与状态哈希。</p>
          <p>同一版本重复运行的结果必须一致；版本之间有意改变求解行为时，应同时检查碰撞、约束、守恒与回放质量。</p>
          <RouterLink to="/docs/determinism" class="more">了解确定性设计 →</RouterLink>
        </div>
      </div>
    </section>
  </div>
</template>

<style scoped>
.bench-page { padding-top: var(--nav-h); }
.b-hero { padding: 60px 0 28px; }
.b-body { padding-bottom: 100px; }
.results { padding: 24px 28px; margin-bottom: 24px; overflow-x: auto; }
table { width: 100%; border-collapse: collapse; font-family: var(--font-mono); font-size: 13px; }
caption { text-align: left; font-family: var(--font-sans); color: var(--text-dim); margin-bottom: 20px; }
th, td { padding: 14px 12px; border-bottom: 1px solid var(--border); text-align: right; white-space: nowrap; }
th:first-child { text-align: left; }
thead th { color: var(--text-faint); font-weight: 500; }
tbody th { color: var(--text-dim); font-weight: 500; }
.speedup { color: var(--primary); }
.below-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; }
.conditions, .note-card { padding: 26px 28px; }
h2 { font-size: 17px; margin-bottom: 14px; }
.cond-row { display: flex; justify-content: space-between; gap: 16px; padding: 8px 0; border-bottom: 1px solid var(--border); font-size: 13px; }
dt { color: var(--text-faint); flex-shrink: 0; }
dd { margin: 0; color: var(--text-dim); text-align: right; font-family: var(--font-mono); font-size: 12px; }
.note-card p { color: var(--text-dim); font-size: 13.5px; margin-bottom: 12px; }
.more { color: var(--primary); font-size: 13.5px; }
@media (max-width: 850px) {
  .below-grid { grid-template-columns: 1fr; }
  .results { padding: 18px 12px; }
}
</style>
