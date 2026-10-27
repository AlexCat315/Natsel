<script setup lang="ts">
import { onMounted, ref } from 'vue'
import { RouterLink } from 'vue-router'

interface Row {
  scene: string
  c: number
  swift: number
  ratio: number
}

const rows: Row[] = [
  { scene: 'large_pyramid', c: 1537.5, swift: 4286.2, ratio: 2.79 },
  { scene: 'many_pyramids', c: 1634.9, swift: 4120.7, ratio: 2.52 },
  { scene: 'joint_grid', c: 864.6, swift: 4322.7, ratio: 5.0 },
  { scene: 'trees25', c: 377.1, swift: 1419.9, ratio: 3.77 },
  { scene: 'trees50', c: 173.5, swift: 669.2, ratio: 3.86 },
  { scene: 'trees100', c: 101.5, swift: 411.0, ratio: 4.05 },
  { scene: 'sleep', c: 597.3, swift: 2150.2, ratio: 3.6 },
  { scene: 'spinner', c: 2825.6, swift: 12453.0, ratio: 4.41 },
  { scene: 'convex_pile', c: 5819.2, swift: 22622.1, ratio: 3.89 },
  { scene: 'washer', c: 17802.5, swift: 79218.5, ratio: 4.45 },
]

const totals = { c: 31733.7, swift: 131673.5, ratio: 4.15 }

// Normalize bars against the max Swift value in each row is unfair;
// instead scale both series by the global max (washer C) so lengths
// are visually comparable across scenes.
const globalMax = Math.max(...rows.map((r) => r.swift))
function width(v: number) {
  return `${(v / globalMax) * 100}%`
}

const animate = ref(false)
onMounted(() => {
  requestAnimationFrame(() => {
    setTimeout(() => (animate.value = true), 150)
  })
})

const conditions = [
  { k: '机器', v: 'Apple M1 · 8 核 · macOS 27.0.1' },
  { k: 'C 构建', v: '-O3 -DNDEBUG · NEON 4 宽求解器' },
  { k: 'Swift 构建', v: 'swift build -c release' },
  { k: '线程', v: '单线程 workerCount = 1' },
  { k: '步长', v: 'dt = 1/60 · 4 子步' },
  { k: '取值', v: '每场景 3 次，取最小值' },
]
</script>

<template>
  <div class="bench-page">
    <section class="b-hero">
      <div class="container">
        <span class="eyebrow reveal">基准性能</span>
        <h1 class="reveal" style="font-size: clamp(32px, 4.5vw, 50px)">用数据说话</h1>
        <p class="reveal" style="color: var(--text-dim); max-width: 700px; font-size: 17px">
          Natsel 与原始 C 参考实现（Box3D）在十个标准场景下的单线程耗时对比。
        </p>
      </div>
    </section>

    <section class="container b-body">
      <div class="chart-card glass hud reveal">
        <div class="legend">
          <span><i class="swatch c"></i>C 参考 (ms)</span>
          <span><i class="swatch swift"></i>Swift (ms)</span>
        </div>

        <div class="bars">
          <div v-for="(r, i) in rows" :key="r.scene" class="bar-row">
            <div class="scene-name">{{ r.scene }}</div>
            <div class="bar-track">
              <div class="bar c-bar" :class="{ grow: animate }" :style="{ width: animate ? width(r.c) : '0%', transitionDelay: `${i * 70}ms` }">
                <span class="bar-label">{{ r.c.toLocaleString() }}</span>
              </div>
              <div class="bar swift-bar" :class="{ grow: animate }" :style="{ width: animate ? width(r.swift) : '0%', transitionDelay: `${i * 70 + 120}ms` }">
                <span class="bar-label">{{ r.swift.toLocaleString() }}</span>
              </div>
            </div>
            <div class="ratio">{{ r.ratio.toFixed(2) }}x</div>
          </div>
        </div>

        <div class="total-row">
          <div class="scene-name">合计</div>
          <div class="bar-track">
            <div class="bar c-bar" :style="{ width: animate ? width(totals.c) : '0%' }">
              <span class="bar-label">{{ totals.c.toLocaleString() }}</span>
            </div>
            <div class="bar swift-bar" :style="{ width: animate ? width(totals.swift) : '0%' }">
              <span class="bar-label">{{ totals.swift.toLocaleString() }}</span>
            </div>
          </div>
          <div class="ratio total-ratio">{{ totals.ratio }}x</div>
        </div>
      </div>

      <div class="below-grid">
        <div class="conditions glass reveal">
          <h3>测量条件</h3>
          <div v-for="c in conditions" :key="c.k" class="cond-row">
            <span class="cond-k">{{ c.k }}</span>
            <span class="cond-v">{{ c.v }}</span>
          </div>
        </div>

        <div class="note-card glass reveal">
          <h3>如实说明</h3>
          <p>
            当前整体差距约 <strong>4.15 倍</strong>，尚未达到与 C 参考 5% 以内的目标。
            项目已通过消除热循环中的值拷贝、求解器缓冲区复用与栈上 scratch 等手段，
            把差距从约 5.0x 缩小到 4.15x，优化仍在继续。
          </p>
          <p class="dim">
            硬指标是同版本/同配置下的可重复性、worker 数无关的确定性，以及独立测试的碰撞、约束、
            守恒与回放质量。
          </p>
          <RouterLink to="/docs/determinism" class="more">了解确定性设计 →</RouterLink>
        </div>
      </div>
    </section>
  </div>
</template>

<style scoped>
.bench-page { padding-top: var(--nav-h); }
.b-hero { padding: 60px 0 20px; }
.b-body { padding-bottom: 100px; }

.chart-card { padding: 30px 32px; margin-bottom: 24px; }
.legend {
  display: flex;
  gap: 24px;
  font-size: 13px;
  color: var(--text-dim);
  margin-bottom: 26px;
}
.swatch {
  display: inline-block;
  width: 12px;
  height: 12px;
  border-radius: 3px;
  margin-right: 7px;
  vertical-align: -1px;
}
.swatch.c { background: #fbbf24; }
.swatch.swift { background: linear-gradient(120deg, #4ade80, #a3e635); }

.bars { display: flex; flex-direction: column; gap: 14px; }
.bar-row {
  display: grid;
  grid-template-columns: 130px 1fr 64px;
  gap: 14px;
  align-items: center;
}
.scene-name {
  font-family: var(--font-mono);
  font-size: 12.5px;
  color: var(--text-dim);
}
.bar-track {
  display: flex;
  flex-direction: column;
  gap: 4px;
}
.bar {
  height: 19px;
  border-radius: 5px;
  transition: width 1s cubic-bezier(0.2, 0.8, 0.2, 1);
  position: relative;
  min-width: 2px;
}
.c-bar {
  background: linear-gradient(90deg, #f59e0b, #fbbf24);
  opacity: 0.85;
}
.swift-bar {
  background: linear-gradient(90deg, #22c55e, #a3e635);
  box-shadow: 0 0 16px -4px rgba(74, 222, 128, 0.6);
}
.bar-label {
  position: absolute;
  right: 8px;
  top: 50%;
  transform: translateY(-50%);
  font-family: var(--font-mono);
  font-size: 10.5px;
  color: rgba(0, 0, 0, 0.7);
  font-weight: 700;
  white-space: nowrap;
}
.ratio {
  font-family: var(--font-mono);
  font-size: 13px;
  color: var(--accent);
  font-weight: 700;
  text-align: right;
}

.total-row {
  display: grid;
  grid-template-columns: 130px 1fr 64px;
  gap: 14px;
  align-items: center;
  margin-top: 20px;
  padding-top: 18px;
  border-top: 1px dashed var(--border-strong);
}
.total-ratio { color: var(--primary); }

.below-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; }
.conditions { padding: 26px 28px; }
.conditions h3 { font-size: 17px; margin-bottom: 14px; }
.cond-row {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  padding: 8px 0;
  border-bottom: 1px solid var(--border);
  font-size: 13px;
}
.cond-k { color: var(--text-faint); flex-shrink: 0; }
.cond-v { color: var(--text-dim); text-align: right; font-family: var(--font-mono); font-size: 12px; }

.note-card { padding: 26px 28px; }
.note-card h3 { font-size: 17px; margin-bottom: 12px; }
.note-card p { font-size: 13.5px; color: var(--text-dim); margin-bottom: 12px; }
.note-card strong { color: var(--primary); }
.note-card .dim { font-size: 12.5px; }
.more { color: var(--primary); font-size: 13.5px; }

@media (max-width: 850px) {
  .below-grid { grid-template-columns: 1fr; }
  .bar-row, .total-row { grid-template-columns: 96px 1fr 48px; }
  .scene-name { font-size: 11px; }
}
</style>
