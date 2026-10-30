<script setup lang="ts">
import { onMounted, reactive } from 'vue'
import { RouterLink } from 'vue-router'
import CodeBlock from '@/components/CodeBlock.vue'

const quickCode = `use natsel::collision::hull::make_box_hull;
use natsel::math::types::Pos;
use natsel::simulation::world_runtime::World;
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::shape_def::ShapeDef;

let mut world = World::default();
let shape_def = ShapeDef::default();

// 地面与一个盒子
let mut ground_def = BodyDef::default();
ground_def.position = Pos::new(0.0, -1.0, 0.0);
let ground = world.create_body(&ground_def);
world.create_hull_shape(ground, &shape_def, &make_box_hull(20.0, 1.0, 20.0));

let mut box_def = BodyDef::default();
box_def.type_ = BodyType::DynamicBody;
box_def.position = Pos::new(0.0, 5.0, 0.0);
let body = world.create_body(&box_def);
world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

// 以固定步长推进
for _ in 0..60 {
    world.step(1.0 / 60.0, 4);
}

let position = world.body_position(body);
println!("{}", position.y);  // 正在下落…`

const highlights = [
  {
    icon: 'M12 2 2 7l10 5 10-5zM2 17l10 5 10-5',
    title: '刚体与碰撞',
    desc: '凸包、网格、高度场、复合形状；GJK 距离、动态树宽相位。',
  },
  {
    icon: 'M5 12h14M12 5l7 7-7 7',
    title: '连续碰撞 CCD',
    desc: '高速物体不再穿透，CCD 冲击事件每步重置、可回放。',
  },
  {
    icon: 'M13 2 3 14h7l-1 8 10-12h-7z',
    title: '宽 SIMD 求解器',
    width: true,
    desc: '多线程岛屿 + 宽接触求解，核心循环为 SIMD 批量处理。',
  },
  {
    icon: 'M8 3H5a2 2 0 0 0-2 2v3m18 0V5a2 2 0 0 0-2-2h-3M3 16v3a2 2 0 0 0 2 2h3',
    title: '车辆 / 角色 / 骨骼',
    desc: '车辆传动、虚拟角色、骨骼映射、布娃娃与软体一应俱全。',
  },
]

// animated counters
const stats = reactive([
  { value: 18, suffix: '', label: '基准场景', current: 0 },
  { value: 26, suffix: '', label: '引擎模块', current: 0 },
  { value: 60, suffix: 'Hz', label: '固定步长', current: 0 },
  { value: 4, suffix: 'x', label: '多线程并行', current: 0 },
])

onMounted(() => {
  const duration = 1400
  const start = performance.now()
  const tick = (now: number) => {
    const p = Math.min(1, (now - start) / duration)
    const eased = 1 - Math.pow(1 - p, 3)
    stats.forEach((s) => (s.current = s.value * eased))
    if (p < 1) requestAnimationFrame(tick)
  }
  requestAnimationFrame(tick)
})
</script>

<template>
  <!-- HERO -->
  <section class="hero">
    <div class="container">
      <div class="hero-top">
        <div class="hero-badge reveal">
          <span class="dot"></span> Rust · 确定性 · 多线程 · 100% 原生
        </div>
        <h1 class="hero-title reveal">
          让世界<br />
          <span class="gradient-text">按物理规律运转</span>
        </h1>
        <p class="hero-sub reveal">
          Natsel 是用 Rust 编写的快速、确定性物理引擎。刚体、车辆、角色、骨骼、软体、关节、连续碰撞
          与多线程宽 SIMD 求解器——为游戏与实时模拟提供坚实底座。
        </p>
        <div class="hero-cta reveal">
          <RouterLink to="/download" class="btn btn-primary">
            获取 Natsel
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5">
              <path d="M5 12h14M12 5l7 7-7 7" />
            </svg>
          </RouterLink>
          <RouterLink to="/docs/getting-started" class="btn btn-ghost">快速开始</RouterLink>
        </div>
      </div>

      <div class="hero-stats reveal">
        <div v-for="s in stats" :key="s.label" class="stat">
          <strong>{{ Math.round(s.current) }}{{ s.suffix }}</strong>
          <span>{{ s.label }}</span>
        </div>
      </div>
    </div>
  </section>

  <!-- HIGHLIGHTS -->
  <section class="section">
    <div class="container">
      <div class="reveal">
        <span class="eyebrow">能力概览</span>
        <h2 class="section-title">一个引擎，覆盖大多数物理需求</h2>
      </div>
      <div class="hl-grid">
        <div v-for="(h, i) in highlights" :key="h.title" class="hl-card glass hud reveal" :data-reveal-delay="i * 90">
          <div class="hl-icon">
            <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
              <path :d="h.icon" />
            </svg>
          </div>
          <h3>{{ h.title }}</h3>
          <p>{{ h.desc }}</p>
        </div>
      </div>
    </div>
  </section>

  <!-- CODE -->
  <section class="section code-section">
    <div class="container code-layout">
      <div class="code-copy reveal">
        <span class="eyebrow">简洁 API</span>
        <h2 class="section-title">十几行，跑起一个世界</h2>
        <p class="section-sub">b3 风格的 API：`Vec3`、`Transform`、`World`，配合自由函数与类型化的 `BodyDef`，直观且可发现。</p>
        <RouterLink to="/docs/concepts" class="btn btn-ghost">学习核心概念</RouterLink>
      </div>
      <div class="reveal">
        <CodeBlock :code="quickCode" title="hello-physics.rs" />
      </div>
    </div>
  </section>

  <!-- CTA -->
  <section class="section">
    <div class="container reveal">
      <div class="cta-box glass hud">
        <h2>为你的世界注入物理</h2>
        <p>通过 Cargo 加入 Natsel，立即开始。</p>
        <div class="hero-cta" style="justify-content: center">
          <RouterLink to="/download" class="btn btn-primary">获取 Natsel</RouterLink>
          <RouterLink to="/benchmarks" class="btn btn-ghost">查看性能数据</RouterLink>
        </div>
      </div>
    </div>
  </section>
</template>

<style scoped>
.hero {
  padding: calc(var(--nav-h) + 70px) 0 30px;
}
.hero-top {
  max-width: 760px;
}
.hero-badge {
  display: inline-flex;
  align-items: center;
  gap: 9px;
  padding: 7px 16px;
  border-radius: 999px;
  border: 1px solid var(--border-strong);
  background: var(--surface);
  font-size: 13px;
  color: var(--text-dim);
  margin-bottom: 26px;
}
.hero-badge .dot {
  width: 8px; height: 8px; border-radius: 50%;
  background: var(--primary);
  box-shadow: 0 0 8px var(--primary);
}
.hero-title { font-size: clamp(40px, 5.6vw, 68px); margin-bottom: 22px; }
.hero-sub { font-size: 17px; color: var(--text-dim); max-width: 600px; margin-bottom: 34px; }
.hero-cta { display: flex; gap: 14px; flex-wrap: wrap; }

.hero-stats {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 20px;
  margin-top: 64px;
  padding: 28px;
  border: 1px solid var(--border);
  border-radius: var(--radius-lg);
  background: var(--surface);
}
.stat strong {
  font-size: 32px;
  font-weight: 800;
  font-family: var(--font-mono);
  background: var(--gradient);
  -webkit-background-clip: text;
  background-clip: text;
  color: transparent;
}
.stat span { font-size: 13px; color: var(--text-faint); }

.hl-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 18px;
  margin-top: 44px;
}
.hl-card { padding: 28px 28px; transition: transform 0.3s, border-color 0.3s; }
.hl-card:hover { transform: translateY(-5px); border-color: var(--border-strong); }
.hl-icon {
  width: 46px; height: 46px; border-radius: 12px;
  display: flex; align-items: center; justify-content: center;
  background: linear-gradient(135deg, rgba(74, 222, 128, 0.18), rgba(163, 230, 53, 0.1));
  color: var(--primary);
  margin-bottom: 16px;
}
.hl-card h3 { font-size: 19px; margin-bottom: 8px; }
.hl-card p { color: var(--text-dim); font-size: 14px; }

.code-section { background: linear-gradient(180deg, transparent, rgba(10, 18, 13, 0.6), transparent); }
.code-layout { display: grid; grid-template-columns: 0.9fr 1.1fr; gap: 48px; align-items: center; }

.cta-box { text-align: center; padding: 64px 40px; }
.cta-box h2 { font-size: clamp(26px, 3.5vw, 38px); margin-bottom: 10px; }
.cta-box p { color: var(--text-dim); margin-bottom: 28px; }

@media (max-width: 900px) {
  .hero-stats { grid-template-columns: repeat(2, 1fr); }
  .hl-grid { grid-template-columns: 1fr; }
  .code-layout { grid-template-columns: 1fr; }
}
</style>
