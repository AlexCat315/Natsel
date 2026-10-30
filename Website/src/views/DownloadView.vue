<script setup lang="ts">
import { ref } from 'vue'

const cargoSnippet = `# Cargo.toml
[dependencies]
natsel = { git = "https://github.com/AlexCat315/Natsel", branch = "main" }`

const copied = ref(false)
async function copy() {
  await navigator.clipboard.writeText(cargoSnippet)
  copied.value = true
  setTimeout(() => (copied.value = false), 1600)
}

const ways = [
  {
    title: 'Cargo git 依赖',
    desc: '推荐方式。跟随 main 分支，Cargo.lock 记录实际提交。',
    icon: 'M12 2 2 7l10 5 10-5zM2 17l10 5 10-5M2 12l10 5 10-5',
  },
  {
    title: '本地路径引用',
    desc: '克隆后用 path 依赖引用，便于调试与贡献。',
    icon: 'M15 3h4a2 2 0 0 1 2 2v14a2 2 0 0 1-2 2h-4M10 17l5-5-5-5M15 12H3',
  },
]

const steps = [
  { n: '01', t: '克隆仓库', d: 'git clone https://github.com/AlexCat315/Natsel.git' },
  { n: '02', t: '添加依赖', d: '在 Cargo.toml 的 [dependencies] 中加入 natsel' },
  { n: '03', t: '构建', d: 'cargo build --release' },
  { n: '04', t: '验证', d: 'cargo test，或 cargo run --release --bin benchmarks' },
]
</script>

<template>
  <div class="dl-page">
    <section class="dl-hero">
      <div class="container">
        <span class="eyebrow reveal">获取</span>
        <h1 class="reveal" style="font-size: clamp(32px, 4.5vw, 50px)">获取 Natsel</h1>
        <p class="reveal" style="color: var(--text-dim); max-width: 640px; font-size: 17px">
          Natsel 通过 Cargo 分发，要求 Rust 1.85 及以上（edition 2024），无第三方依赖。
        </p>
      </div>
    </section>

    <section class="container dl-body">
      <div class="ways-grid">
        <div v-for="(w, i) in ways" :key="w.title" class="way glass hud reveal" :data-reveal-delay="i * 100">
          <div class="way-icon">
            <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
              <path :d="w.icon" />
            </svg>
          </div>
          <h3>{{ w.title }}</h3>
          <p>{{ w.desc }}</p>
        </div>
      </div>

      <div class="snippet-card glass hud reveal">
        <div class="snippet-head">
          <span>Cargo.toml</span>
          <button class="copy-btn" @click="copy">{{ copied ? '已复制 ✓' : '复制' }}</button>
        </div>
        <pre class="snippet-body">{{ cargoSnippet }}</pre>
      </div>

      <div class="steps-grid">
        <div v-for="(s, i) in steps" :key="s.n" class="step glass reveal" :data-reveal-delay="i * 80">
          <span class="step-n">{{ s.n }}</span>
          <h4>{{ s.t }}</h4>
          <code class="step-d">{{ s.d }}</code>
        </div>
      </div>

      <div class="links-card glass reveal">
        <h3>相关链接</h3>
        <div class="links">
          <a href="https://github.com/AlexCat315/Natsel" target="_blank" rel="noopener">GitHub 仓库</a>
          <a href="https://github.com/AlexCat315/Natsel/blob/main/DEVELOPMENT.md" target="_blank" rel="noopener">开发说明</a>
          <a href="https://github.com/AlexCat315/Natsel/blob/main/PERFORMANCE.md" target="_blank" rel="noopener">性能报告原文</a>
        </div>
      </div>
    </section>
  </div>
</template>

<style scoped>
.dl-page { padding-top: var(--nav-h); }
.dl-hero { padding: 60px 0 20px; }
.dl-body { padding-bottom: 100px; }

.ways-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 16px;
  margin: 30px 0 22px;
}
.way { padding: 26px 28px; }
.way-icon {
  width: 44px; height: 44px; border-radius: 12px;
  display: flex; align-items: center; justify-content: center;
  background: linear-gradient(135deg, rgba(74, 222, 128, 0.18), rgba(163, 230, 53, 0.1));
  color: var(--primary);
  margin-bottom: 14px;
}
.way h3 { font-size: 18px; margin-bottom: 7px; }
.way p { font-size: 14px; color: var(--text-dim); }

.snippet-card { overflow: hidden; margin-bottom: 26px; }
.snippet-head {
  display: flex;
  justify-content: space-between;
  padding: 11px 20px;
  background: rgba(16, 32, 24, 0.55);
  border-bottom: 1px solid var(--border);
  font-family: var(--font-mono);
  font-size: 12.5px;
  color: var(--text-faint);
}
.copy-btn {
  background: none; border: none;
  color: var(--text-dim); font-size: 12px; cursor: pointer;
}
.copy-btn:hover { color: var(--primary); }
.snippet-body {
  padding: 22px 26px;
  font-family: var(--font-mono);
  font-size: 13.5px;
  line-height: 1.8;
  color: #dcf2e2;
}

.steps-grid {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 14px;
  margin-bottom: 26px;
}
.step { padding: 22px; }
.step-n {
  font-family: var(--font-mono);
  font-size: 20px;
  font-weight: 700;
  color: var(--primary);
  opacity: 0.65;
}
.step h4 { font-size: 15px; margin: 10px 0 8px; }
.step-d {
  font-size: 11.5px;
  color: var(--text-dim);
  word-break: break-all;
}

.links-card { padding: 26px 30px; }
.links-card h3 { font-size: 17px; margin-bottom: 12px; }
.links { display: flex; flex-direction: column; gap: 4px; }
.links a {
  color: var(--text-dim);
  font-size: 14px;
  padding: 7px 0;
  border-bottom: 1px solid var(--border);
}
.links a:hover { color: var(--primary); }

@media (max-width: 800px) {
  .ways-grid, .steps-grid { grid-template-columns: 1fr 1fr; }
}
@media (max-width: 520px) {
  .ways-grid, .steps-grid { grid-template-columns: 1fr; }
}
</style>
