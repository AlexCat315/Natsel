<script setup lang="ts">
import { RouterLink } from 'vue-router'

const year = new Date().getFullYear()

const columns = [
  {
    title: '产品',
    links: [
      { label: '功能特性', to: '/features' },
      { label: '基准性能', to: '/benchmarks' },
      { label: '获取 Natsel', to: '/download' },
    ],
  },
  {
    title: '资源',
    links: [
      { label: '快速开始', to: '/docs/getting-started' },
      { label: '安装与集成', to: '/docs/installation' },
      { label: '核心概念', to: '/docs/concepts' },
      { label: '确定性', to: '/docs/determinism' },
      { label: 'API 速查', to: '/docs/api-reference' },
      { label: '常见问题', to: '/docs/faq' },
    ],
  },
  {
    title: '专题',
    links: [
      { label: '刚体与形状', to: '/docs/bodies' },
      { label: '关节', to: '/docs/joints' },
      { label: '虚拟角色', to: '/docs/character' },
      { label: '车辆', to: '/docs/vehicle' },
      { label: '软体', to: '/docs/soft-body' },
      { label: '快照与状态哈希', to: '/docs/snapshots' },
    ],
  },
]
</script>

<template>
  <footer class="footer">
    <div class="container">
      <div class="footer-grid">
        <div class="footer-brand">
          <div class="brand-row">
            <svg viewBox="0 0 64 64" width="28" height="28">
              <circle cx="20" cy="22" r="7" fill="none" stroke="#4ade80" stroke-width="4" />
              <rect x="34" y="14" width="14" height="14" rx="2" fill="none" stroke="#a3e635" stroke-width="4" />
              <path d="M10 46h44" stroke="#fbbf24" stroke-width="4" stroke-linecap="round" />
            </svg>
            <span>Natsel</span>
          </div>
          <p>用 Rust 编写的快速、确定性物理引擎，为游戏与实时模拟而生。</p>
          <div class="socials">
            <a href="https://github.com/AlexCat315/Natsel" target="_blank" rel="noopener" aria-label="GitHub">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 .5C5.7.5.5 5.7.5 12c0 5.1 3.3 9.4 7.9 10.9.6.1.8-.3.8-.6v-2c-3.2.7-3.9-1.5-3.9-1.5-.5-1.3-1.3-1.7-1.3-1.7-1.1-.7.1-.7.1-.7 1.2.1 1.8 1.2 1.8 1.2 1 1.8 2.7 1.3 3.4 1 .1-.8.4-1.3.7-1.6-2.6-.3-5.3-1.3-5.3-5.8 0-1.3.5-2.3 1.2-3.1-.1-.3-.5-1.5.1-3.1 0 0 1-.3 3.3 1.2a11.5 11.5 0 0 1 6 0C17.3 4.7 18.3 5 18.3 5c.6 1.6.2 2.8.1 3.1.8.8 1.2 1.8 1.2 3.1 0 4.5-2.7 5.5-5.3 5.8.4.4.8 1.1.8 2.2v3.3c0 .3.2.7.8.6 4.6-1.5 7.9-5.8 7.9-10.9C23.5 5.7 18.3.5 12 .5z" />
              </svg>
            </a>
          </div>
        </div>

        <div v-for="col in columns" :key="col.title" class="footer-col">
          <h4>{{ col.title }}</h4>
          <RouterLink v-for="l in col.links" :key="l.label" :to="l.to">{{ l.label }}</RouterLink>
        </div>

        <div class="footer-col">
          <h4>生态</h4>
          <a href="https://github.com/AlexCat315/Cove" target="_blank" rel="noopener">Cove UI 框架</a>
          <a href="https://github.com/AlexCat315/Diffuse" target="_blank" rel="noopener">Diffuse 工作站</a>
        </div>
      </div>

      <div class="footer-bottom">
        <span>© {{ year }} Natsel.</span>
        <span>Rust · Wide Lanes · Multithreaded Determinism</span>
      </div>
    </div>
  </footer>
</template>

<style scoped>
.footer {
  position: relative;
  z-index: 1;
  border-top: 1px solid var(--border);
  background: rgba(4, 9, 6, 0.8);
  padding: 72px 0 32px;
  margin-top: 40px;
}
.footer-grid { display: grid; grid-template-columns: 1.6fr 1fr 1fr 1fr 1fr; gap: 40px; }
.brand-row { display: flex; align-items: center; gap: 10px; font-size: 20px; font-weight: 800; margin-bottom: 14px; }
.footer-brand p { color: var(--text-dim); font-size: 14px; max-width: 300px; margin-bottom: 18px; }
.socials { display: flex; gap: 14px; }
.socials a { color: var(--text-faint); transition: color 0.2s, transform 0.2s; }
.socials a:hover { color: var(--primary); transform: translateY(-2px); }
.footer-col h4 { font-size: 14px; margin-bottom: 16px; }
.footer-col a { display: block; color: var(--text-dim); font-size: 14px; padding: 5px 0; }
.footer-col a:hover { color: var(--primary); }
.footer-bottom {
  margin-top: 56px;
  padding-top: 24px;
  border-top: 1px solid var(--border);
  display: flex;
  justify-content: space-between;
  color: var(--text-faint);
  font-size: 13px;
}
@media (max-width: 1100px) {
  .footer-grid { grid-template-columns: 1fr 1fr 1fr; gap: 36px; }
  .footer-brand { grid-column: 1 / -1; }
}
@media (max-width: 900px) {
  .footer-grid { grid-template-columns: 1fr 1fr; gap: 36px; }
  .footer-sitemap { grid-template-columns: 1fr 1fr; }
  .footer-brand { grid-column: 1 / -1; }
  .footer-bottom { flex-direction: column; gap: 8px; }
}
</style>
