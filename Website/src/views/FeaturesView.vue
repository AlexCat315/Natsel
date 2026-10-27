<script setup lang="ts">
import { RouterLink } from 'vue-router'

const categories = [
  {
    title: '碰撞',
    tint: '#4ade80',
    items: [
      '凸包（Hull）存储与构建器',
      '三角形网格（Mesh）与 BVH',
      '高度场（HeightField）',
      '复合形状（Compound）',
      'GJK 距离与流形裁剪',
      '宽相位动态树（Dynamic Tree）',
      'AABB 射线与形状投射',
    ],
  },
  {
    title: '模拟',
    tint: '#a3e635',
    items: [
      '刚体动力学与岛屿求解',
      '约束图与图着色',
      '宽 SIMD 接触求解器',
      '位置修正与速度迭代',
      '连续碰撞（CCD）',
      '传感器（Sensor）',
      '休眠（Sleeping）',
    ],
  },
  {
    title: '关节',
    tint: '#fbbf24',
    items: [
      '旋转关节（Revolute）',
      '六自由度关节（6DOF）',
      '轮子关节（Wheel）',
      '路径关节（Path）',
      '约束驱动与马达',
    ],
  },
  {
    title: '角色与生物',
    tint: '#86efac',
    items: [
      '虚拟角色（Character Virtual）',
      '车辆与传动系统（Vehicle）',
      '骨骼与骨骼映射（Skeleton）',
      '布娃娃（Ragdoll）',
      '骨骼动画（Skeletal Animation）',
      '软体（Soft Body）',
    ],
  },
  {
    title: '世界服务',
    tint: '#67e8f9',
    items: [
      '多线程调度器',
      '录制与回放（Recording / Replay）',
      '快照（Snapshots）',
      '名称缓存与 ID 池',
      '确定性状态哈希',
    ],
  },
  {
    title: '基础设施',
    tint: '#c4b5fd',
    items: [
      'Vec3 / Quat / Matrix 数学库',
      '竞技场、位集、哈希表容器',
      'rapidhash 哈希',
      '跨平台线程与原子',
      '可移植宽转置（无 simd 平台）',
    ],
  },
]
</script>

<template>
  <div class="features-page">
    <section class="f-hero">
      <div class="container">
        <span class="eyebrow reveal">功能特性</span>
        <h1 class="reveal" style="font-size: clamp(32px, 4.5vw, 50px)">完整的物理能力清单</h1>
        <p class="reveal" style="color: var(--text-dim); max-width: 680px; font-size: 17px">
          从碰撞检测到世界服务，Natsel 以清晰的分层构建，每个模块都有独立测试覆盖。
        </p>
      </div>
    </section>

    <section class="container f-grid">
      <div v-for="(cat, i) in categories" :key="cat.title" class="f-card glass hud reveal" :data-reveal-delay="(i % 2) * 90">
        <div class="f-head">
          <h2>{{ cat.title }}</h2>
          <span class="f-count" :style="{ color: cat.tint }">{{ cat.items.length }}</span>
        </div>
        <ul class="f-list">
          <li v-for="it in cat.items" :key="it">
            <span class="bullet" :style="{ background: cat.tint }"></span>
            {{ it }}
          </li>
        </ul>
      </div>
    </section>

    <section class="container">
      <div class="f-cta glass reveal">
        <h2>想了解这些能力怎么用？</h2>
        <p>文档与示例已准备好。</p>
        <RouterLink to="/docs/getting-started" class="btn btn-primary">阅读文档</RouterLink>
      </div>
    </section>
  </div>
</template>

<style scoped>
.features-page { padding-top: var(--nav-h); }
.f-hero { padding: 60px 0 20px; }
.f-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 18px;
  padding: 40px 0 60px;
}
.f-card { padding: 26px 26px; }
.f-head {
  display: flex;
  justify-content: space-between;
  align-items: baseline;
  margin-bottom: 16px;
}
.f-head h2 { font-size: 20px; }
.f-count {
  font-family: var(--font-mono);
  font-size: 15px;
  font-weight: 700;
}
.f-list { list-style: none; }
.f-list li {
  display: flex;
  align-items: center;
  gap: 10px;
  font-size: 13.5px;
  color: var(--text-dim);
  padding: 6px 0;
}
.bullet {
  width: 6px;
  height: 6px;
  border-radius: 2px;
  flex-shrink: 0;
}
.f-cta {
  text-align: center;
  padding: 54px 30px;
}
.f-cta h2 { font-size: 25px; margin-bottom: 8px; }
.f-cta p { color: var(--text-dim); margin-bottom: 22px; }

@media (max-width: 950px) {
  .f-grid { grid-template-columns: repeat(2, 1fr); }
}
@media (max-width: 620px) {
  .f-grid { grid-template-columns: 1fr; }
}
</style>
