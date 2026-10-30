import { createRouter, createWebHistory } from 'vue-router'

const router = createRouter({
  history: createWebHistory(),
  routes: [
    {
      path: '/',
      name: 'home',
      component: () => import('@/views/HomeView.vue'),
      meta: { title: 'Natsel · Rust 物理引擎' },
    },
    {
      path: '/features',
      name: 'features',
      component: () => import('@/views/FeaturesView.vue'),
      meta: { title: '功能特性 · Natsel' },
    },
    {
      path: '/benchmarks',
      name: 'benchmarks',
      component: () => import('@/views/BenchmarksView.vue'),
      meta: { title: '基准性能 · Natsel' },
    },
    {
      path: '/download',
      name: 'download',
      component: () => import('@/views/DownloadView.vue'),
      meta: { title: '获取 · Natsel' },
    },
    {
      path: '/docs/:slug?',
      name: 'docs',
      component: () => import('@/views/DocsView.vue'),
      meta: { title: '文档 · Natsel' },
    },
    {
      path: '/:pathMatch(.*)*',
      name: 'not-found',
      component: () => import('@/views/NotFoundView.vue'),
      meta: { title: '页面未找到 · Natsel' },
    },
  ],
  scrollBehavior(to, _from, savedPosition) {
    if (savedPosition) return savedPosition
    if (to.hash) return { el: to.hash, behavior: 'smooth', top: 90 }
    return { top: 0 }
  },
})

router.afterEach((to) => {
  document.title = (to.meta.title as string) ?? 'Natsel'
})

export default router
