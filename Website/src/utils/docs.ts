/** Documentation manifest — the sidebar tree. */
export interface DocLeaf {
  slug: string
  title: string
  raw: string
}
export interface DocGroup {
  group: string
  items: DocLeaf[]
}

import gettingStarted from '@/content/getting-started.md?raw'
import installation from '@/content/installation.md?raw'

import concepts from '@/content/concepts.md?raw'
import architecture from '@/content/architecture.md?raw'
import units from '@/content/units.md?raw'
import determinism from '@/content/determinism.md?raw'

import bodies from '@/content/bodies.md?raw'
import shapes from '@/content/shapes.md?raw'
import geometry from '@/content/geometry.md?raw'
import materials from '@/content/materials.md?raw'

import queries from '@/content/queries.md?raw'
import events from '@/content/events.md?raw'
import joints from '@/content/joints.md?raw'
import ccd from '@/content/ccd.md?raw'
import buoyancy from '@/content/buoyancy.md?raw'
import multithreading from '@/content/multithreading.md?raw'

import character from '@/content/character.md?raw'
import vehicle from '@/content/vehicle.md?raw'
import skeleton from '@/content/skeleton.md?raw'
import softBody from '@/content/soft-body.md?raw'

import debugDraw from '@/content/debug-draw.md?raw'
import snapshots from '@/content/snapshots.md?raw'
import apiReference from '@/content/api-reference.md?raw'
import faq from '@/content/faq.md?raw'

export const docGroups: DocGroup[] = [
  {
    group: '指南',
    items: [
      { slug: 'getting-started', title: '快速开始', raw: gettingStarted },
      { slug: 'installation', title: '安装与集成', raw: installation },
    ],
  },
  {
    group: '核心概念',
    items: [
      { slug: 'concepts', title: '核心概念', raw: concepts },
      { slug: 'architecture', title: '架构总览', raw: architecture },
      { slug: 'units', title: '坐标与单位', raw: units },
      { slug: 'determinism', title: '确定性', raw: determinism },
    ],
  },
  {
    group: '实体与几何',
    items: [
      { slug: 'bodies', title: '刚体', raw: bodies },
      { slug: 'shapes', title: '形状', raw: shapes },
      { slug: 'geometry', title: '几何构建器', raw: geometry },
      { slug: 'materials', title: '材质与过滤', raw: materials },
    ],
  },
  {
    group: '模拟系统',
    items: [
      { slug: 'queries', title: '空间查询', raw: queries },
      { slug: 'events', title: '事件与传感器', raw: events },
      { slug: 'joints', title: '关节', raw: joints },
      { slug: 'ccd', title: '连续碰撞检测', raw: ccd },
      { slug: 'buoyancy', title: '浮力与爆炸', raw: buoyancy },
      { slug: 'multithreading', title: '多线程与性能', raw: multithreading },
    ],
  },
  {
    group: '角色与生物',
    items: [
      { slug: 'character', title: '虚拟角色', raw: character },
      { slug: 'vehicle', title: '车辆', raw: vehicle },
      { slug: 'skeleton', title: '骨骼与布娃娃', raw: skeleton },
      { slug: 'soft-body', title: '软体', raw: softBody },
    ],
  },
  {
    group: '工程实践',
    items: [
      { slug: 'debug-draw', title: '调试绘制', raw: debugDraw },
      { slug: 'snapshots', title: '快照与状态哈希', raw: snapshots },
      { slug: 'api-reference', title: 'API 速查', raw: apiReference },
      { slug: 'faq', title: '常见问题', raw: faq },
    ],
  },
]

export const allDocs: DocLeaf[] = docGroups.flatMap((g) => g.items)

export function findDoc(slug?: string): DocLeaf {
  return allDocs.find((d) => d.slug === slug) ?? allDocs[0]
}
