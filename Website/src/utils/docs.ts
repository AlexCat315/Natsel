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
import concepts from '@/content/concepts.md?raw'
import determinism from '@/content/determinism.md?raw'

export const docGroups: DocGroup[] = [
  {
    group: '指南',
    items: [{ slug: 'getting-started', title: '快速开始', raw: gettingStarted }],
  },
  {
    group: '核心概念',
    items: [
      { slug: 'concepts', title: '核心概念', raw: concepts },
      { slug: 'determinism', title: '确定性', raw: determinism },
    ],
  },
]

export const allDocs: DocLeaf[] = docGroups.flatMap((g) => g.items)

export function findDoc(slug?: string): DocLeaf {
  return allDocs.find((d) => d.slug === slug) ?? allDocs[0]
}
