import { marked } from 'marked'
import type { Renderer } from 'marked'
import { highlight, escapeHtml } from './highlight'

/** Render markdown to HTML with highlighted, copyable code blocks. */
export function renderMarkdown(src: string): string {
  const renderer = new marked.Renderer()

  renderer.code = ({ text, lang }: { text: string; lang?: string }) => {
    const language = lang ?? 'text'
    const codeAttr = escapeHtml(text)
    const highlighted = /^(text|plain)$/i.test(language)
      ? escapeHtml(text)
      : highlight(text, language)
    return `<div class="md-code" data-code="${codeAttr}">
      <div class="md-code-head">
        <span>${escapeHtml(language)}</span>
        <button class="md-copy" type="button">复制</button>
      </div>
      <pre><code>${highlighted}</code></pre>
    </div>`
  }

  // Render the heading's inline content properly: parseInline keeps <code>, <strong>,
  // <a> and friends instead of flattening the heading to bare text.
  renderer.heading = function (this: Renderer, { tokens, depth }) {
    const inner = this.parser.parseInline(tokens)
    const id = slugify(stripTags(inner))
    return `<h${depth} id="${id}">${inner}</h${depth}>`
  }

  marked.setOptions({ renderer, gfm: true, breaks: false })
  return marked.parse(src) as string
}

/** Strip HTML tags, leaving their text content. */
export function stripTags(html: string): string {
  return html
    .replace(/<[^>]*>/g, '')
    .replace(/&amp;/g, '&')
    .replace(/&lt;/g, '<')
    .replace(/&gt;/g, '>')
    .replace(/&quot;/g, '"')
    .replace(/&#39;/g, "'")
}

export function slugify(text: string): string {
  return text
    .toLowerCase()
    .replace(/[^\w\u4e00-\u9fa5\s-]/g, '')
    .trim()
    .replace(/\s+/g, '-')
}

/**
 * Build the on-page table of contents from rendered HTML.
 *
 * Heading markup can contain inline elements (`<code>`, `<strong>`, `<a>`), so the
 * text is captured as a non-greedy run of anything and then stripped of tags.
 */
export function extractToc(html: string): { id: string; text: string; depth: number }[] {
  const re = /<h([23])\b[^>]*\bid="([^"]+)"[^>]*>([\s\S]*?)<\/h\1>/g
  const toc: { id: string; text: string; depth: number }[] = []
  let m: RegExpExecArray | null
  while ((m = re.exec(html))) {
    const text = stripTags(m[3]).replace(/\s+/g, ' ').trim()
    if (!text) continue
    toc.push({ depth: Number(m[1]), id: m[2], text })
  }
  return toc
}
