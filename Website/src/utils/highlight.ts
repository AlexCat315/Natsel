const KEYWORDS =
  /\b(fn|let|mut|struct|enum|trait|impl|pub|crate|mod|use|return|if|else|for|while|match|loop|in|self|Self|true|false|const|static|unsafe|where|as|ref|move|async|await|dyn|type|break|continue|extern|super|Option|Result|Some|None|Ok|Err)\b/g

const SHELL_KEYWORDS =
  /\b(curl|sh|sudo|brew|apt|git|cd|ls|export|cargo|rustc|echo|source|chmod|mkdir)\b/g

const STRING_RE = /(&quot;.*?&quot;|&#39;.*?&#39;)/g

export function escapeHtml(s: string): string {
  return s
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;')
}

export function highlight(src: string, lang: string): string {
  const isShell = /^(sh|bash|shell|powershell|zsh)$/i.test(lang)
  const lines = src.replace(/\n$/, '').split('\n')

  return lines
    .map((line) => {
      let out = escapeHtml(line)
      let comment = ''

      const commentMatch = out.match(isShell ? /#/ : /(\/\/|\/\*)/)
      if (commentMatch && commentMatch.index !== undefined) {
        comment = `<span class="tok-comment">${out.slice(commentMatch.index)}</span>`
        out = out.slice(0, commentMatch.index)
      }

      let mark = 0xe000
      const tokens = new Map<string, string>()
      out = out.replace(STRING_RE, (m) => {
        const token = String.fromCodePoint(mark++)
        tokens.set(token, m)
        return token
      })

      out = out.replace(
        isShell ? SHELL_KEYWORDS : KEYWORDS,
        '<span class="tok-keyword">$&</span>',
      )
      if (!isShell) {
        out = out.replace(/\b(\d+\.?\d*)\b/g, '<span class="tok-number">$1</span>')
      }

      for (const [token, s] of tokens) {
        out = out.split(token).join(`<span class="tok-string">${s}</span>`)
      }

      return out + comment
    })
    .join('\n')
}
