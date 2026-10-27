<script setup lang="ts">
/**
 * PhysicsCanvas — interactive viewport over the mini 2D engine.
 * Scenes: pyramid / balls / chain / rain. Click or drag to launch
 * bodies; an HUD shows body count, contacts and frame time.
 */
import { onBeforeUnmount, onMounted, ref } from 'vue'
import { World, type Body } from '@/physics/engine'

type Scene = 'pyramid' | 'balls' | 'chain' | 'rain'

const canvas = ref<HTMLCanvasElement | null>(null)
const scene = ref<Scene>('pyramid')
const bodyCount = ref(0)
const contactCount = ref(0)
const frameMs = ref(0)

let ctx: CanvasRenderingContext2D | null = null
let world: World
let raf = 0
let cssW = 0
let cssH = 0
let dpr = 1
let last = performance.now()
let rainTimer = 0

const drag = { active: false, x: 0, y: 0, sx: 0, sy: 0 }

function resize() {
  if (!canvas.value) return
  const parent = canvas.value.parentElement
  cssW = parent?.clientWidth ?? window.innerWidth
  cssH = parent?.clientHeight ?? 500
  dpr = Math.min(window.devicePixelRatio || 1, 2)
  canvas.value.width = cssW * dpr
  canvas.value.height = cssH * dpr
  canvas.value.style.width = `${cssW}px`
  canvas.value.style.height = `${cssH}px`
  ctx?.setTransform(dpr, 0, 0, dpr, 0, 0)
  rebuild()
}

function rebuild() {
  const groundY = cssH - 8
  world = new World(cssW, cssH, groundY)
  // side walls
  loadScene(scene.value)
}

function loadScene(s: Scene) {
  world.clearDynamic()
  const groundY = world.groundY
  // ground slab (visual static body)
  world.addBox(cssW / 2, groundY + 20, cssW, 20, { isStatic: true })

  if (s === 'pyramid') {
    const rows = 9
    const size = 22
    const spacing = size * 2 + 2
    for (let r = 0; r < rows; r++) {
      for (let i = 0; i <= r; i++) {
        const x = cssW / 2 + (i - r / 2) * spacing
        const y = groundY - 40 - (rows - 1 - r) * spacing
        world.addBox(x, y, size, size, { hue: 80 + r * 6 })
      }
    }
  } else if (s === 'balls') {
    for (let i = 0; i < 40; i++) {
      const radius = 10 + Math.random() * 16
      world.addCircle(
        20 + Math.random() * (cssW - 40),
        40 + Math.random() * cssH * 0.5,
        radius,
        { hue: 120 + Math.random() * 60 },
      )
    }
  } else if (s === 'chain') {
    const n = 14
    let prev: Body | null = null
    const anchor = world.addCircle(cssW / 2, 30, 6, { isStatic: true })
    prev = anchor
    for (let i = 1; i <= n; i++) {
      const link = world.addCircle(cssW / 2 + i * 2, 30 + i * 22, 9, { hue: 50 })
      world.addJoint(prev, link, 22, 0.95)
      prev = link
    }
    // a weight at the end
    const weight = world.addBox(cssW / 2 + 30, 30 + n * 22 + 16, 20, 20, { hue: 45 })
    world.addJoint(prev, weight, 26, 0.95)
  } else if (s === 'rain') {
    spawnRain()
  }
}

function spawnRain() {
  for (let i = 0; i < 3; i++) {
    if (Math.random() > 0.5) {
      world.addCircle(Math.random() * cssW, -20, 8 + Math.random() * 14, {
        hue: 120 + Math.random() * 60,
      })
    } else {
      const s = 12 + Math.random() * 10
      world.addBox(Math.random() * cssW, -20, s, s, { hue: 80 + Math.random() * 40 })
    }
  }
  // cap body count
  if (world.bodies.length > 160) {
    world.bodies.splice(0, world.bodies.length - 160)
  }
}

function chooseScene(s: Scene) {
  scene.value = s
  loadScene(s)
}

function launchAt(x: number, y: number, vx: number, vy: number) {
  if (Math.random() > 0.45) {
    const s = 14
    world.addBox(x, y, s, s, { vx, vy, hue: 80 + Math.random() * 40 })
  } else {
    world.addCircle(x, y, 14, { vx, vy, hue: 120 + Math.random() * 60 })
  }
}

function pointerPos(e: PointerEvent) {
  const rect = canvas.value!.getBoundingClientRect()
  return { x: e.clientX - rect.left, y: e.clientY - rect.top }
}

function onDown(e: PointerEvent) {
  const p = pointerPos(e)
  canvas.value?.setPointerCapture(e.pointerId)
  drag.active = true
  drag.sx = p.x
  drag.sy = p.y
  drag.x = p.x
  drag.y = p.y
}
function onMove(e: PointerEvent) {
  if (!drag.active) return
  const p = pointerPos(e)
  drag.x = p.x
  drag.y = p.y
}
function onUp(e: PointerEvent) {
  if (!drag.active) return
  const p = pointerPos(e)
  const vx = (drag.sx - p.x) * 6
  const vy = (drag.sy - p.y) * 6
  launchAt(p.x, p.y, vx, vy)
  drag.active = false
}

function drawBody(b: Body) {
  if (!ctx) return
  const light = 58 + b.flash * 30
  const color = `hsl(${b.hue}, 75%, ${light}%)`
  const stroke = b.flash > 0.4 ? `hsl(${b.hue}, 95%, 80%)` : `hsl(${b.hue}, 70%, 40%)`

  ctx.save()
  ctx.translate(b.x, b.y)
  ctx.rotate(b.angle)
  if (b.kind === 'circle') {
    ctx.beginPath()
    ctx.arc(0, 0, b.radius, 0, Math.PI * 2)
    ctx.fillStyle = b.isStatic ? 'rgba(74,222,128,0.15)' : color
    ctx.fill()
    ctx.lineWidth = 2
    ctx.strokeStyle = b.isStatic ? 'rgba(74,222,128,0.5)' : stroke
    ctx.stroke()
    if (!b.isStatic) {
      ctx.beginPath()
      ctx.moveTo(0, 0)
      ctx.lineTo(b.radius, 0)
      ctx.strokeStyle = 'rgba(0,0,0,0.25)'
      ctx.lineWidth = 1.5
      ctx.stroke()
    }
  } else {
    ctx.fillStyle = b.isStatic ? 'rgba(74,222,128,0.08)' : color
    ctx.strokeStyle = b.isStatic ? 'rgba(74,222,128,0.4)' : stroke
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.rect(-b.halfW, -b.halfH, b.halfW * 2, b.halfH * 2)
    ctx.fill()
    ctx.stroke()
  }
  ctx.restore()
}

function frame(now: number) {
  if (!ctx) return
  const dt = Math.min((now - last) / 1000, 1 / 30)
  last = now
  frameMs.value = now - (now - dt * 1000)

  if (scene.value === 'rain') {
    rainTimer += dt
    if (rainTimer > 0.25) {
      rainTimer = 0
      spawnRain()
    }
  }

  // fixed sub-steps for stability
  world.step(dt)

  ctx.clearRect(0, 0, cssW, cssH)

  // joints
  ctx.strokeStyle = 'rgba(251,191,36,0.5)'
  ctx.lineWidth = 1.5
  for (const j of world.joints) {
    ctx.beginPath()
    ctx.moveTo(j.a.x, j.a.y)
    ctx.lineTo(j.b.x, j.b.y)
    ctx.stroke()
  }

  for (const b of world.bodies) drawBody(b)

  // drag aim line
  if (drag.active) {
    ctx.strokeStyle = 'rgba(134,239,172,0.7)'
    ctx.setLineDash([6, 6])
    ctx.beginPath()
    ctx.moveTo(drag.sx, drag.sy)
    ctx.lineTo(drag.x, drag.y)
    ctx.stroke()
    ctx.setLineDash([])
  }

  bodyCount.value = world.bodies.filter((b) => !b.isStatic).length
  contactCount.value = world.contactCount
  frameMs.value = dt * 1000

  raf = requestAnimationFrame(frame)
}

const scenes: { id: Scene; label: string }[] = [
  { id: 'pyramid', label: '金字塔' },
  { id: 'balls', label: '球堆' },
  { id: 'chain', label: '链条' },
  { id: 'rain', label: '粒子雨' },
]

onMounted(() => {
  ctx = canvas.value!.getContext('2d')
  resize()
  window.addEventListener('resize', resize)
  raf = requestAnimationFrame(frame)
})
onBeforeUnmount(() => {
  cancelAnimationFrame(raf)
  window.removeEventListener('resize', resize)
})
</script>

<template>
  <div class="physics-viewport hud">
    <canvas
      ref="canvas"
      class="physics-canvas"
      @pointerdown="onDown"
      @pointermove="onMove"
      @pointerup="onUp"
      @pointercancel="onUp"
    ></canvas>

    <div class="scene-bar">
      <button
        v-for="s in scenes"
        :key="s.id"
        class="scene-btn"
        :class="{ active: scene === s.id }"
        @click="chooseScene(s.id)"
      >
        {{ s.label }}
      </button>
    </div>

    <div class="hud-panel">
      <div><span class="hud-label">BODIES</span><span class="hud-value">{{ bodyCount }}</span></div>
      <div><span class="hud-label">CONTACTS</span><span class="hud-value">{{ contactCount }}</span></div>
      <div><span class="hud-label">STEP</span><span class="hud-value">{{ frameMs.toFixed(1) }}ms</span></div>
    </div>

    <div class="hint">点击或拖拽发射物体</div>
  </div>
</template>

<style scoped>
.physics-viewport {
  position: relative;
  width: 100%;
  height: 560px;
  border-radius: var(--radius-lg);
  overflow: hidden;
  border: 1px solid var(--border);
  background:
    radial-gradient(120% 100% at 50% 0%, rgba(74, 222, 128, 0.06), transparent 60%),
    #050a07;
}

.physics-canvas {
  position: absolute;
  inset: 0;
  cursor: crosshair;
  touch-action: none;
}

.scene-bar {
  position: absolute;
  top: 16px;
  left: 16px;
  display: flex;
  gap: 6px;
  flex-wrap: wrap;
}
.scene-btn {
  padding: 7px 15px;
  font-size: 12.5px;
  font-family: var(--font-mono);
  border-radius: 8px;
  border: 1px solid var(--border);
  background: rgba(6, 14, 9, 0.72);
  color: var(--text-dim);
  cursor: pointer;
  backdrop-filter: blur(8px);
}
.scene-btn:hover { color: var(--text); border-color: var(--border-strong); }
.scene-btn.active {
  color: #08130b;
  background: var(--gradient);
  border-color: transparent;
  font-weight: 700;
}

.hud-panel {
  position: absolute;
  top: 16px;
  right: 16px;
  display: flex;
  flex-direction: column;
  gap: 6px;
  padding: 12px 14px;
  border-radius: 10px;
  border: 1px solid var(--border);
  background: rgba(6, 14, 9, 0.72);
  backdrop-filter: blur(8px);
  font-family: var(--font-mono);
  min-width: 116px;
}
.hud-panel div {
  display: flex;
  justify-content: space-between;
  gap: 14px;
}
.hud-label { font-size: 10.5px; color: var(--text-faint); letter-spacing: 0.08em; }
.hud-value { font-size: 13px; color: var(--primary-soft); font-weight: 600; }

.hint {
  position: absolute;
  bottom: 14px;
  left: 16px;
  font-size: 12px;
  font-family: var(--font-mono);
  color: var(--text-faint);
}

@media (max-width: 600px) {
  .physics-viewport { height: 460px; }
  .hud-panel { min-width: 100px; }
}
</style>
