/* ============================================================
   Mini 2D rigid-body physics (impulse-based, SAT for boxes).
   A small, self-contained engine used to demonstrate the kind
   of simulation Natsel provides in 3D. Not a port — a demo.
   ============================================================ */

export type BodyKind = 'circle' | 'box'

export interface Body {
  kind: BodyKind
  x: number
  y: number
  vx: number
  vy: number
  angle: number
  angularVelocity: number
  // circle
  radius: number
  // box half extents
  halfW: number
  halfH: number
  invMass: number
  invInertia: number
  restitution: number
  friction: number
  isStatic: boolean
  hue: number
  // accumulator for flash on spawn/impact
  flash: number
}

export interface DistanceJoint {
  a: Body
  b: Body
  length: number
  stiffness: number
}

interface Contact {
  a: Body
  b: Body
  nx: number
  ny: number
  depth: number
  px: number
  py: number
  // accumulated impulses (Box2D-style warm-started accumulation)
  jn: number
  jt: number
}

const VELOCITY_ITERATIONS: number = 30
const POSITION_ITERATIONS: number = 6
const PENETRATION_SLOP: number = 0.5
const PENETRATION_PERCENT: number = 0.4

/** Clip a segment to the half-space (nn·p >= offset). Box2D-style. */
function clipSegment(
  v1: [number, number],
  v2: [number, number],
  nnx: number,
  nny: number,
  offset: number,
): [number, number][] {
  const d1 = nnx * v1[0] + nny * v1[1] - offset
  const d2 = nnx * v2[0] + nny * v2[1] - offset
  const pts: [number, number][] = []
  if (d1 >= 0) pts.push([v1[0], v1[1]])
  if (d2 >= 0) pts.push([v2[0], v2[1]])
  if (d1 * d2 < 0) {
    const t = d1 / (d1 - d2)
    pts.push([v1[0] + t * (v2[0] - v1[0]), v1[1] + t * (v2[1] - v1[1])])
  }
  return pts
}

export class World {
  bodies: Body[] = []
  joints: DistanceJoint[] = []
  gravity: number
  width: number
  height: number
  groundY: number
  contactCount = 0

  constructor(width: number, height: number, groundY: number, gravity = 1400) {
    this.width = width
    this.height = height
    this.groundY = groundY
    this.gravity = gravity
  }

  addCircle(x: number, y: number, radius: number, opts: Partial<Body> = {}): Body {
    const density = opts.isStatic ? 0 : 1
    const mass = Math.PI * radius * radius * density
    const inertia = (mass * radius * radius) / 2
    const b: Body = {
      kind: 'circle',
      x,
      y,
      vx: opts.vx ?? 0,
      vy: opts.vy ?? 0,
      angle: 0,
      angularVelocity: 0,
      radius,
      halfW: 0,
      halfH: 0,
      invMass: opts.isStatic ? 0 : 1 / mass,
      invInertia: opts.isStatic ? 0 : 1 / inertia,
      restitution: opts.restitution ?? 0.18,
      friction: opts.friction ?? 0.5,
      isStatic: opts.isStatic ?? false,
      hue: opts.hue ?? 140,
      flash: 1,
    }
    this.bodies.push(b)
    return b
  }

  addBox(x: number, y: number, halfW: number, halfH: number, opts: Partial<Body> = {}): Body {
    const density = opts.isStatic ? 0 : 1
    const mass = halfW * 2 * halfH * 2 * density
    const inertia = (mass * (halfW * halfW * 4 + halfH * halfH * 4)) / 12
    const b: Body = {
      kind: 'box',
      x,
      y,
      vx: opts.vx ?? 0,
      vy: opts.vy ?? 0,
      angle: opts.angle ?? 0,
      angularVelocity: 0,
      radius: 0,
      halfW,
      halfH,
      invMass: opts.isStatic ? 0 : 1 / mass,
      invInertia: opts.isStatic ? 0 : 1 / inertia,
      restitution: opts.restitution ?? 0.12,
      friction: opts.friction ?? 0.55,
      isStatic: opts.isStatic ?? false,
      hue: opts.hue ?? 90,
      flash: 1,
    }
    this.bodies.push(b)
    return b
  }

  addJoint(a: Body, b: Body, length?: number, stiffness = 0.9): DistanceJoint {
    const len = length ?? Math.hypot(a.x - b.x, a.y - b.y)
    const j: DistanceJoint = { a, b, length: len, stiffness }
    this.joints.push(j)
    return j
  }

  clearDynamic() {
    this.bodies = this.bodies.filter((b) => b.isStatic)
    this.joints = []
  }

  /* ---------------------------------------------------------- */
  step(dt: number) {
    // Integrate forces
    for (const b of this.bodies) {
      if (b.invMass === 0) continue
      b.vy += this.gravity * dt
      // mild angular damping
      b.angularVelocity *= 0.98
      b.vx *= 0.999
      b.flash = Math.max(0, b.flash - dt * 2.5)
    }

    const contacts: Contact[] = []
    this.findContacts(contacts)
    this.contactCount = contacts.length

    // Solve velocity constraints
    for (let i = 0; i < VELOCITY_ITERATIONS; i++) {
      for (const c of contacts) this.solveContact(c)
      for (const j of this.joints) this.solveJoint(j)
    }

    // Integrate positions
    for (const b of this.bodies) {
      if (b.invMass === 0) continue
      b.x += b.vx * dt
      b.y += b.vy * dt
      b.angle += b.angularVelocity * dt
    }

    // Positional correction — re-detect contacts each iteration so the
    // manifold reflects current geometry (prevents neighbouring bodies
    // being pushed into each other and oscillating).
    for (let i = 0; i < POSITION_ITERATIONS; i++) {
      const pc: Contact[] = []
      this.findContacts(pc)
      for (const c of pc) this.correctPosition(c)
    }

    // Static bounds (walls + ground)
    for (const b of this.bodies) {
      if (b.isStatic) continue
      const r = b.kind === 'circle' ? b.radius : Math.max(b.halfW, b.halfH)
      if (b.x - r < 0) {
        b.x = r
        b.vx = Math.abs(b.vx) * 0.6
      }
      if (b.x + r > this.width) {
        b.x = this.width - r
        b.vx = -Math.abs(b.vx) * 0.6
      }
      if (b.y + r > this.groundY) {
        b.y = this.groundY - r
        b.vy = -Math.abs(b.vy) * b.restitution
        b.vx *= 0.92
        b.angularVelocity *= 0.9
      }
    }
  }

  /* ---------------------------------------------------------- */
  findContacts(out: Contact[]) {
    for (let i = 0; i < this.bodies.length; i++) {
      for (let j = i + 1; j < this.bodies.length; j++) {
        const a = this.bodies[i]
        const b = this.bodies[j]
        if (a.invMass + b.invMass === 0) continue
        if (a.kind === 'circle' && b.kind === 'circle') this.circleCircle(a, b, out)
        else if (a.kind === 'circle' && b.kind === 'box') this.circleBox(a, b, out)
        else if (a.kind === 'box' && b.kind === 'circle') this.circleBox(b, a, out)
        else this.boxBox(a, b, out)
      }
    }
  }

  circleCircle(a: Body, b: Body, out: Contact[]) {
    const dx = b.x - a.x
    const dy = b.y - a.y
    const dist = Math.hypot(dx, dy)
    const r = a.radius + b.radius
    if (dist >= r) return
    const nx = dist === 0 ? 1 : dx / dist
    const ny = dist === 0 ? 0 : dy / dist
    out.push({
      a,
      b,
      nx,
      ny,
      depth: r - dist,
      px: (a.x + b.x) / 2,
      py: (a.y + b.y) / 2,
      jn: 0,
      jt: 0,
    })
  }

  circleBox(circle: Body, box: Body, out: Contact[]) {
    // Circle center in box local space
    const cos = Math.cos(-box.angle)
    const sin = Math.sin(-box.angle)
    const dx = circle.x - box.x
    const dy = circle.y - box.y
    const lx = dx * cos - dy * sin
    const ly = dx * sin + dy * cos

    const cx = Math.max(-box.halfW, Math.min(box.halfW, lx))
    const cy = Math.max(-box.halfH, Math.min(box.halfH, ly))

    const ddx = lx - cx
    const ddy = ly - cy
    const d = Math.hypot(ddx, ddy)

    if (d > circle.radius && !(d === 0 && circle.radius > 0)) {
      // center inside box gives d==0, handled below
      if (d !== 0) return
    }

    // closest point back to world
    const cosW = Math.cos(box.angle)
    const sinW = Math.sin(box.angle)
    const wx = box.x + cx * cosW - cy * sinW
    const wy = box.y + cx * sinW + cy * cosW

    let nx: number
    let ny: number
    let depth: number
    if (d === 0) {
      // circle center inside box: push out along nearest face
      const left = Math.abs(lx + box.halfW)
      const right = Math.abs(box.halfW - lx)
      const top = Math.abs(ly + box.halfH)
      const bottom = Math.abs(box.halfH - ly)
      const min = Math.min(left, right, top, bottom)
      let lnx = 0
      let lny = 0
      if (min === left) lnx = -1
      else if (min === right) lnx = 1
      else if (min === top) lny = -1
      else lny = 1
      nx = lnx * cosW - lny * sinW
      ny = lnx * sinW + lny * cosW
      depth = circle.radius
    } else {
      const lnx = ddx / d
      const lny = ddy / d
      nx = lnx * cosW - lny * sinW
      ny = lnx * sinW + lny * cosW
      depth = circle.radius - d
    }
    out.push({ a: circle, b: box, nx, ny, depth, px: wx, py: wy, jn: 0, jt: 0 })
  }

  boxBox(a: Body, b: Body, out: Contact[]) {
    const axesA = this.boxAxes(a)
    const axesB = this.boxAxes(b)

    let minOverlap = Infinity
    let nx = 0
    let ny = 0

    for (const [ax, ay] of [...axesA, ...axesB]) {
      const [minA, maxA] = this.project(a, ax, ay)
      const [minB, maxB] = this.project(b, ax, ay)
      if (maxA <= minB || maxB <= minA) return // separating axis
      const overlap = Math.min(maxA - minB, maxB - minA)
      if (overlap < minOverlap) {
        minOverlap = overlap
        nx = ax
        ny = ay
        // direction from a -> b
        const cx = b.x - a.x
        const cy = b.y - a.y
        if (cx * ax + cy * ay < 0) {
          nx = -ax
          ny = -ay
        }
      }
    }

    // Build a 2-point manifold by clipping the incident edge (b) against
    // the reference face (a). Two contacts are required for stacked boxes
    // to rest without tipping.
    const facesA = this.boxFaces(a)
    const facesB = this.boxFaces(b)

    // reference face: a-face whose normal best aligns with n
    let refFace = facesA[0]
    let refDot = -Infinity
    for (const f of facesA) {
      const d = f.n[0] * nx + f.n[1] * ny
      if (d > refDot) {
        refDot = d
        refFace = f
      }
    }
    // incident face: b-face whose normal best aligns with -n
    let incFace = facesB[0]
    let incDot = -Infinity
    for (const f of facesB) {
      const d = f.n[0] * -nx + f.n[1] * -ny
      if (d > incDot) {
        incDot = d
        incFace = f
      }
    }

    const e0 = refFace.v[0]
    const e1 = refFace.v[1]
    // tangent along the reference face
    let tgx = e1[0] - e0[0]
    let tgy = e1[1] - e0[1]
    const tlen = Math.hypot(tgx, tgy) || 1
    tgx /= tlen
    tgy /= tlen

    const i1 = incFace.v[0]
    const i2 = incFace.v[1]

    // clip incident edge to the two ends of the reference face
    let cp = clipSegment(i1, i2, tgx, tgy, tgx * e0[0] + tgy * e0[1])
    if (cp.length < 2) {
      out.push({
        a,
        b,
        nx,
        ny,
        depth: minOverlap,
        px: (i1[0] + i2[0]) / 2,
        py: (i1[1] + i2[1]) / 2,
        jn: 0,
        jt: 0,
      })
      return
    }
    cp = clipSegment(cp[0], cp[1], -tgx, -tgy, -tgx * e1[0] - tgy * e1[1])

    const refPlane = nx * e0[0] + ny * e0[1]
    for (const p of cp) {
      const d = nx * p[0] + ny * p[1] - refPlane
      if (d <= 0) {
        out.push({ a, b, nx, ny, depth: -d, px: p[0], py: p[1], jn: 0, jt: 0 })
      }
    }
  }

  boxFaces(b: Body): { v: [number, number][]; n: [number, number] }[] {
    const c = Math.cos(b.angle)
    const s = Math.sin(b.angle)
    const verts = this.boxVerts(b)
    const normals: [number, number][] = [
      [s, -c],
      [c, s],
      [-s, c],
      [-c, -s],
    ]
    return [
      { v: [verts[0], verts[1]], n: normals[0] },
      { v: [verts[1], verts[2]], n: normals[1] },
      { v: [verts[2], verts[3]], n: normals[2] },
      { v: [verts[3], verts[0]], n: normals[3] },
    ]
  }

  boxAxes(b: Body): [number, number][] {
    const c = Math.cos(b.angle)
    const s = Math.sin(b.angle)
    return [
      [c, s],
      [-s, c],
    ]
  }

  project(b: Body, ax: number, ay: number): [number, number] {
    const verts = this.boxVerts(b)
    let min = Infinity
    let max = -Infinity
    for (const [vx, vy] of verts) {
      const p = vx * ax + vy * ay
      if (p < min) min = p
      if (p > max) max = p
    }
    return [min, max]
  }

  boxVerts(b: Body): [number, number][] {
    const c = Math.cos(b.angle)
    const s = Math.sin(b.angle)
    const corners: [number, number][] = [
      [-b.halfW, -b.halfH],
      [b.halfW, -b.halfH],
      [b.halfW, b.halfH],
      [-b.halfW, b.halfH],
    ]
    return corners.map(([lx, ly]) => [
      b.x + lx * c - ly * s,
      b.y + lx * s + ly * c,
    ])
  }

  /* ---------------------------------------------------------- */
  /** Apply a normal + tangent impulse pair at a contact point. */
  applyContactImpulse(c: Contact, jn: number, jt: number) {
    const { a, b, nx, ny, px, py } = c
    const rax = px - a.x
    const ray = py - a.y
    const rbx = px - b.x
    const rby = py - b.y
    // fixed tangent: 90° CCW from normal
    const tx = -ny
    const ty = nx
    const jx = jn * nx + jt * tx
    const jy = jn * ny + jt * ty
    a.vx -= jx * a.invMass
    a.vy -= jy * a.invMass
    a.angularVelocity -= (rax * jy - ray * jx) * a.invInertia
    b.vx += jx * b.invMass
    b.vy += jy * b.invMass
    b.angularVelocity += (rbx * jy - rby * jx) * b.invInertia
  }

  solveContact(c: Contact) {
    const { a, b, nx, ny, px, py } = c
    const rax = px - a.x
    const ray = py - a.y
    const rbx = px - b.x
    const rby = py - b.y
    // fixed tangent (90° CCW from normal) so warm-start direction is stable
    const tx = -ny
    const ty = nx

    const relVel = (): [number, number] => [
      (b.vx + -b.angularVelocity * rby) - (a.vx + -a.angularVelocity * ray),
      (b.vy + b.angularVelocity * rbx) - (a.vy + a.angularVelocity * rax),
    ]
    let [rvx, rvy] = relVel()

    const k = a.invMass + b.invMass
    const raCrossN = rax * ny - ray * nx
    const rbCrossN = rbx * ny - rby * nx
    const raCrossT = rax * ty - ray * tx
    const rbCrossT = rbx * ty - rby * tx
    const normalMass =
      k + raCrossN * raCrossN * a.invInertia + rbCrossN * rbCrossN * b.invInertia
    const tangentMass =
      k + raCrossT * raCrossT * a.invInertia + rbCrossT * rbCrossT * b.invInertia
    if (normalMass === 0 || tangentMass === 0) return

    // ---- normal impulse: accumulated, clamped to stay non-negative ----
    let vn = rvx * nx + rvy * ny
    const e = vn < -60 ? Math.min(a.restitution, b.restitution) : 0
    let dln = (-(1 + e) * vn) / normalMass
    const oldJn = c.jn
    c.jn = Math.max(0, oldJn + dln)
    this.applyContactImpulse(c, c.jn - oldJn, 0)

    // ---- friction impulse: accumulated, clamped to ±mu·jn ----
    ;[rvx, rvy] = relVel()
    const vt = rvx * tx + rvy * ty
    let djt = -vt / tangentMass
    const mu = Math.sqrt(a.friction * b.friction)
    const oldJt = c.jt
    const maxF = c.jn * mu
    const newJt = Math.max(-maxF, Math.min(maxF, oldJt + djt))
    c.jt = newJt
    this.applyContactImpulse(c, 0, newJt - oldJt)

    // impact flash on hard hits
    if (c.jn > 400) {
      const f = Math.min(1, c.jn / 2000)
      if (!a.isStatic) a.flash = Math.max(a.flash, f)
      if (!b.isStatic) b.flash = Math.max(b.flash, f)
    }
  }

  correctPosition(c: Contact) {
    const { a, b, nx, ny, depth } = c
    const correction = Math.max(depth - PENETRATION_SLOP, 0) /
      (a.invMass + b.invMass) * PENETRATION_PERCENT
    const ix = correction * nx
    const iy = correction * ny
    a.x -= ix * a.invMass
    a.y -= iy * a.invMass
    b.x += ix * b.invMass
    b.y += iy * b.invMass
  }

  solveJoint(j: DistanceJoint) {
    const { a, b, length, stiffness } = j
    const dx = b.x - a.x
    const dy = b.y - a.y
    const dist = Math.hypot(dx, dy) || 1e-6
    const nx = dx / dist
    const ny = dy / dist
    const error = (dist - length) * stiffness

    // positional spring
    const invMassSum = a.invMass + b.invMass
    if (invMassSum > 0) {
      const corr = error / invMassSum
      a.x += nx * corr * a.invMass
      a.y += ny * corr * a.invMass
      b.x -= nx * corr * b.invMass
      b.y -= ny * corr * b.invMass
    }

    // velocity constraint along the joint axis
    const relVn = (b.vx - a.vx) * nx + (b.vy - a.vy) * ny
    const k = 1 / (a.invMass + b.invMass || 1)
    const impulse = -relVn * k * stiffness
    a.vx -= impulse * nx * a.invMass
    a.vy -= impulse * ny * a.invMass
    b.vx += impulse * nx * b.invMass
    b.vy += impulse * ny * b.invMass
  }
}
