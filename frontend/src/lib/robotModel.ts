// Single source of truth for the robot's geometry (everything except the SO-101 arm, which comes
// from the URDF). The website renders these parts, and scripts/export-robot.ts exports the same
// list (plus meshes for the decks) so the MuJoCo simulation uses the identical model.
//
// Frame: x forward, y left, z up, metres. No local imports so Node can run it directly.
import * as THREE from 'three'

export type Vec3 = [number, number, number]
export type Quat = [number, number, number, number] // x y z w

export type PartShape = 'box' | 'cylinder' | 'capsule' | 'sphere' | 'deck'

export interface Part {
  name: string
  shape: PartShape
  // box: [x, y, z] full size · cylinder: [rTop, rBottom, height, segments] (axis Y)
  // capsule: [radius, middleLength, capSegments, radialSegments] (axis Y) · sphere: [r, wSeg, hSeg]
  // deck: [a, b, notchY, depth] (extruded oval plate with notches and holes; see deckShape)
  args: number[]
  position: Vec3
  quaternion: Quat
  color: string
  metalness?: number
  roughness?: number
  emissive?: string
  emissiveIntensity?: number
  castShadow?: boolean
}

export const BASE = {
  wheelRadius: 0.0325,
  wheelWidth: 0.03,
  halfWheelbase: 0.085, // L: wheel centre distance from chassis centre along x
  halfTrack: 0.105, // W: wheel centre distance from chassis centre along y
}

export const DECK_TOP = 0.0915 // z of the upper deck's top surface (arm mounts here)
export const ARM_MOUNT: Vec3 = [0.03, 0, DECK_TOP]
export const PI_POS: Vec3 = [-0.095, 0, DECK_TOP]

export const WHEELS: { tag: 'fl' | 'fr' | 'rl' | 'rr'; position: Vec3; handed: 1 | -1 }[] = [
  { tag: 'fl', position: [BASE.halfWheelbase, BASE.halfTrack, BASE.wheelRadius], handed: 1 },
  { tag: 'fr', position: [BASE.halfWheelbase, -BASE.halfTrack, BASE.wheelRadius], handed: -1 },
  { tag: 'rl', position: [-BASE.halfWheelbase, BASE.halfTrack, BASE.wheelRadius], handed: -1 },
  { tag: 'rr', position: [-BASE.halfWheelbase, -BASE.halfTrack, BASE.wheelRadius], handed: 1 },
]

const ID: Quat = [0, 0, 0, 1]
const quatEuler = (x: number, y: number, z: number): Quat => {
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(x, y, z))
  return [q.x, q.y, q.z, q.w]
}

// Oval aluminium deck with a notch cut at each wheel, a round hole and slots (like the blue kit chassis).
export function deckShape(a: number, b: number, notchY: number): THREE.Shape {
  const n = 2.8 // superellipse exponent: oval with flattened sides
  const wx = BASE.halfWheelbase
  const pts: THREE.Vector2[] = []
  const N = 220
  for (let i = 0; i < N; i++) {
    const t = (i / N) * Math.PI * 2
    const c = Math.cos(t)
    const s = Math.sin(t)
    const x = a * Math.sign(c) * Math.abs(c) ** (2 / n)
    let y = b * Math.sign(s) * Math.abs(s) ** (2 / n)
    const inNotch = Math.abs(Math.abs(x) - wx) < 0.04
    if (inNotch) y = Math.sign(y) * Math.min(Math.abs(y), notchY)
    pts.push(new THREE.Vector2(x, y))
  }
  const shape = new THREE.Shape(pts)
  const hole = new THREE.Path()
  hole.absarc(-0.085, 0, 0.02, 0, Math.PI * 2, true)
  shape.holes.push(hole)
  for (const [sx, sy] of [[0.06, 0.04], [0.06, -0.04], [-0.01, 0.055], [-0.01, -0.055]] as const) {
    const slot = new THREE.Path()
    slot.moveTo(sx - 0.02, sy - 0.004)
    slot.lineTo(sx - 0.02, sy + 0.004)
    slot.lineTo(sx + 0.02, sy + 0.004)
    slot.lineTo(sx + 0.02, sy - 0.004)
    slot.closePath()
    shape.holes.push(slot)
  }
  return shape
}

export function makeGeometry(p: Part): THREE.BufferGeometry {
  const a = p.args
  switch (p.shape) {
    case 'box':
      return new THREE.BoxGeometry(a[0], a[1], a[2])
    case 'cylinder':
      return new THREE.CylinderGeometry(a[0], a[1], a[2], a[3])
    case 'capsule':
      return new THREE.CapsuleGeometry(a[0], a[1], a[2], a[3])
    case 'sphere':
      return new THREE.SphereGeometry(a[0], a[1], a[2])
    case 'deck':
      return new THREE.ExtrudeGeometry(deckShape(a[0], a[1], a[2]), { depth: a[3], bevelEnabled: false })
  }
}

const part = (name: string, shape: PartShape, args: number[], position: Vec3, color: string, extra: Partial<Part> = {}): Part => ({
  name,
  shape,
  args,
  position,
  quaternion: ID,
  color,
  ...extra,
})

const [px, py, pz] = PI_POS
const add = (p: Vec3, d: Vec3): Vec3 => [p[0] + d[0], p[1] + d[1], p[2] + d[2]]

export function chassisParts(): Part[] {
  const out: Part[] = [
    part('deck_lower', 'deck', [0.155, 0.1, 0.07, 0.003], [0, 0, 0.045], '#1c8fc4', { metalness: 0.65, roughness: 0.35, castShadow: true }),
    part('deck_upper', 'deck', [0.14, 0.092, 0.07, 0.003], [0, 0, 0.0885], '#1c8fc4', { metalness: 0.65, roughness: 0.35, castShadow: true }),
  ]
  const posts: [number, number][] = [[0.11, 0.06], [0.11, -0.06], [-0.11, 0.06], [-0.11, -0.06]]
  posts.forEach(([x, y], i) =>
    out.push(part(`standoff${i}`, 'cylinder', [0.004, 0.004, 0.04, 10], [x, y, 0.067], '#b9c2cc', { metalness: 0.8, roughness: 0.3, quaternion: quatEuler(Math.PI / 2, 0, 0) })),
  )
  out.push(part('battery', 'box', [0.12, 0.07, 0.03], [0.02, 0, 0.066], '#17779f', { metalness: 0.3, roughness: 0.5, castShadow: true }))
  return out
}

const ROLLERS = 9

// Parts of one wheel, relative to the wheel centre. Axle is the local Y axis (the wheel spins about Y).
export function wheelParts(handed: 1 | -1): Part[] {
  const R = BASE.wheelRadius
  const W = BASE.wheelWidth
  const out: Part[] = [
    part('rim', 'cylinder', [R * 0.78, R * 0.78, W * 0.8, 24], [0, 0, 0], '#f4c20d', { roughness: 0.5 }),
    part('hub', 'cylinder', [R * 0.25, R * 0.25, W * 0.95, 16], [0, 0, 0], '#111111', { roughness: 0.6 }),
  ]
  const up = new THREE.Vector3(0, 1, 0)
  for (let i = 0; i < ROLLERS; i++) {
    const a = (i / ROLLERS) * Math.PI * 2
    const tangent = new THREE.Vector3(-Math.sin(a), 0, Math.cos(a))
    const axis = tangent.multiplyScalar(Math.SQRT1_2).add(new THREE.Vector3(0, handed * Math.SQRT1_2, 0)).normalize()
    const q = new THREE.Quaternion().setFromUnitVectors(up, axis)
    out.push(
      part(`roller${i}`, 'capsule', [R * 0.22, W * 0.55, 4, 10], [R * Math.cos(a), 0, R * Math.sin(a)], '#26262b', {
        roughness: 0.8,
        quaternion: [q.x, q.y, q.z, q.w],
      }),
    )
  }
  return out
}

// Raspberry Pi on stand-offs (illustrative placeholder). Absolute robot-frame positions.
export function piParts(): Part[] {
  const silver = { color: '#c9ced6', metalness: 0.9, roughness: 0.3 }
  const out: Part[] = []
  ;[[-0.0365, -0.0245], [0.0365, -0.0245], [-0.0365, 0.0245], [0.0365, 0.0245]].forEach(([x, y], i) =>
    out.push(part(`pi_standoff${i}`, 'cylinder', [0.0025, 0.0025, 0.006, 8], add(PI_POS, [x, y, 0.003]), '#d6a84a', { metalness: 0.7, roughness: 0.4, quaternion: quatEuler(Math.PI / 2, 0, 0) })),
  )
  out.push(
    part('pi_pcb', 'box', [0.085, 0.056, 0.0016], add(PI_POS, [0, 0, 0.0068]), '#0e7a3b', { roughness: 0.5, castShadow: true }),
    part('pi_soc', 'box', [0.014, 0.014, 0.0055], add(PI_POS, [-0.008, 0, 0.0102]), silver.color, silver),
    part('pi_gpio', 'box', [0.051, 0.005, 0.0085], add(PI_POS, [0, 0.0235, 0.0102]), '#17181c', { roughness: 0.6 }),
    part('pi_usb0', 'box', [0.0175, 0.0135, 0.0145], add(PI_POS, [0.0345, -0.012, 0.0125]), silver.color, silver),
    part('pi_usb1', 'box', [0.0175, 0.0135, 0.0145], add(PI_POS, [0.0345, 0.008, 0.0125]), silver.color, silver),
    part('pi_eth', 'box', [0.02, 0.0155, 0.0105], add(PI_POS, [0.0345, 0.0, 0.0108]), silver.color, silver),
    part('pi_led_red', 'sphere', [0.0012, 8, 8], add(PI_POS, [-0.035, -0.02, 0.0085]), '#ff3030', { emissive: '#ff3030', emissiveIntensity: 1.5 }),
    part('pi_led_green', 'sphere', [0.0012, 8, 8], add(PI_POS, [-0.0325, -0.02, 0.0085]), '#35ff6a', { emissive: '#35ff6a', emissiveIntensity: 1.5 }),
  )
  return out
}
