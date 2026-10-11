// "Pick Rush" drive game: arena layout, products, collisions and pick/drop checks.
// Everything is in the robot's ROS frame (metres, x forward/east, y left/north, z up).
import { DECK_TOP } from './robotModel'

export type Pose = { x: number; y: number; yaw: number }
export type Box = { x: number; y: number; hx: number; hy: number }
export type Rack = Box & { id: string; height: number }

export const ARENA = { hx: 1.6, hy: 1.15 }
export const ROBOT_RADIUS = 0.16
export const START_POSE: Pose = { x: -1.25, y: -0.7, yaw: 0 }
// Where the gripper ends up in front of the base when reaching (robot frame, x).
export const REACH = 0.24
export const PICK_RANGE = 0.1
export const DROP_RANGE = 0.22

// Two rows of racks (all slots face south, -y) and a packing bench along the south wall.
export const RACKS: Rack[] = [
  { id: 'A', x: -1.0, y: 0.9, hx: 0.3, hy: 0.12, height: 0.55 },
  { id: 'B', x: 0.0, y: 0.9, hx: 0.3, hy: 0.12, height: 0.55 },
  { id: 'C', x: 1.0, y: 0.9, hx: 0.3, hy: 0.12, height: 0.55 },
  { id: 'D', x: -0.6, y: 0.05, hx: 0.3, hy: 0.12, height: 0.4 },
  { id: 'E', x: 0.6, y: 0.05, hx: 0.3, hy: 0.12, height: 0.4 },
]
export const BENCH: Box = { x: 0.2, y: -0.98, hx: 0.32, hy: 0.12 }
export const BENCH_HEIGHT = 0.16
export const TOTE = { x: 0.2, y: -0.95 }
export const DOCK = { x: START_POSE.x, y: START_POSE.y }
export const OBSTACLES: Box[] = [...RACKS, BENCH]

export type Shape = 'can' | 'box' | 'bottle'
export type Product = { name: string; shape: Shape; color: string }
// Names from the warehouse inventory (config/inventory.yaml).
export const PRODUCTS: Product[] = [
  { name: 'Ketchup', shape: 'bottle', color: '#d33a2c' },
  { name: 'Mustard', shape: 'bottle', color: '#e8b923' },
  { name: 'Cookies', shape: 'box', color: '#8a5a3c' },
  { name: 'Crackers', shape: 'box', color: '#e07b39' },
  { name: 'Popcorn', shape: 'box', color: '#f2d16b' },
  { name: 'Tomato soup', shape: 'can', color: '#c8432f' },
  { name: 'Tuna', shape: 'can', color: '#4f86c6' },
  { name: 'Milk', shape: 'box', color: '#f1f3f6' },
  { name: 'Orange juice', shape: 'box', color: '#f39c33' },
  { name: 'Water bottle', shape: 'bottle', color: '#67c4e8' },
  { name: 'Butter', shape: 'box', color: '#f4e3a1' },
  { name: 'Sugar', shape: 'box', color: '#e9ecef' },
  { name: 'BBQ sauce', shape: 'bottle', color: '#6b2e1f' },
  { name: 'Alphabet soup', shape: 'can', color: '#d8a03a' },
  { name: 'Cream cheese', shape: 'box', color: '#9ec5e8' },
]

export type Slot = { id: string; rack: string; x: number; y: number; z: number }
const SLOT_OFFSETS = [-0.2, 0, 0.2]
// Pickable slots: the bottom board of each rack, just inside its south face.
export const SLOTS: Slot[] = RACKS.flatMap((r) =>
  SLOT_OFFSETS.map((dx, i) => ({ id: `${r.id}${i + 1}`, rack: r.id, x: r.x + dx, y: r.y - r.hy + 0.05, z: 0.035 })),
)

export type ItemState = 'shelf' | 'carried' | 'delivered'
export type Item = { slot: Slot; product: Product; state: ItemState; target: boolean }

export function shuffled<T>(xs: T[]): T[] {
  const a = [...xs]
  for (let i = a.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1))
    ;[a[i], a[j]] = [a[j], a[i]]
  }
  return a
}

// Every slot gets a product; `count` of them are the current order.
export function stockShelves(count: number): Item[] {
  const products = shuffled(PRODUCTS)
  const targets = new Set(shuffled(SLOTS.map((s) => s.id)).slice(0, count))
  return SLOTS.map((slot, i) => ({ slot, product: products[i % products.length], state: 'shelf', target: targets.has(slot.id) }))
}

/** Push a circle out of an axis-aligned box. */
function resolveBox(p: { x: number; y: number }, b: Box, r: number) {
  const cx = Math.max(b.x - b.hx, Math.min(p.x, b.x + b.hx))
  const cy = Math.max(b.y - b.hy, Math.min(p.y, b.y + b.hy))
  const dx = p.x - cx
  const dy = p.y - cy
  const d2 = dx * dx + dy * dy
  if (d2 >= r * r) return false
  if (d2 > 1e-9) {
    const d = Math.sqrt(d2)
    p.x = cx + (dx / d) * r
    p.y = cy + (dy / d) * r
  } else {
    // Centre inside the box: leave by the nearest face.
    const pushes = [b.x + b.hx - p.x + r, p.x - (b.x - b.hx) + r, b.y + b.hy - p.y + r, p.y - (b.y - b.hy) + r]
    const m = Math.min(...pushes)
    if (m === pushes[0]) p.x += pushes[0]
    else if (m === pushes[1]) p.x -= pushes[1]
    else if (m === pushes[2]) p.y += pushes[2]
    else p.y -= pushes[3]
  }
  return true
}

/** Move the robot, keeping it inside the arena and out of racks. Returns true on a bump. */
export function collide(p: Pose): boolean {
  let hit = false
  for (const b of OBSTACLES) hit = resolveBox(p, b, ROBOT_RADIUS) || hit
  const mx = ARENA.hx - ROBOT_RADIUS
  const my = ARENA.hy - ROBOT_RADIUS
  if (Math.abs(p.x) > mx || Math.abs(p.y) > my) hit = true
  p.x = Math.max(-mx, Math.min(mx, p.x))
  p.y = Math.max(-my, Math.min(my, p.y))
  return hit
}

export function gripperPoint(p: Pose) {
  return { x: p.x + Math.cos(p.yaw) * REACH, y: p.y + Math.sin(p.yaw) * REACH }
}

export const dist = (a: { x: number; y: number }, b: { x: number; y: number }) => Math.hypot(a.x - b.x, a.y - b.y)

/** Bearing of a point relative to the robot's heading, radians (+ = to the left). */
export function bearing(p: Pose, to: { x: number; y: number }) {
  const a = Math.atan2(to.y - p.y, to.x - p.x) - p.yaw
  return Math.atan2(Math.sin(a), Math.cos(a))
}

// Spots on the deck where picked items ride (robot frame), between the Pi and the arm.
export const BASKET: [number, number, number][] = [
  [-0.03, 0.075, DECK_TOP + 0.002],
  [-0.03, -0.075, DECK_TOP + 0.002],
  [-0.03, 0, DECK_TOP + 0.002],
]

