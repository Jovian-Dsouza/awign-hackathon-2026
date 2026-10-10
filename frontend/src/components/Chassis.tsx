import { useMemo } from 'react'
import * as THREE from 'three'
import { BASE } from '../lib/mecanum'

// Oval aluminium deck with a notch cut at each wheel, a round hole and slots (like the blue kit chassis).
function deckShape(a: number, b: number, notchY: number): THREE.Shape {
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

function Deck({ z, a, b, notchY }: { z: number; a: number; b: number; notchY: number }) {
  const geo = useMemo(() => new THREE.ExtrudeGeometry(deckShape(a, b, notchY), { depth: 0.003, bevelEnabled: false }), [a, b, notchY])
  return (
    <mesh geometry={geo} position={[0, 0, z]} castShadow receiveShadow>
      <meshStandardMaterial color="#1c8fc4" metalness={0.65} roughness={0.35} />
    </mesh>
  )
}

export const DECK_TOP = 0.0915 // z of the upper deck's top surface (arm mounts here)

export default function Chassis() {
  const posts: [number, number][] = [[0.11, 0.06], [0.11, -0.06], [-0.11, 0.06], [-0.11, -0.06]]
  return (
    <group>
      <Deck z={0.045} a={0.155} b={0.1} notchY={0.07} />
      <Deck z={0.0885} a={0.14} b={0.092} notchY={0.07} />
      {posts.map(([x, y], i) => (
        <mesh key={i} position={[x, y, 0.067]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.004, 0.004, 0.04, 10]} />
          <meshStandardMaterial color="#b9c2cc" metalness={0.8} roughness={0.3} />
        </mesh>
      ))}
      {/* Battery box between decks */}
      <mesh position={[0.02, 0, 0.066]} castShadow>
        <boxGeometry args={[0.12, 0.07, 0.03]} />
        <meshStandardMaterial color="#17779f" metalness={0.3} roughness={0.5} />
      </mesh>
    </group>
  )
}
