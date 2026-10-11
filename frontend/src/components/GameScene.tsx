import { useMemo, useRef, type MutableRefObject } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { ContactShadows, Environment, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import Arm from './Arm'
import Chassis from './Chassis'
import { RaspberryPi } from './Parts'
import Wheel from './Wheel'
import { BASE, wheelSpeeds, type Twist } from '../lib/mecanum'
import type { JointState } from '../lib/joints'
import { ARM_MOUNT } from '../lib/robotModel'
import {
  ARENA, BASKET, BENCH, BENCH_HEIGHT, DOCK, PRODUCTS, RACKS, TOTE, collide,
  type Item, type Pose, type Product, type Rack,
} from '../lib/game'

const { halfWheelbase: L, halfTrack: W, wheelRadius: R } = BASE
export type CameraMode = 'chase' | 'top' | 'orbit'

type Palette = { rack: string; board: string; wall: string; floorLine: string; bench: string; tote: string; brand: string }
const PALETTE: Record<'light' | 'dark', Palette> = {
  light: { rack: '#8f9aab', board: '#d7dde6', wall: '#c4ccd8', floorLine: '#e6b800', bench: '#b9c2cf', tote: '#156cdd', brand: '#156cdd' },
  dark: { rack: '#4a5260', board: '#2b313b', wall: '#20252d', floorLine: '#b8920a', bench: '#353c48', tote: '#156cdd', brand: '#3f8cf0' },
}

/** One product. Cylinders are Y-up in three.js, so they are turned upright for the Z-up frame. */
function ProductMesh({ product, scale = 1 }: { product: Product; scale?: number }) {
  const mat = <meshStandardMaterial color={product.color} roughness={0.45} metalness={product.shape === 'can' ? 0.5 : 0.05} />
  if (product.shape === 'box')
    return (
      <mesh castShadow position={[0, 0, 0.035 * scale]} scale={scale}>
        <boxGeometry args={[0.03, 0.045, 0.07]} />
        {mat}
      </mesh>
    )
  const [r, h] = product.shape === 'can' ? [0.021, 0.055] : [0.016, 0.085]
  return (
    <group scale={scale}>
      <mesh castShadow rotation={[Math.PI / 2, 0, 0]} position={[0, 0, h / 2]}>
        <cylinderGeometry args={[r, r, h, 20]} />
        {mat}
      </mesh>
      {product.shape === 'bottle' && (
        <mesh castShadow rotation={[Math.PI / 2, 0, 0]} position={[0, 0, h + 0.012]}>
          <cylinderGeometry args={[0.006, 0.01, 0.024, 12]} />
          <meshStandardMaterial color="#f5f5f5" />
        </mesh>
      )}
    </group>
  )
}

function RackMesh({ rack, c, decor }: { rack: Rack; c: Palette; decor: Product[] }) {
  const levels = rack.height > 0.5 ? [0.03, 0.2, 0.37, 0.54] : [0.03, 0.2, 0.37]
  const posts: [number, number][] = [
    [-rack.hx, -rack.hy], [rack.hx, -rack.hy], [-rack.hx, rack.hy], [rack.hx, rack.hy],
  ]
  return (
    <group position={[rack.x, rack.y, 0]}>
      {posts.map(([x, y]) => (
        <mesh key={`${x}${y}`} position={[x, y, rack.height / 2]} castShadow>
          <boxGeometry args={[0.02, 0.02, rack.height]} />
          <meshStandardMaterial color={c.rack} metalness={0.6} roughness={0.4} />
        </mesh>
      ))}
      {levels.map((z) => (
        <mesh key={z} position={[0, 0, z]} receiveShadow castShadow>
          <boxGeometry args={[rack.hx * 2, rack.hy * 2, 0.012]} />
          <meshStandardMaterial color={c.board} roughness={0.8} />
        </mesh>
      ))}
      {/* back-stock on the upper levels */}
      {levels.slice(1).flatMap((z, li) =>
        [-0.2, -0.07, 0.07, 0.2].map((dx, i) => (
          <group key={`${li}-${i}`} position={[dx, 0.02, z + 0.006]}>
            <ProductMesh product={decor[(li * 4 + i) % decor.length]} />
          </group>
        )),
      )}
    </group>
  )
}

/** Pulsing ring and a bobbing arrow over a pick or drop target. */
function Beacon({ x, y, z = 0, color }: { x: number; y: number; z?: number; color: string }) {
  const ring = useRef<THREE.Mesh>(null)
  const arrow = useRef<THREE.Mesh>(null)
  useFrame(({ clock }) => {
    const t = clock.elapsedTime
    if (ring.current) ring.current.scale.setScalar(1 + 0.15 * Math.sin(t * 4))
    if (arrow.current) arrow.current.position.z = z + 0.2 + 0.03 * Math.sin(t * 3)
  })
  return (
    <group position={[x, y, 0]}>
      <mesh ref={ring} position={[0, 0, 0.003]}>
        <ringGeometry args={[0.07, 0.09, 40]} />
        <meshBasicMaterial color={color} transparent opacity={0.85} />
      </mesh>
      <mesh ref={arrow} rotation={[-Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.025, 0.06, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <pointLight position={[0, 0, z + 0.12]} color={color} intensity={0.6} distance={0.5} />
    </group>
  )
}

function Arena({ c, items, deliverActive }: { c: Palette; items: Item[]; deliverActive: boolean }) {
  const decor = useMemo(() => [...PRODUCTS].reverse(), [])
  const wall = (x: number, y: number, sx: number, sy: number) => (
    <mesh key={`${x},${y}`} position={[x, y, 0.05]} receiveShadow>
      <boxGeometry args={[sx, sy, 0.1]} />
      <meshStandardMaterial color={c.wall} transparent opacity={0.85} />
    </mesh>
  )
  return (
    <group>
      <mesh position={[0, 0, -0.001]} receiveShadow>
        <planeGeometry args={[ARENA.hx * 2, ARENA.hy * 2]} />
        <meshStandardMaterial color={c.board} roughness={0.95} />
      </mesh>
      {wall(0, ARENA.hy + 0.02, ARENA.hx * 2 + 0.08, 0.04)}
      {wall(0, -ARENA.hy - 0.02, ARENA.hx * 2 + 0.08, 0.04)}
      {wall(ARENA.hx + 0.02, 0, 0.04, ARENA.hy * 2)}
      {wall(-ARENA.hx - 0.02, 0, 0.04, ARENA.hy * 2)}
      {/* yellow safety lines along the rack faces */}
      {[0.7, -0.15].map((y) => (
        <mesh key={y} position={[0, y, 0.001]}>
          <planeGeometry args={[ARENA.hx * 2 - 0.2, 0.012]} />
          <meshBasicMaterial color={c.floorLine} />
        </mesh>
      ))}
      {/* dock pad */}
      <mesh position={[DOCK.x, DOCK.y, 0.002]}>
        <planeGeometry args={[0.36, 0.36]} />
        <meshBasicMaterial color={c.brand} transparent opacity={0.18} />
      </mesh>
      {RACKS.map((r) => (
        <RackMesh key={r.id} rack={r} c={c} decor={decor} />
      ))}
      {/* packing bench + tote */}
      <mesh position={[BENCH.x, BENCH.y, BENCH_HEIGHT / 2]} castShadow receiveShadow>
        <boxGeometry args={[BENCH.hx * 2, BENCH.hy * 2, BENCH_HEIGHT]} />
        <meshStandardMaterial color={c.bench} />
      </mesh>
      <mesh position={[TOTE.x, TOTE.y, BENCH_HEIGHT + 0.03]} castShadow>
        <boxGeometry args={[0.16, 0.12, 0.06]} />
        <meshStandardMaterial color={c.tote} transparent opacity={0.9} />
      </mesh>
      {items.map((it) =>
        it.state === 'shelf' ? (
          <group key={it.slot.id} position={[it.slot.x, it.slot.y, it.slot.z]}>
            <ProductMesh product={it.product} />
          </group>
        ) : null,
      )}
      {items
        .filter((it) => it.target && it.state === 'shelf')
        .map((it) => (
          <Beacon key={`b-${it.slot.id}`} x={it.slot.x} y={it.slot.y - 0.09} z={0.08} color={c.brand} />
        ))}
      {deliverActive && <Beacon x={TOTE.x} y={BENCH.y + BENCH.hy + 0.1} z={BENCH_HEIGHT} color="#22c55e" />}
    </group>
  )
}

function Rover({
  inputRef, poseRef, joints, carried, onBump,
}: {
  inputRef: MutableRefObject<Twist>
  poseRef: MutableRefObject<Pose>
  joints: JointState
  carried: Product[]
  onBump?: () => void
}) {
  const root = useRef<THREE.Group>(null)
  const wheels = { fl: useRef<THREE.Group>(null), fr: useRef<THREE.Group>(null), rl: useRef<THREE.Group>(null), rr: useRef<THREE.Group>(null) }
  const bumped = useRef(false)

  useFrame((_, rawDt) => {
    const dt = Math.min(rawDt, 0.05)
    const p = poseRef.current
    const t = inputRef.current
    p.x += (t.vx * Math.cos(p.yaw) - t.vy * Math.sin(p.yaw)) * dt
    p.y += (t.vx * Math.sin(p.yaw) + t.vy * Math.cos(p.yaw)) * dt
    p.yaw += t.wz * dt
    const hit = collide(p)
    if (hit && !bumped.current) onBump?.()
    bumped.current = hit
    const g = root.current
    if (g) {
      g.position.set(p.x, p.y, 0)
      g.rotation.z = p.yaw
    }
    const w = wheelSpeeds(t)
    for (const k of ['fl', 'fr', 'rl', 'rr'] as const) {
      const wheel = wheels[k].current
      if (wheel) wheel.rotation.y += w[k] * dt
    }
  })

  return (
    <group ref={root}>
      <Chassis />
      <Wheel ref={wheels.fl} position={[L, W, R]} handed={1} />
      <Wheel ref={wheels.fr} position={[L, -W, R]} handed={-1} />
      <Wheel ref={wheels.rl} position={[-L, W, R]} handed={-1} />
      <Wheel ref={wheels.rr} position={[-L, -W, R]} handed={1} />
      <Arm joints={joints} position={ARM_MOUNT} />
      <RaspberryPi />
      {carried.slice(0, BASKET.length).map((p, i) => (
        <group key={i} position={BASKET[i]}>
          <ProductMesh product={p} scale={0.8} />
        </group>
      ))}
    </group>
  )
}

/** ROS (x, y, z) -> three.js (x, z, -y). */
const toThree = (x: number, y: number, z: number, out: THREE.Vector3) => out.set(x, z, -y)

function FollowCamera({ poseRef, mode }: { poseRef: MutableRefObject<Pose>; mode: CameraMode }) {
  const { camera } = useThree()
  const goal = useRef(new THREE.Vector3())
  const look = useRef(new THREE.Vector3())
  const lookGoal = useRef(new THREE.Vector3())
  useFrame((_, dt) => {
    if (mode === 'orbit') return
    const p = poseRef.current
    if (mode === 'chase') {
      toThree(p.x - Math.cos(p.yaw) * 0.85, p.y - Math.sin(p.yaw) * 0.85, 0.55, goal.current)
      toThree(p.x + Math.cos(p.yaw) * 0.45, p.y + Math.sin(p.yaw) * 0.45, 0.1, lookGoal.current)
    } else {
      toThree(p.x * 0.4, p.y * 0.4 - 1.4, 2.6, goal.current)
      toThree(p.x * 0.6, p.y * 0.6, 0, lookGoal.current)
    }
    const k = 1 - Math.exp(-dt * 4)
    camera.position.lerp(goal.current, k)
    look.current.lerp(lookGoal.current, k)
    camera.lookAt(look.current)
  })
  return null
}

export default function GameScene({
  theme, inputRef, poseRef, joints, items, carried, deliverActive, cameraMode, onBump,
}: {
  theme: 'light' | 'dark'
  inputRef: MutableRefObject<Twist>
  poseRef: MutableRefObject<Pose>
  joints: JointState
  items: Item[]
  carried: Product[]
  deliverActive: boolean
  cameraMode: CameraMode
  onBump?: () => void
}) {
  const light = theme === 'light'
  const c = PALETTE[theme]
  return (
    <Canvas shadows camera={{ position: [-2.2, 1.0, 1.4], fov: 50, near: 0.02 }} dpr={[1, 2]} gl={{ alpha: true }}>
      <hemisphereLight args={['#ffffff', light ? '#9aa3b2' : '#333344', light ? 0.9 : 0.55]} />
      <directionalLight position={[1.5, 3, 1]} intensity={1.5} castShadow shadow-mapSize={[2048, 2048]}>
        <orthographicCamera attach="shadow-camera" args={[-2.2, 2.2, 2.2, -2.2, 0.1, 8]} />
      </directionalLight>
      <Environment preset="city" />
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <Arena c={c} items={items} deliverActive={deliverActive} />
        <Rover inputRef={inputRef} poseRef={poseRef} joints={joints} carried={carried} onBump={onBump} />
      </group>
      <ContactShadows position={[0, 0.0005, 0]} opacity={light ? 0.3 : 0.45} scale={4} blur={2.5} far={0.6} />
      {cameraMode === 'orbit' ? (
        <OrbitControls makeDefault target={[0, 0.1, 0]} maxPolarAngle={Math.PI / 2.05} />
      ) : (
        <FollowCamera poseRef={poseRef} mode={cameraMode} />
      )}
    </Canvas>
  )
}
