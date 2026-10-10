import { useRef } from 'react'
import { Canvas, useFrame, useThree } from '@react-three/fiber'
import { ContactShadows, Environment, Grid, Html, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import Arm from './Arm'
import Chassis from './Chassis'
import { ChassisCamera, CHASSIS_CAM_POS, PI_POS, RaspberryPi } from './Parts'
import Wheel from './Wheel'
import { BASE, wheelSpeeds, type Twist } from '../lib/mecanum'
import type { JointState } from '../lib/joints'
import { CAMERA, type Focus } from '../lib/story'
import { ARM_MOUNT, DECK_TOP } from '../lib/robotModel'

const { halfWheelbase: L, halfTrack: W, wheelRadius: R } = BASE

function Callout({ position, title, line }: { position: [number, number, number]; title: string; line: string }) {
  return (
    <Html position={position} center zIndexRange={[10, 0]} style={{ pointerEvents: 'none' }}>
      <div className="callout">
        <b>{title}</b>
        <span>{line}</span>
      </div>
    </Html>
  )
}

// Drivable rover: integrates the commanded twist into a pose and spins each wheel.
function Rover({ twist, joints, focus }: { twist: Twist; joints: JointState; focus?: Focus }) {
  const root = useRef<THREE.Group>(null)
  const wheels = { fl: useRef<THREE.Group>(null), fr: useRef<THREE.Group>(null), rl: useRef<THREE.Group>(null), rr: useRef<THREE.Group>(null) }
  const twistRef = useRef(twist)
  twistRef.current = twist

  useFrame((_, dt) => {
    const g = root.current
    if (!g) return
    const t = twistRef.current
    const yaw = g.rotation.z
    g.position.x += (t.vx * Math.cos(yaw) - t.vy * Math.sin(yaw)) * dt
    g.position.y += (t.vx * Math.sin(yaw) + t.vy * Math.cos(yaw)) * dt
    g.rotation.z += t.wz * dt
    const w = wheelSpeeds(t)
    for (const k of ['fl', 'fr', 'rl', 'rr'] as const) {
      const wheel = wheels[k].current
      if (wheel) wheel.rotation.y += w[k] * dt
    }
  })

  const wristLabel =
    focus === 'eyes' ? <Callout position={[0, 0.1, -0.05]} title="Wrist camera" line="close view for grasping" /> : undefined

  return (
    <group ref={root}>
      <Chassis />
      <Wheel ref={wheels.fl} position={[L, W, R]} handed={1} />
      <Wheel ref={wheels.fr} position={[L, -W, R]} handed={-1} />
      <Wheel ref={wheels.rl} position={[-L, W, R]} handed={-1} />
      <Wheel ref={wheels.rr} position={[-L, -W, R]} handed={1} />
      <Arm joints={joints} position={ARM_MOUNT} wristNode={wristLabel} />
      <RaspberryPi />
      <ChassisCamera />
      {focus === 'base' && <Callout position={[L, W, 0.1]} title="4 × mecanum wheels" line="forward · sideways · rotate" />}
      {focus === 'arm' && <Callout position={[0.03, 0, DECK_TOP + 0.13]} title="SO-101 arm" line="6 joints" />}
      {focus === 'eyes' && (
        <Callout position={[CHASSIS_CAM_POS[0] + 0.02, 0, CHASSIS_CAM_POS[2] + 0.04]} title="Chassis camera" line="looks down the aisle" />
      )}
      {focus === 'brain' && <Callout position={[PI_POS[0], 0, PI_POS[2] + 0.05]} title="Raspberry Pi" line="onboard computer" />}
    </group>
  )
}

// Eases the camera to the preset for the active story section.
function CameraRig({ focus }: { focus: Focus }) {
  const { camera } = useThree()
  const look = useRef(new THREE.Vector3(...CAMERA.hero.target))
  const goalPos = useRef(new THREE.Vector3())
  const goalLook = useRef(new THREE.Vector3())
  useFrame((_, dt) => {
    const p = CAMERA[focus]
    goalPos.current.set(...p.pos)
    goalLook.current.set(...p.target)
    const k = 1 - Math.exp(-dt * 3)
    camera.position.lerp(goalPos.current, k)
    look.current.lerp(goalLook.current, k)
    camera.lookAt(look.current)
  })
  return null
}

export default function RobotScene({
  twist,
  joints,
  resetKey = 0,
  story,
  theme = 'dark',
}: {
  twist: Twist
  joints: JointState
  resetKey?: number
  story?: { focus: Focus }
  theme?: 'dark' | 'light'
}) {
  const light = theme === 'light'
  return (
    <Canvas shadows camera={{ position: CAMERA.hero.pos, fov: story ? 34 : 40 }} dpr={[1, 2]}>
      <color attach="background" args={[light ? '#f3f4f6' : '#0d0f14']} />
      <hemisphereLight args={['#ffffff', light ? '#9aa3b2' : '#333344', light ? 0.9 : 0.6]} />
      <directionalLight position={[2, 3, 2]} intensity={1.6} castShadow />
      <Environment preset="city" />
      <Grid
        args={[20, 20]}
        cellSize={0.1}
        sectionSize={0.5}
        cellColor={light ? '#d9dde4' : '#2a2f3a'}
        sectionColor={light ? '#c3c9d3' : '#3b4252'}
        fadeDistance={6}
        infiniteGrid
      />
      {/* ROS frame is Z-up (x forward); three.js is Y-up. */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <Rover key={resetKey} twist={twist} joints={joints} focus={story?.focus} />
      </group>
      <ContactShadows position={[0, 0.001, 0]} opacity={light ? 0.35 : 0.5} scale={4} blur={2.5} far={1} />
      {story ? <CameraRig focus={story.focus} /> : <OrbitControls makeDefault target={[0, 0.15, 0]} maxPolarAngle={Math.PI / 2.02} />}
    </Canvas>
  )
}
