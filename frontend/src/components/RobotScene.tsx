import { useRef } from 'react'
import { Canvas, useFrame } from '@react-three/fiber'
import { ContactShadows, Environment, Grid, OrbitControls } from '@react-three/drei'
import * as THREE from 'three'
import Arm from './Arm'
import Chassis, { DECK_TOP } from './Chassis'
import Wheel from './Wheel'
import { BASE, wheelSpeeds, type Twist } from '../lib/mecanum'
import type { JointState } from '../lib/joints'

const { halfWheelbase: L, halfTrack: W, wheelRadius: R } = BASE
const ARM_MOUNT: [number, number, number] = [0.03, 0, DECK_TOP]

// Drivable rover: integrates the commanded twist into a pose and spins each wheel.
function Rover({ twist, joints }: { twist: Twist; joints: JointState }) {
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

  return (
    <group ref={root}>
      <Chassis />
      <Wheel ref={wheels.fl} position={[L, W, R]} handed={1} />
      <Wheel ref={wheels.fr} position={[L, -W, R]} handed={-1} />
      <Wheel ref={wheels.rl} position={[-L, W, R]} handed={-1} />
      <Wheel ref={wheels.rr} position={[-L, -W, R]} handed={1} />
      <Arm joints={joints} position={ARM_MOUNT} />
    </group>
  )
}

export default function RobotScene({ twist, joints, resetKey }: { twist: Twist; joints: JointState; resetKey: number }) {
  return (
    <Canvas shadows camera={{ position: [0.6, 0.45, 0.55], fov: 40 }} dpr={[1, 2]}>
      <color attach="background" args={['#0d0f14']} />
      <hemisphereLight args={['#ffffff', '#333344', 0.6]} />
      <directionalLight position={[2, 3, 2]} intensity={1.6} castShadow />
      <Environment preset="city" />
      <Grid args={[20, 20]} cellSize={0.1} sectionSize={0.5} cellColor="#2a2f3a" sectionColor="#3b4252" fadeDistance={6} infiniteGrid position={[0, 0, 0]} />
      {/* ROS frame is Z-up (x forward); three.js is Y-up. */}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        <Rover key={resetKey} twist={twist} joints={joints} />
      </group>
      <ContactShadows position={[0, 0.001, 0]} opacity={0.5} scale={4} blur={2.5} far={1} />
      <OrbitControls makeDefault target={[0, 0.15, 0]} maxPolarAngle={Math.PI / 2.02} />
    </Canvas>
  )
}
