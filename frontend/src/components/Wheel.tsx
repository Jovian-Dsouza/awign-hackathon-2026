import { forwardRef, useMemo } from 'react'
import * as THREE from 'three'
import { BASE } from '../lib/mecanum'

const ROLLERS = 9

// One mecanum wheel. Axle is the local Y axis; the returned group is what spins (rotation.y).
// `handed` flips the 45 degree roller direction (FL/RR vs FR/RL).
const Wheel = forwardRef<THREE.Group, { position: [number, number, number]; handed: 1 | -1 }>(
  ({ position, handed }, ref) => {
    const R = BASE.wheelRadius
    const rollers = useMemo(() => {
      const up = new THREE.Vector3(0, 1, 0)
      return Array.from({ length: ROLLERS }, (_, i) => {
        const a = (i / ROLLERS) * Math.PI * 2
        const tangent = new THREE.Vector3(-Math.sin(a), 0, Math.cos(a))
        const axis = tangent.multiplyScalar(Math.SQRT1_2).add(new THREE.Vector3(0, handed * Math.SQRT1_2, 0))
        const q = new THREE.Quaternion().setFromUnitVectors(up, axis.normalize())
        return { pos: [R * Math.cos(a), 0, R * Math.sin(a)] as [number, number, number], q }
      })
    }, [R, handed])

    return (
      <group position={position}>
        <group ref={ref}>
          <mesh>
            <cylinderGeometry args={[R * 0.78, R * 0.78, BASE.wheelWidth * 0.8, 24]} />
            <meshStandardMaterial color="#f4c20d" roughness={0.5} />
          </mesh>
          <mesh>
            <cylinderGeometry args={[R * 0.25, R * 0.25, BASE.wheelWidth * 0.95, 16]} />
            <meshStandardMaterial color="#111" roughness={0.6} />
          </mesh>
          {rollers.map((r, i) => (
            <mesh key={i} position={r.pos} quaternion={r.q}>
              <capsuleGeometry args={[R * 0.22, BASE.wheelWidth * 0.55, 4, 10]} />
              <meshStandardMaterial color="#26262b" roughness={0.8} />
            </mesh>
          ))}
        </group>
      </group>
    )
  },
)

export default Wheel
