import { forwardRef, useMemo } from 'react'
import * as THREE from 'three'
import PartMesh from './PartMesh'
import { wheelParts } from '../lib/robotModel'

// One mecanum wheel. Axle is the local Y axis; the inner group is what spins (rotation.y).
// `handed` flips the 45 degree roller direction (FL/RR vs FR/RL).
const Wheel = forwardRef<THREE.Group, { position: [number, number, number]; handed: 1 | -1 }>(({ position, handed }, ref) => {
  const parts = useMemo(() => wheelParts(handed), [handed])
  return (
    <group position={position}>
      <group ref={ref}>
        {parts.map((p) => (
          <PartMesh key={p.name} part={p} />
        ))}
      </group>
    </group>
  )
})

export default Wheel
