import { useMemo } from 'react'
import { makeGeometry, type Part } from '../lib/robotModel'

// Renders one part of the shared robot model.
export default function PartMesh({ part }: { part: Part }) {
  const geometry = useMemo(() => makeGeometry(part), [part])
  return (
    <mesh geometry={geometry} position={part.position} quaternion={part.quaternion} castShadow={part.castShadow} receiveShadow={part.shape === 'deck'}>
      <meshStandardMaterial
        color={part.color}
        metalness={part.metalness ?? 0}
        roughness={part.roughness ?? 0.6}
        emissive={part.emissive ?? '#000000'}
        emissiveIntensity={part.emissiveIntensity ?? 0}
      />
    </mesh>
  )
}
