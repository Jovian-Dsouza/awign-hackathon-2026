import PartMesh from './PartMesh'
import { PI_POS, piParts } from '../lib/robotModel'

// Illustrative placeholders: exact boards and mounts are not final.
export { PI_POS }

const PI = piParts()

export function RaspberryPi() {
  return (
    <group>
      {PI.map((p) => (
        <PartMesh key={p.name} part={p} />
      ))}
    </group>
  )
}
