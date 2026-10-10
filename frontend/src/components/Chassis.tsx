import PartMesh from './PartMesh'
import { DECK_TOP, chassisParts } from '../lib/robotModel'

export { DECK_TOP }

const PARTS = chassisParts()

export default function Chassis() {
  return (
    <group>
      {PARTS.map((p) => (
        <PartMesh key={p.name} part={p} />
      ))}
    </group>
  )
}
