import PartMesh from './PartMesh'
import { CHASSIS_CAM_POS, PI_POS, chassisCameraParts, piParts } from '../lib/robotModel'

// Illustrative placeholders: exact boards, mounts and cameras are not final.
export { CHASSIS_CAM_POS, PI_POS }

const PI = piParts()
const CAMERA = chassisCameraParts()

export function RaspberryPi() {
  return (
    <group>
      {PI.map((p) => (
        <PartMesh key={p.name} part={p} />
      ))}
    </group>
  )
}

export function ChassisCamera() {
  return (
    <group>
      {CAMERA.map((p) => (
        <PartMesh key={p.name} part={p} />
      ))}
    </group>
  )
}
