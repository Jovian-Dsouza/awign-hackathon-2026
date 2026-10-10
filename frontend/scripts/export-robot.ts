// Exports the robot model (src/lib/robotModel.ts) for the MuJoCo simulation:
//   <outDir>/robot_parts.json  every part (primitives with colours/poses, meshes by file name)
//   <outDir>/deck_*.obj        extruded deck plates
// Usage: node scripts/export-robot.ts ../robot-sim/assets/robot
import { mkdirSync, writeFileSync } from 'node:fs'
import { join, resolve } from 'node:path'
import {
  ARM_MOUNT,
  BASE,
  CHASSIS_CAM_POS,
  DECK_TOP,
  PI_POS,
  WHEELS,
  chassisCameraParts,
  chassisParts,
  makeGeometry,
  piParts,
  wheelParts,
  type Part,
} from '../src/lib/robotModel.ts'

const outDir = resolve(process.argv[2] ?? '../robot-sim/assets/robot')
mkdirSync(outDir, { recursive: true })

function objOf(part: Part): string {
  const geo = makeGeometry(part).toNonIndexed()
  const pos = geo.getAttribute('position')
  const lines = ['# exported from frontend/src/lib/robotModel.ts']
  for (let i = 0; i < pos.count; i++) lines.push(`v ${pos.getX(i)} ${pos.getY(i)} ${pos.getZ(i)}`)
  for (let i = 0; i < pos.count; i += 3) lines.push(`f ${i + 1} ${i + 2} ${i + 3}`)
  return lines.join('\n') + '\n'
}

function withMesh(p: Part): Part & { mesh?: string } {
  if (p.shape !== 'deck') return p
  const file = `${p.name}.obj`
  writeFileSync(join(outDir, file), objOf(p))
  return { ...p, mesh: file }
}

const model = {
  units: 'm',
  frame: 'x forward, y left, z up',
  base: BASE,
  anchors: { deck_top: DECK_TOP, arm_mount: ARM_MOUNT, pi: PI_POS, chassis_cam: CHASSIS_CAM_POS },
  chassis: chassisParts().map(withMesh),
  pi: piParts(),
  chassis_camera: chassisCameraParts(),
  wheels: Object.fromEntries(WHEELS.map((w) => [w.tag, { position: w.position, handed: w.handed, parts: wheelParts(w.handed) }])),
}
writeFileSync(join(outDir, 'robot_parts.json'), JSON.stringify(model, null, 1))
console.log(`exported ${model.chassis.length + model.pi.length + model.chassis_camera.length} body parts + 4 wheels -> ${outDir}`)
