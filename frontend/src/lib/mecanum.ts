// Mecanum base geometry (metres) lives in robotModel.ts so the website and the simulation share it.
import { BASE } from './robotModel'

export { BASE }

export type Twist = { vx: number; vy: number; wz: number } // m/s, m/s, rad/s (robot frame, x fwd, y left)
export type WheelSpeeds = { fl: number; fr: number; rl: number; rr: number } // rad/s, + = rolls forward

// Standard mecanum inverse kinematics: body twist -> wheel angular speeds.
export function wheelSpeeds({ vx, vy, wz }: Twist): WheelSpeeds {
  const k = BASE.halfWheelbase + BASE.halfTrack
  const r = BASE.wheelRadius
  return {
    fl: (vx - vy - k * wz) / r,
    fr: (vx + vy + k * wz) / r,
    rl: (vx + vy - k * wz) / r,
    rr: (vx - vy + k * wz) / r,
  }
}
