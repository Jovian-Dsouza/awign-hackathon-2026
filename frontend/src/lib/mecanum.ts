// Mecanum base geometry (metres). Matches the blue 4-wheel chassis in the build photo.
export const BASE = {
  wheelRadius: 0.0325,
  wheelWidth: 0.03,
  halfWheelbase: 0.085, // L: wheel centre distance from chassis centre along x
  halfTrack: 0.105, // W: wheel centre distance from chassis centre along y
}

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
