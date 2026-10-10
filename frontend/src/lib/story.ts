import type { JointState } from './joints'
import { POSES } from './joints'

export type Focus = 'hero' | 'base' | 'arm' | 'eyes' | 'brain'

// Camera presets in three.js world coords (Y up; robot front is +X).
export const CAMERA: Record<Focus, { pos: [number, number, number]; target: [number, number, number] }> = {
  hero: { pos: [0.75, 0.42, 0.6], target: [0.05, 0.17, 0] },
  base: { pos: [0.42, 0.1, 0.42], target: [0.03, 0.05, 0.05] },
  arm: { pos: [0.7, 0.4, 0.2], target: [0.08, 0.22, 0] },
  eyes: { pos: [0.85, 0.4, 0.62], target: [0.1, 0.17, 0] },
  brain: { pos: [-0.35, 0.42, 0.45], target: [-0.07, 0.1, 0] },
}

export const ARM_POSE: Record<Focus, JointState> = {
  hero: { ...POSES.Ready, shoulder_pan: -15 },
  base: POSES.Rest,
  arm: POSES['Reach shelf'],
  eyes: { ...POSES.Ready, wrist_flex: 20, gripper: 20 },
  brain: POSES.Rest,
}
