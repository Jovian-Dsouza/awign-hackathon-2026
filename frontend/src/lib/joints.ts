// SO-101 follower joints, in degrees. Zero is the middle of each joint's range
// ("new calibration", the same convention LeRobot uses with use_degrees=True).
export const JOINTS = [
  { name: 'shoulder_pan', label: 'Shoulder pan', min: -110, max: 110 },
  { name: 'shoulder_lift', label: 'Shoulder lift', min: -100, max: 100 },
  { name: 'elbow_flex', label: 'Elbow flex', min: -96.8, max: 96.8 },
  { name: 'wrist_flex', label: 'Wrist flex', min: -95, max: 95 },
  { name: 'wrist_roll', label: 'Wrist roll', min: -157.2, max: 162.8 },
  { name: 'gripper', label: 'Gripper', min: -10, max: 100 },
] as const

export type JointName = (typeof JOINTS)[number]['name']
export type JointState = Record<JointName, number>

export const POSES: Record<string, JointState> = {
  Rest: { shoulder_pan: 0, shoulder_lift: -100, elbow_flex: 95, wrist_flex: 65, wrist_roll: 0, gripper: 0 },
  Ready: { shoulder_pan: 0, shoulder_lift: -40, elbow_flex: 55, wrist_flex: 60, wrist_roll: 0, gripper: 40 },
  'Reach shelf': { shoulder_pan: 0, shoulder_lift: 10, elbow_flex: 10, wrist_flex: 0, wrist_roll: 0, gripper: 60 },
  Grasp: { shoulder_pan: 0, shoulder_lift: 10, elbow_flex: 10, wrist_flex: 0, wrist_roll: 0, gripper: 5 },
}

export const REST: JointState = POSES.Rest
