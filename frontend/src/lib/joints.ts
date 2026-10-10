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
  // Where the real arm starts every episode of the working ACT pipeline: all 10 recordings in
  // data/real/so101_pick_test1_*/ begin at pan -3, lift -85.8, elbow 97, wrist flex 3.8, roll -81, gripper 1.3 (0-100).
  // The model's roll zero is 90 deg from the real calibration (real picks at ~0, level jaws here are -90), so the
  // real roll is -171, past the joint limit (-157.2): the nearest reachable value is used. Same pose as the simulation
  // (robot-sim/src/dsr_sim/robot.py POSES_DEG["rest"]).
  Rest: { shoulder_pan: -2.9, shoulder_lift: -85.8, elbow_flex: 96.8, wrist_flex: 3.8, wrist_roll: -157, gripper: -4.2 },
  Ready: { shoulder_pan: 0, shoulder_lift: -40, elbow_flex: 55, wrist_flex: 60, wrist_roll: 0, gripper: 40 },
  'Reach shelf': { shoulder_pan: 0, shoulder_lift: 10, elbow_flex: 10, wrist_flex: 0, wrist_roll: 0, gripper: 60 },
  Grasp: { shoulder_pan: 0, shoulder_lift: 10, elbow_flex: 10, wrist_flex: 0, wrist_roll: 0, gripper: 5 },
}

export const REST: JointState = POSES.Rest
