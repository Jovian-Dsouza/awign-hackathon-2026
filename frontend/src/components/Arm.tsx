import { useEffect, useRef, useState, type ReactNode } from 'react'
import { createPortal, useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { JOINTS, type JointState } from '../lib/joints'

// SO-101 follower; the stock wrist-camera mount is hidden and replaced by the bar camera built on the real arm.
const URDF_URL = '/models/so101/so101_new_calib_camera.urdf'

// Recolour to match the printed-black arm in the photo; keep motors darker than printed parts.
function restyle(robot: URDFRobot) {
  robot.traverse((o) => {
    const mesh = o as THREE.Mesh
    if (!mesh.isMesh) return
    const motor = (mesh.material as THREE.Material).name === 'sts3215'
    mesh.material = new THREE.MeshStandardMaterial({
      color: motor ? '#0c0c0e' : '#2b2b30',
      roughness: motor ? 0.5 : 0.65,
      metalness: 0.05,
    })
    mesh.castShadow = true
  })
}

// Bar webcam on a printed bracket above the gripper, as built on the real arm (see the photos). All numbers are in
// gripper_link coordinates (x = jaw-opening direction, +y = camera side, -z = forward) and match
// robot-sim/src/dsr_sim/robot.py (WRIST_CAM_*), which mirrors y because the sim's grasp roll has the opposite sign.
const PRINTED = '#2b2b30'
const WRIST_PARTS: { pos: [number, number, number]; size: [number, number, number]; color: string }[] = [
  { pos: [0, 0.043, -0.02], size: [0.022, 0.034, 0.034], color: PRINTED }, // post glued to the gripper top
  { pos: [0, 0.0615, -0.03], size: [0.05, 0.006, 0.04], color: PRINTED }, // platform
  { pos: [-0.027, 0.07, -0.03], size: [0.005, 0.016, 0.04], color: PRINTED }, // cheeks
  { pos: [0.027, 0.07, -0.03], size: [0.005, 0.016, 0.04], color: PRINTED },
  { pos: [0, 0.0755, -0.034], size: [0.092, 0.024, 0.026], color: '#0c0c0e' }, // bar camera
  { pos: [0, 0.0755, -0.0475], size: [0.04, 0.016, 0.003], color: '#17171a' }, // front bezel
]
const WRIST_LENS: [number, number, number] = [0, 0.0755, -0.0495]
const WRIST_CABLE: [number, number, number][] = [
  [-0.046, 0.0755, -0.034],
  [-0.056, 0.074, -0.018],
  [-0.05, 0.052, 0.02],
  [-0.034, 0.03, 0.062],
]

// Replaces the stock wrist-camera mount with the real bracket + bar camera + thin lead.
// Returns the group, which sits at the lens so callouts can attach to it.
function addWristCamera(robot: URDFRobot): THREE.Group | null {
  const links = robot.links as Record<string, THREE.Object3D>
  const grip = links['gripper_link']
  if (!grip) return null
  for (const name of ['wrist_camera_mount_link', 'wrist_camera_link']) {
    links[name]?.traverse((o) => {
      if ((o as THREE.Mesh).isMesh) o.visible = false
    })
  }
  const mat = (color: string, roughness = 0.6, metalness = 0.05) =>
    new THREE.MeshStandardMaterial({ color, roughness, metalness })
  const g = new THREE.Group()
  g.name = 'wrist_bar_camera'
  const add = (m: THREE.Mesh, pos: [number, number, number]) => {
    m.position.set(...pos)
    m.castShadow = true
    g.add(m)
    return m
  }
  for (const p of WRIST_PARTS) add(new THREE.Mesh(new THREE.BoxGeometry(...p.size), mat(p.color)), p.pos)
  // Lens: cylinders along z (forward).
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.0085, 0.0085, 0.0024, 28), mat('#b9bec6', 0.3, 0.9))
  ring.rotation.x = Math.PI / 2
  add(ring, WRIST_LENS)
  const glass = new THREE.Mesh(new THREE.CylinderGeometry(0.0058, 0.0058, 0.0024, 24), mat('#05060a', 0.2))
  glass.rotation.x = Math.PI / 2
  add(glass, [WRIST_LENS[0], WRIST_LENS[1], WRIST_LENS[2] - 0.0008])
  // Thin flat USB lead running back along the arm.
  const curve = new THREE.CatmullRomCurve3(WRIST_CABLE.map((p) => new THREE.Vector3(...p)))
  const cable = new THREE.Mesh(new THREE.TubeGeometry(curve, 24, 0.0014, 6), mat('#0a0a0c', 0.7))
  cable.castShadow = true
  g.add(cable)
  grip.add(g)
  return g
}

export default function Arm({
  joints,
  position,
  wristNode,
}: {
  joints: JointState
  position: [number, number, number]
  wristNode?: ReactNode // rendered attached to the wrist camera link (for callouts)
}) {
  const [robot, setRobot] = useState<URDFRobot | null>(null)
  const target = useRef(joints)
  target.current = joints
  const current = useRef<Record<string, number> | null>(null)

  useEffect(() => {
    let alive = true
    const loader = new URDFLoader()
    let loaded: URDFRobot | null = null
    // STL meshes stream in after the URDF parses, so recolour once the loading manager is idle.
    loader.manager.onLoad = () => {
      if (!alive || !loaded) return
      restyle(loaded)
      addWristCamera(loaded)
      setRobot(loaded)
    }
    loader.load(URDF_URL, (r) => {
      loaded = r
    })
    return () => {
      alive = false
    }
  }, [])

  // Ease each joint toward its target so pose changes are smooth.
  useFrame((_, dt) => {
    if (!robot) return
    const cur = (current.current ??= Object.fromEntries(JOINTS.map((j) => [j.name, target.current[j.name]])))
    const k = 1 - Math.exp(-dt * 7)
    for (const j of JOINTS) {
      cur[j.name] += (target.current[j.name] - cur[j.name]) * k
      robot.setJointValue(j.name, THREE.MathUtils.degToRad(cur[j.name]))
    }
  })

  if (!robot) return null
  const wristLink = (robot.links as Record<string, THREE.Object3D>)['gripper_link']?.getObjectByName('wrist_bar_camera')
  return (
    <>
      <primitive object={robot} position={position} />
      {wristNode && wristLink ? createPortal(wristNode, wristLink) : null}
    </>
  )
}
