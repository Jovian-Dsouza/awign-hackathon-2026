import { useEffect, useRef, useState, type ReactNode } from 'react'
import { createPortal, useFrame } from '@react-three/fiber'
import * as THREE from 'three'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { JOINTS, type JointState } from '../lib/joints'

// SO-101 follower with the stock wrist-camera mount and camera.
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

// Adds a silver lens ring to the wrist camera, facing the gripper's forward (-z of gripper_link).
function addLens(robot: URDFRobot) {
  const links = robot.links as Record<string, THREE.Object3D>
  const cam = links['wrist_camera_link']
  const grip = links['gripper_link']
  if (!cam || !grip) return
  let mesh: THREE.Mesh | null = null
  cam.traverse((o) => {
    if (!mesh && (o as THREE.Mesh).isMesh) mesh = o as THREE.Mesh
  })
  if (!mesh) return
  const geo = (mesh as THREE.Mesh).geometry
  geo.computeBoundingBox()
  const box = geo.boundingBox!
  const center = box.getCenter(new THREE.Vector3())
  const size = box.getSize(new THREE.Vector3())
  robot.updateMatrixWorld(true)
  const inv = new THREE.Matrix4().copy(cam.matrixWorld).invert()
  const fwd = new THREE.Vector3(0, 0, -1).transformDirection(grip.matrixWorld).transformDirection(inv).normalize()
  const half = (Math.abs(fwd.x) * size.x + Math.abs(fwd.y) * size.y + Math.abs(fwd.z) * size.z) / 2
  const g = new THREE.Group()
  g.position.copy(center).addScaledVector(fwd, half)
  g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), fwd)
  const ring = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0085, 0.0085, 0.004, 24),
    new THREE.MeshStandardMaterial({ color: '#b9bec6', metalness: 0.9, roughness: 0.3 }),
  )
  ring.position.y = 0.0015
  const glass = new THREE.Mesh(
    new THREE.CylinderGeometry(0.0052, 0.0052, 0.0045, 20),
    new THREE.MeshStandardMaterial({ color: '#05060a', roughness: 0.2 }),
  )
  glass.position.y = 0.0016
  g.add(ring, glass)
  cam.add(g)
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
      addLens(loaded)
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
  const wristLink = (robot.links as Record<string, THREE.Object3D>)['wrist_camera_link']
  return (
    <>
      <primitive object={robot} position={position} />
      {wristNode && wristLink ? createPortal(wristNode, wristLink) : null}
    </>
  )
}
