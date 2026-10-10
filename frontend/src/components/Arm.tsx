import { useEffect, useState } from 'react'
import * as THREE from 'three'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { JOINTS, type JointState } from '../lib/joints'

const URDF_URL = '/models/so101/so101_new_calib.urdf'

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

export default function Arm({ joints, position }: { joints: JointState; position: [number, number, number] }) {
  const [robot, setRobot] = useState<URDFRobot | null>(null)

  useEffect(() => {
    let alive = true
    const loader = new URDFLoader()
    let loaded: URDFRobot | null = null
    // STL meshes stream in after the URDF parses, so recolour once the loading manager is idle.
    loader.manager.onLoad = () => {
      if (!alive || !loaded) return
      restyle(loaded)
      setRobot(loaded)
    }
    loader.load(URDF_URL, (r) => {
      loaded = r
    })
    return () => {
      alive = false
    }
  }, [])

  useEffect(() => {
    if (!robot) return
    for (const j of JOINTS) robot.setJointValue(j.name, THREE.MathUtils.degToRad(joints[j.name]))
  }, [robot, joints])

  return robot ? <primitive object={robot} position={position} /> : null
}
