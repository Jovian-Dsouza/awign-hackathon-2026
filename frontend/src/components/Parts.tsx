import { DECK_TOP } from './Chassis'

// Illustrative placeholders: exact boards, mounts and cameras are not final.
// Positions are in the robot frame (x forward, y left, z up), metres.

export const PI_POS: [number, number, number] = [-0.095, 0, DECK_TOP]
export const CHASSIS_CAM_POS: [number, number, number] = [0.13, 0, DECK_TOP + 0.02]

export function RaspberryPi() {
  const silver = <meshStandardMaterial color="#c9ced6" metalness={0.9} roughness={0.3} />
  return (
    <group position={PI_POS}>
      {/* standoffs */}
      {[[-0.0365, -0.0245], [0.0365, -0.0245], [-0.0365, 0.0245], [0.0365, 0.0245]].map(([x, y], i) => (
        <mesh key={i} position={[x, y, 0.003]} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry args={[0.0025, 0.0025, 0.006, 8]} />
          <meshStandardMaterial color="#d6a84a" metalness={0.7} roughness={0.4} />
        </mesh>
      ))}
      {/* PCB */}
      <mesh position={[0, 0, 0.0068]} castShadow>
        <boxGeometry args={[0.085, 0.056, 0.0016]} />
        <meshStandardMaterial color="#0e7a3b" roughness={0.5} />
      </mesh>
      {/* SoC with metal lid */}
      <mesh position={[-0.008, 0, 0.0102]}>
        <boxGeometry args={[0.014, 0.014, 0.0055]} />
        {silver}
      </mesh>
      {/* GPIO header along one long edge */}
      <mesh position={[0, 0.0235, 0.0102]}>
        <boxGeometry args={[0.051, 0.005, 0.0085]} />
        <meshStandardMaterial color="#17181c" roughness={0.6} />
      </mesh>
      {/* USB + Ethernet block at the short edge */}
      <mesh position={[0.0345, -0.012, 0.0125]}>
        <boxGeometry args={[0.0175, 0.0135, 0.0145]} />
        {silver}
      </mesh>
      <mesh position={[0.0345, 0.008, 0.0125]}>
        <boxGeometry args={[0.0175, 0.0135, 0.0145]} />
        {silver}
      </mesh>
      <mesh position={[0.0345, 0.0, 0.0108]}>
        <boxGeometry args={[0.02, 0.0155, 0.0105]} />
        {silver}
      </mesh>
      {/* status LEDs */}
      <mesh position={[-0.035, -0.02, 0.0085]}>
        <sphereGeometry args={[0.0012, 8, 8]} />
        <meshStandardMaterial color="#ff3030" emissive="#ff3030" emissiveIntensity={1.5} />
      </mesh>
      <mesh position={[-0.0325, -0.02, 0.0085]}>
        <sphereGeometry args={[0.0012, 8, 8]} />
        <meshStandardMaterial color="#35ff6a" emissive="#35ff6a" emissiveIntensity={1.5} />
      </mesh>
    </group>
  )
}

// Black USB bar camera across the front of the top deck: textured grip pad on one side,
// silver-ringed lens and a red LED on the other (matches the photo; dimensions approximate).
export function ChassisCamera() {
  return (
    <group position={CHASSIS_CAM_POS}>
      <mesh position={[-0.004, 0, -0.022]}>
        <boxGeometry args={[0.016, 0.03, 0.024]} />
        <meshStandardMaterial color="#2b2b30" roughness={0.65} />
      </mesh>
      <mesh castShadow>
        <boxGeometry args={[0.014, 0.085, 0.024]} />
        <meshStandardMaterial color="#0f0f12" roughness={0.55} />
      </mesh>
      <mesh position={[0.0075, 0.02, 0]}>
        <boxGeometry args={[0.002, 0.04, 0.02]} />
        <meshStandardMaterial color="#1c1c20" roughness={0.9} />
      </mesh>
      <mesh position={[0.0075, -0.022, 0.002]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.0085, 0.0085, 0.003, 24]} />
        <meshStandardMaterial color="#b9bec6" metalness={0.9} roughness={0.3} />
      </mesh>
      <mesh position={[0.0092, -0.022, 0.002]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.0052, 0.0052, 0.002, 20]} />
        <meshStandardMaterial color="#05060a" roughness={0.2} />
      </mesh>
      <mesh position={[0.0078, -0.022, -0.0085]}>
        <sphereGeometry args={[0.0013, 8, 8]} />
        <meshStandardMaterial color="#ff2a2a" emissive="#ff2a2a" emissiveIntensity={1.5} />
      </mesh>
    </group>
  )
}
