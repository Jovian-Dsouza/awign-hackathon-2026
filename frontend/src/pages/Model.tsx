import { useEffect, useState } from 'react'
import { Link } from 'react-router-dom'
import RobotScene from '../components/RobotScene'
import ThemeToggle from '../components/ThemeToggle'
import { useResolvedTheme } from '../lib/theme'
import { JOINTS, POSES, REST, type JointState } from '../lib/joints'
import { wheelSpeeds, type Twist } from '../lib/mecanum'

const MAX_V = 0.3
const MAX_W = 1.5
const KEYS: Record<string, keyof Twist | undefined> = { w: 'vx', s: 'vx', a: 'vy', d: 'vy', q: 'wz', e: 'wz' }
const SIGN: Record<string, number> = { w: 1, s: -1, a: 1, d: -1, q: 1, e: -1 }

export default function Model() {
  const [joints, setJoints] = useState<JointState>(REST)
  const [slider, setSlider] = useState<Twist>({ vx: 0, vy: 0, wz: 0 })
  const [keys, setKeys] = useState<Set<string>>(new Set())
  const [resetKey, setResetKey] = useState(0)
  const theme = useResolvedTheme()

  useEffect(() => {
    const typing = (e: KeyboardEvent) => (e.target as HTMLElement)?.tagName === 'INPUT'
    const down = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase()
      if (!(k in KEYS) || typing(e)) return
      setKeys((s) => new Set(s).add(k))
    }
    const up = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase()
      setKeys((s) => {
        const n = new Set(s)
        n.delete(k)
        return n
      })
    }
    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
    }
  }, [])

  const twist: Twist = { ...slider }
  for (const k of keys) {
    const axis = KEYS[k]
    if (axis) twist[axis] += SIGN[k] * (axis === 'wz' ? MAX_W : MAX_V)
  }
  const wheels = wheelSpeeds(twist)

  return (
    <div className="model-page">
      <div className="viewport glass">
        <RobotScene twist={twist} joints={joints} resetKey={resetKey} theme={theme} />
        <div className="viewport-bar">
          <Link className="back glass-strong" to="/">
            ← Home
          </Link>
          <ThemeToggle />
        </div>
      </div>
      <aside className="panel glass">
        <h2>Arm — SO-101</h2>
        <div className="presets">
          {Object.entries(POSES).map(([name, pose]) => (
            <button key={name} className="pill" onClick={() => setJoints(pose)}>
              {name}
            </button>
          ))}
        </div>
        {JOINTS.map((j) => (
          <label key={j.name} className="row">
            <span>{j.label}</span>
            <input
              type="range"
              min={j.min}
              max={j.max}
              step={0.5}
              value={joints[j.name]}
              onChange={(e) => setJoints((s) => ({ ...s, [j.name]: Number(e.target.value) }))}
            />
            <output>{joints[j.name].toFixed(0)}°</output>
          </label>
        ))}

        <h2>Base — mecanum</h2>
        <p className="hint">Keys: W/S forward, A/D strafe, Q/E rotate</p>
        {(
          [
            ['vx', 'Forward', MAX_V, 'm/s'],
            ['vy', 'Strafe', MAX_V, 'm/s'],
            ['wz', 'Rotate', MAX_W, 'rad/s'],
          ] as const
        ).map(([k, label, max, unit]) => (
          <label key={k} className="row">
            <span>{label}</span>
            <input type="range" min={-max} max={max} step={0.01} value={slider[k]} onChange={(e) => setSlider((s) => ({ ...s, [k]: Number(e.target.value) }))} />
            <output>
              {twist[k].toFixed(2)} {unit}
            </output>
          </label>
        ))}
        <div className="wheels">
          {(['fl', 'fr', 'rl', 'rr'] as const).map((k) => (
            <div key={k}>
              <small>{k.toUpperCase()}</small>
              <b>{wheels[k].toFixed(1)}</b>
            </div>
          ))}
          <small className="unit">wheel rad/s</small>
        </div>
        <div className="presets">
          <button className="pill" onClick={() => setSlider({ vx: 0, vy: 0, wz: 0 })}>Stop</button>
          <button className="pill" onClick={() => setResetKey((n) => n + 1)}>Reset position</button>
        </div>
      </aside>
    </div>
  )
}
