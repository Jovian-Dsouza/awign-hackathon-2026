import { useCallback, useEffect, useRef, useState, type PointerEvent } from 'react'
import { Link } from 'react-router-dom'
import GameScene, { type CameraMode } from '../components/GameScene'
import ThemeToggle from '../components/ThemeToggle'
import { useResolvedTheme } from '../lib/theme'
import { JOINTS, POSES, REST, type JointState } from '../lib/joints'
import type { Twist } from '../lib/mecanum'
import {
  ARENA, BENCH, DROP_RANGE, PICK_RANGE, RACKS, START_POSE, TOTE,
  bearing, dist, gripperPoint, stockShelves, type Item, type Pose, type Product,
} from '../lib/game'

const MAX_V = 0.4
const MAX_W = 1.8
const TURBO = 1.6
const START_TIME = 90
const BEST_KEY = 'uthale-pick-rush-best'

// Keyboard (and on-screen pad) -> one of the base's three axes.
const KEYMAP: Record<string, { axis: keyof Twist; sign: number }> = {
  w: { axis: 'vx', sign: 1 }, arrowup: { axis: 'vx', sign: 1 },
  s: { axis: 'vx', sign: -1 }, arrowdown: { axis: 'vx', sign: -1 },
  a: { axis: 'vy', sign: 1 }, d: { axis: 'vy', sign: -1 },
  q: { axis: 'wz', sign: 1 }, arrowleft: { axis: 'wz', sign: 1 },
  e: { axis: 'wz', sign: -1 }, arrowright: { axis: 'wz', sign: -1 },
}

const ARM = {
  carry: { ...POSES.Ready, gripper: 5 },
  reach: POSES['Reach shelf'],
  grasp: POSES.Grasp,
  drop: { shoulder_pan: 0, shoulder_lift: -5, elbow_flex: 30, wrist_flex: 30, wrist_roll: 0, gripper: 70 },
}

type Mode = 'menu' | 'play' | 'paused' | 'over' | 'free'
type Hud = { time: number; prompt: string; bearing: number; distance: number; pose: Pose; bump: boolean }

function readBest(): number {
  try {
    return Number(localStorage.getItem(BEST_KEY)) || 0
  } catch {
    return 0
  }
}
function saveBest(n: number) {
  try {
    localStorage.setItem(BEST_KEY, String(n))
  } catch {
    /* storage unavailable: best score just isn't remembered */
  }
}

const orderSize = (done: number) => Math.min(3, 1 + Math.floor(done / 2))
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms))

export default function Model() {
  const theme = useResolvedTheme()
  const [mode, setMode] = useState<Mode>('menu')
  const [joints, setJoints] = useState<JointState>(REST)
  const [items, setItems] = useState<Item[]>(() => stockShelves(1))
  const [carried, setCarried] = useState<Product[]>([])
  const [score, setScore] = useState(0)
  const [ordersDone, setOrdersDone] = useState(0)
  const [best, setBest] = useState(readBest)
  const [toast, setToast] = useState<string | null>(null)
  const [cameraMode, setCameraMode] = useState<CameraMode>('chase')
  const [slider, setSlider] = useState<Twist>({ vx: 0, vy: 0, wz: 0 })
  const [hud, setHud] = useState<Hud>({ time: START_TIME, prompt: '', bearing: 0, distance: 0, pose: START_POSE, bump: false })

  const poseRef = useRef<Pose>({ ...START_POSE })
  const inputRef = useRef<Twist>({ vx: 0, vy: 0, wz: 0 })
  const keys = useRef(new Set<string>())
  const turbo = useRef(false)
  const busy = useRef(false)
  const timeLeft = useRef(START_TIME)
  const bumpAt = useRef(0)
  // Latest values for the event handlers and the HUD loop.
  const live = useRef({ mode, items, carried, slider, score, ordersDone })
  live.current = { mode, items, carried, slider, score, ordersDone }

  const updateInput = useCallback(() => {
    const t: Twist = live.current.mode === 'free' ? { ...live.current.slider } : { vx: 0, vy: 0, wz: 0 }
    const driving = live.current.mode === 'play' || live.current.mode === 'free'
    if (driving && !busy.current) {
      const boost = turbo.current ? TURBO : 1
      for (const k of keys.current) {
        const m = KEYMAP[k]
        if (m) t[m.axis] += m.sign * (m.axis === 'wz' ? MAX_W : MAX_V * boost)
      }
    } else {
      t.vx = t.vy = t.wz = 0
    }
    inputRef.current = t
  }, [])
  useEffect(updateInput, [mode, slider, updateInput])

  const flash = useCallback((msg: string) => {
    setToast(msg)
    window.setTimeout(() => setToast((m) => (m === msg ? null : m)), 1400)
  }, [])

  const reset = useCallback((next: Mode) => {
    poseRef.current = { ...START_POSE }
    timeLeft.current = START_TIME
    busy.current = false
    keys.current.clear()
    setItems(stockShelves(1))
    setCarried([])
    setScore(0)
    setOrdersDone(0)
    setJoints(ARM.carry)
    setMode(next)
  }, [])

  const targets = items.filter((it) => it.target)
  const orderReady = targets.length > 0 && targets.every((it) => it.state === 'carried')

  // Space: pick the target in front of the gripper, or pack a complete order at the tote.
  const act = useCallback(async () => {
    const { mode: m, items: its, carried: car, ordersDone: done } = live.current
    if ((m !== 'play' && m !== 'free') || busy.current) return
    const pose = poseRef.current
    const grip = gripperPoint(pose)
    const pick = its.find((it) => it.target && it.state === 'shelf' && dist(grip, it.slot) < PICK_RANGE)
    const ready = its.some((it) => it.target) && its.filter((it) => it.target).every((it) => it.state === 'carried')

    if (pick) {
      busy.current = true
      updateInput()
      setJoints(ARM.reach)
      await sleep(450)
      setJoints(ARM.grasp)
      await sleep(350)
      setItems((xs) => xs.map((it) => (it.slot.id === pick.slot.id ? { ...it, state: 'carried' } : it)))
      setCarried((c) => [...c, pick.product])
      setJoints(ARM.carry)
      await sleep(350)
      busy.current = false
      updateInput()
      return
    }
    if (ready && dist(grip, TOTE) < DROP_RANGE) {
      busy.current = true
      updateInput()
      setJoints(ARM.drop)
      await sleep(600)
      const n = car.length
      const gained = n * 100 + 50
      const bonus = 6 + n * 3
      setCarried([])
      setScore((s) => s + gained)
      setOrdersDone(done + 1)
      if (m === 'play') timeLeft.current += bonus
      flash(m === 'play' ? `Order packed · +${gained} · +${bonus}s` : 'Order packed')
      setJoints(ARM.carry)
      await sleep(400)
      setItems(stockShelves(orderSize(done + 1)))
      busy.current = false
      updateInput()
      return
    }
    flash(ready ? 'Get closer to the tote' : 'Nothing to pick here')
  }, [flash, updateInput])

  // Keyboard.
  useEffect(() => {
    const typing = (e: KeyboardEvent) => (e.target as HTMLElement)?.tagName === 'INPUT'
    const down = (e: KeyboardEvent) => {
      if (typing(e)) return
      const k = e.key.toLowerCase()
      const m = live.current.mode
      if (k in KEYMAP || k === ' ') e.preventDefault()
      if (k === 'shift') turbo.current = true
      if (k in KEYMAP) keys.current.add(k)
      if (k === ' ' && !e.repeat) void act()
      if (k === 'c' && !e.repeat) setCameraMode((c) => (c === 'chase' ? 'top' : c === 'top' ? 'orbit' : 'chase'))
      if ((k === 'escape' || k === 'p') && (m === 'play' || m === 'paused')) setMode(m === 'play' ? 'paused' : 'play')
      if (k === 'enter' && (m === 'menu' || m === 'over')) reset('play')
      updateInput()
    }
    const up = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase()
      if (k === 'shift') turbo.current = false
      keys.current.delete(k)
      updateInput()
    }
    const blur = () => {
      keys.current.clear()
      updateInput()
    }
    window.addEventListener('keydown', down)
    window.addEventListener('keyup', up)
    window.addEventListener('blur', blur)
    return () => {
      window.removeEventListener('keydown', down)
      window.removeEventListener('keyup', up)
      window.removeEventListener('blur', blur)
    }
  }, [act, reset, updateInput])

  // HUD + game clock, 10 times a second.
  useEffect(() => {
    let last = performance.now()
    const id = window.setInterval(() => {
      const now = performance.now()
      const dt = (now - last) / 1000
      last = now
      const { mode: m, items: its } = live.current
      if (m === 'play') {
        timeLeft.current = Math.max(0, timeLeft.current - dt)
        if (timeLeft.current === 0) {
          const s = live.current.score
          if (s > readBest()) {
            saveBest(s)
            setBest(s)
          }
          keys.current.clear()
          setMode('over')
        }
      }
      const pose = { ...poseRef.current }
      const grip = gripperPoint(pose)
      const tgts = its.filter((it) => it.target)
      const remaining = tgts.filter((it) => it.state === 'shelf')
      const ready = tgts.length > 0 && remaining.length === 0
      let goal = { x: TOTE.x, y: BENCH.y + BENCH.hy + 0.2 }
      let prompt = 'Drive to the packing tote'
      if (!ready) {
        const near = [...remaining].sort((a, b) => dist(pose, a.slot) - dist(pose, b.slot))[0]
        goal = { x: near.slot.x, y: near.slot.y - 0.25 }
        prompt = dist(grip, near.slot) < PICK_RANGE ? `SPACE · pick ${near.product.name}` : `Drive to the ${near.product.name} on rack ${near.slot.rack}`
      } else if (dist(grip, TOTE) < DROP_RANGE) {
        prompt = 'SPACE · pack the order'
      }
      setHud({ time: timeLeft.current, prompt, bearing: bearing(pose, goal), distance: dist(pose, goal), pose, bump: now - bumpAt.current < 400 })
    }, 100)
    return () => window.clearInterval(id)
  }, [])

  const onBump = useCallback(() => {
    bumpAt.current = performance.now()
  }, [])

  // On-screen pad for touch screens.
  const pad = (k: string) => ({
    onPointerDown: (e: PointerEvent) => {
      e.preventDefault()
      keys.current.add(k)
      updateInput()
    },
    onPointerUp: () => {
      keys.current.delete(k)
      updateInput()
    },
    onPointerLeave: () => {
      keys.current.delete(k)
      updateInput()
    },
  })

  const playing = mode === 'play' || mode === 'free'
  const mm = (x: number, y: number) => ({ x: x + ARENA.hx, y: ARENA.hy - y }) // metres -> minimap coords

  return (
    <div className={`game mode-${mode}`}>
      <div className="game-canvas">
        <GameScene
          theme={theme}
          inputRef={inputRef}
          poseRef={poseRef}
          joints={joints}
          items={items}
          carried={carried}
          deliverActive={orderReady}
          cameraMode={cameraMode}
          onBump={onBump}
        />
      </div>

      {/* top bar */}
      <div className="hud-top">
        <Link className="back glass-strong" to="/">
          ← Home
        </Link>
        {mode !== 'free' && mode !== 'menu' && (
          <div className="hud-stats glass-strong">
            <div className={hud.time < 10 && mode === 'play' ? 'warn' : ''}>
              <small>Time</small>
              <b>{Math.ceil(hud.time)}s</b>
            </div>
            <div>
              <small>Score</small>
              <b>{score}</b>
            </div>
            <div>
              <small>Orders</small>
              <b>{ordersDone}</b>
            </div>
            <div>
              <small>Best</small>
              <b>{best}</b>
            </div>
          </div>
        )}
        <div className="hud-actions">
          <button className="pill glass-strong" onClick={() => setCameraMode((c) => (c === 'chase' ? 'top' : c === 'top' ? 'orbit' : 'chase'))} title="Camera (C)">
            Cam: {cameraMode}
          </button>
          {mode === 'play' && (
            <button className="pill glass-strong" onClick={() => setMode('paused')}>
              Pause
            </button>
          )}
          <ThemeToggle />
        </div>
      </div>

      {/* order card */}
      {playing && (
        <div className="order-card glass-strong">
          <p className="plan">Order #{1040 + ordersDone + 1}</p>
          <ul>
            {targets.map((it) => (
              <li key={it.slot.id} className={it.state}>
                <span className="dot" style={{ background: it.product.color }} />
                {it.product.name}
                <small>{it.state === 'shelf' ? `rack ${it.slot.rack}` : 'in basket'}</small>
              </li>
            ))}
          </ul>
          <div className="nav-hint">
            <svg width="28" height="28" viewBox="-14 -14 28 28" style={{ transform: `rotate(${-hud.bearing}rad)` }} aria-hidden>
              <path d="M0 -11 L7 6 L0 2 L-7 6 Z" fill="currentColor" />
            </svg>
            <span>{hud.distance.toFixed(1)} m</span>
          </div>
        </div>
      )}

      {/* minimap */}
      {playing && (
        <svg className="minimap glass-strong" viewBox={`-0.08 -0.08 ${ARENA.hx * 2 + 0.16} ${ARENA.hy * 2 + 0.16}`} aria-label="Minimap">
          <rect x={0} y={0} width={ARENA.hx * 2} height={ARENA.hy * 2} rx={0.06} className="mm-floor" />
          {RACKS.map((r) => (
            <rect key={r.id} {...(({ x, y }) => ({ x: x - r.hx, y: y - r.hy }))(mm(r.x, r.y))} width={r.hx * 2} height={r.hy * 2} className="mm-rack" />
          ))}
          <rect {...(({ x, y }) => ({ x: x - BENCH.hx, y: y - BENCH.hy }))(mm(BENCH.x, BENCH.y))} width={BENCH.hx * 2} height={BENCH.hy * 2} className={orderReady ? 'mm-tote ready' : 'mm-tote'} />
          {targets
            .filter((it) => it.state === 'shelf')
            .map((it) => (
              <circle key={it.slot.id} {...(({ x, y }) => ({ cx: x, cy: y }))(mm(it.slot.x, it.slot.y))} r={0.07} className="mm-target" />
            ))}
          <g transform={`translate(${mm(hud.pose.x, hud.pose.y).x} ${mm(hud.pose.x, hud.pose.y).y}) rotate(${(-hud.pose.yaw * 180) / Math.PI})`}>
            <path d="M0.14 0 L-0.09 0.09 L-0.05 0 L-0.09 -0.09 Z" className="mm-robot" />
          </g>
        </svg>
      )}

      {/* prompt */}
      {playing && <div className={`prompt glass-strong ${hud.prompt.startsWith('SPACE') ? 'go' : ''}`}>{hud.prompt}</div>}
      {toast && <div className="toast glass-strong">{toast}</div>}
      {hud.bump && playing && <div className="bump">Bump!</div>}

      {/* controls legend */}
      {playing && (
        <div className="legend glass-strong">
          <span><kbd>W</kbd><kbd>S</kbd> drive</span>
          <span><kbd>A</kbd><kbd>D</kbd> strafe</span>
          <span><kbd>Q</kbd><kbd>E</kbd> turn</span>
          <span><kbd>Shift</kbd> turbo</span>
          <span><kbd>Space</kbd> pick / pack</span>
          <span><kbd>C</kbd> camera</span>
        </div>
      )}

      {/* touch pad */}
      {playing && (
        <div className="touchpad">
          <div className="pad-grid">
            <button {...pad('q')} aria-label="Turn left">⟲</button>
            <button {...pad('w')} aria-label="Forward">▲</button>
            <button {...pad('e')} aria-label="Turn right">⟳</button>
            <button {...pad('a')} aria-label="Strafe left">◀</button>
            <button {...pad('s')} aria-label="Back">▼</button>
            <button {...pad('d')} aria-label="Strafe right">▶</button>
          </div>
          <button className="pad-act" onPointerDown={(e) => { e.preventDefault(); void act() }}>
            PICK
          </button>
        </div>
      )}

      {/* free-drive arm panel */}
      {mode === 'free' && (
        <aside className="garage panel glass-strong">
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
              <input type="range" min={j.min} max={j.max} step={0.5} value={joints[j.name]} onChange={(e) => setJoints((s) => ({ ...s, [j.name]: Number(e.target.value) }))} />
              <output>{joints[j.name].toFixed(0)}°</output>
            </label>
          ))}
          <h2>Base — mecanum</h2>
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
                {slider[k].toFixed(2)} {unit}
              </output>
            </label>
          ))}
          <div className="presets">
            <button className="pill" onClick={() => setSlider({ vx: 0, vy: 0, wz: 0 })}>Stop</button>
            <button className="pill" onClick={() => (poseRef.current = { ...START_POSE })}>Back to dock</button>
            <button className="pill" onClick={() => reset('menu')}>Exit free drive</button>
          </div>
        </aside>
      )}

      {/* menus */}
      {mode === 'menu' && (
        <div className="overlay">
          <div className="menu glass-strong">
            <p className="eyebrow">UthaLe · drive challenge</p>
            <h1>Pick Rush</h1>
            <p className="sub">
              Orders are coming in. Drive the robot to the glowing items, pick them into its basket, then pack the order at the blue tote. Every packed order adds time to the clock.
            </p>
            <ol className="howto">
              <li><b>Drive</b> with W A S D, turn with Q E. Mecanum wheels let you strafe sideways.</li>
              <li><b>Line up</b> the gripper with the glowing item and press <kbd>Space</kbd>.</li>
              <li><b>Pack</b> the full order at the tote with <kbd>Space</kbd>.</li>
            </ol>
            <div className="buttons">
              <button className="cta primary" onClick={() => reset('play')}>
                Start · Enter
              </button>
              <button className="cta ghost" onClick={() => reset('free')}>
                Free drive
              </button>
            </div>
            {best > 0 && <p className="fineprint">Your best: {best}</p>}
          </div>
        </div>
      )}
      {mode === 'paused' && (
        <div className="overlay">
          <div className="menu glass-strong">
            <h1>Paused</h1>
            <div className="buttons">
              <button className="cta primary" onClick={() => setMode('play')}>
                Resume · Esc
              </button>
              <button className="cta ghost" onClick={() => reset('menu')}>
                Quit
              </button>
            </div>
          </div>
        </div>
      )}
      {mode === 'over' && (
        <div className="overlay">
          <div className="menu glass-strong">
            <p className="eyebrow">Time's up</p>
            <h1>{score} points</h1>
            <p className="sub">
              {ordersDone} {ordersDone === 1 ? 'order' : 'orders'} packed.{' '}
              {score > 0 && score >= best ? 'New best!' : `Best: ${best}`}
            </p>
            <div className="buttons">
              <button className="cta primary" onClick={() => reset('play')}>
                Play again · Enter
              </button>
              <button className="cta ghost" onClick={() => reset('free')}>
                Free drive
              </button>
              <a className="cta ghost" href="/#pricing">
                Join the waitlist
              </a>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
