import { useEffect, useState } from 'react'
import { Link } from 'react-router-dom'
import RobotScene from '../components/RobotScene'
import RobotSketch from '../components/RobotSketch'
import Solution, { Problem } from '../components/Solution'
import ThemeToggle from '../components/ThemeToggle'
import { useResolvedTheme } from '../lib/theme'
import { ARM_POSE, type Focus } from '../lib/story'

const STILL = { vx: 0, vy: 0, wz: 0 }

type Spec = [string, string]
const SECTIONS: { id: Focus; fig: string; label: string; title: [string, string]; body: string; specs: Spec[] }[] = [
  {
    id: 'base',
    fig: '01',
    label: 'The base',
    title: ['Moves any direction.', 'Even sideways.'],
    body: 'Four mecanum wheels let the base strafe and rotate in place, so it can line up with a rack slot in a narrow aisle without turning around.',
    specs: [
      ['wheels', '4 × mecanum'],
      ['motion', 'forward · sideways · rotate'],
      ['chassis', '2-deck aluminium plate'],
      ['control', 'body velocity → 4 wheel speeds'],
    ],
  },
  {
    id: 'arm',
    fig: '02',
    label: 'The arm',
    title: ['Reaches the shelf.', 'Picks it up.'],
    body: 'An SO-101 arm from the LeRobot ecosystem: six joints, 3D-printed links and bus servos, controlled in joint angles.',
    specs: [
      ['arm', 'SO-101 (LeRobot)'],
      ['joints', 'pan · lift · elbow · wrist flex · wrist roll · gripper'],
      ['control', 'joint angles, degrees'],
      ['gripper', 'parallel jaw'],
    ],
  },
  {
    id: 'eyes',
    fig: '03',
    label: 'The eye',
    title: ['One camera.', 'Right at the claw.'],
    body: 'A USB bar camera on a 3D-printed bracket rides on the gripper and looks straight at the item. It is the only camera: the learned picking policy sees what the gripper sees.',
    specs: [
      ['wrist camera', 'USB bar camera on a printed bracket, on the gripper'],
      ['stream', '640 × 480 · 30 fps'],
      ['feeds', 'learned picking policy'],
    ],
  },
  {
    id: 'brain',
    fig: '04',
    label: 'The brain',
    title: ['A Pi in charge.', 'Open software on top.'],
    body: 'A Raspberry Pi is the onboard computer. Navigation uses ROS 2 SLAM; picking uses a transformer policy trained with LeRobot.',
    specs: [
      ['computer', 'Raspberry Pi'],
      ['navigation', 'ROS 2 SLAM'],
      ['picking', 'LeRobot · ACT policy'],
      ['dashboard', 'web app, live inventory'],
    ],
  },
]

// The 3D intro section is #robot in the page but drives the scene's 'hero' camera preset.
const focusOf = (id: string): Focus => (id === 'robot' ? 'hero' : (id as Focus))

export default function Home() {
  const [focus, setFocus] = useState<Focus>('hero')
  const theme = useResolvedTheme()

  useEffect(() => {
    const els = ['robot', ...SECTIONS.map((s) => s.id)].map((id) => document.getElementById(id)!)
    const io = new IntersectionObserver(
      (entries) => {
        for (const e of entries) if (e.isIntersecting) setFocus(focusOf(e.target.id))
      },
      { rootMargin: '-45% 0px -45% 0px' },
    )
    els.forEach((el) => el && io.observe(el))
    return () => io.disconnect()
  }, [])

  const current = SECTIONS.find((s) => s.id === focus)

  return (
    <div className="landing">
      <header className="topbar glass">
        <span className="brand">
          <span className="brand-mark" aria-hidden>
            <svg width="16" height="16" viewBox="0 0 32 32"><path d="M9 8v9a7 7 0 0 0 14 0V8" fill="none" stroke="#fff" strokeWidth="3.5" strokeLinecap="round" /><circle cx="16" cy="17" r="2.5" fill="#fff" /></svg>
          </span>
          UthaLe
        </span>
        <nav>
          <a href="#problem">Problem</a>
          <a href="#robot">Robot</a>
          <a href="#solution">How it works</a>
          <a href="#features">Features</a>
          <a href="#pricing">Pricing</a>
          <ThemeToggle />
          <Link className="cta primary sm" to="/model">
            Play
          </Link>
        </nav>
      </header>

      <div className="details">
        <section className="intro">
          <div className="intro-copy">
            <p className="eyebrow">For quick-commerce dark stores</p>
            <h1>
              Ten-minute delivery.
              <br />
              <span className="muted">Still picked by hand.</span>
            </h1>
            <p className="sub">
              In a dark store, every item of every order is walked to, picked and packed by a person. UthaLe is a mobile picking robot built to take the picks off their hands.
            </p>
            <div className="buttons">
              <a className="cta primary" href="#problem">
                See the problem
              </a>
              <a className="cta ghost" href="#robot">
                Meet the robot
              </a>
            </div>
            <dl className="proof">
              <div className="glass">
                <dt>4</dt>
                <dd>mecanum wheels</dd>
              </div>
              <div className="glass">
                <dt>6</dt>
                <dd>arm joints</dd>
              </div>
              <div className="glass">
                <dt>1</dt>
                <dd>camera at the gripper</dd>
              </div>
            </dl>
          </div>
          <div className="intro-art glass">
            <RobotSketch />
          </div>
        </section>

        <Problem />
      </div>

      <div className="story-wrap">
        <div className="stage" aria-hidden>
          <RobotScene twist={STILL} joints={ARM_POSE[focus]} story={{ focus }} theme={theme} />
          <div className="stage-foot">
            <span>FIG. {current ? current.fig : '00'} / {current ? current.label : 'Overview'}</span>
            <span>PILOT HARDWARE · PLACEMENT ILLUSTRATIVE</span>
          </div>
        </div>

        <main className="story">
          <section id="robot" className="panel-hero">
            <p className="eyebrow">Meet UthaLe</p>
            <h2>
              One robot. Four parts.
              <br />
              <span className="muted">Scroll to take it apart.</span>
            </h2>
            <p className="sub">
              A compact mecanum base, an SO-101 arm, a camera at the gripper and a Raspberry Pi. Every part is off-the-shelf or 3D-printed.
            </p>
            <p className="built">Built with LeRobot · ROS 2 · Raspberry Pi</p>
          </section>

          {SECTIONS.map((s) => (
            <section key={s.id} id={s.id} className="step">
              <p className="eyebrow mono">
                {s.fig} / {s.label.toUpperCase()}
              </p>
              <h2>
                {s.title[0]}
                <br />
                <span className="muted">{s.title[1]}</span>
              </h2>
              <p className="sub">{s.body}</p>
              <div className="specs-card glass">
                <table className="specs">
                  <tbody>
                    {s.specs.map(([k, v]) => (
                      <tr key={k}>
                        <th>{k}</th>
                        <td>{v}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </section>
          ))}
        </main>
      </div>

      <Solution />
    </div>
  )
}
