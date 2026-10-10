import { useEffect, useState } from 'react'
import { Link } from 'react-router-dom'
import RobotScene from '../components/RobotScene'
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

export default function Home() {
  const [focus, setFocus] = useState<Focus>('hero')

  useEffect(() => {
    const els = ['hero', ...SECTIONS.map((s) => s.id)].map((id) => document.getElementById(id)!)
    const io = new IntersectionObserver(
      (entries) => {
        for (const e of entries) if (e.isIntersecting) setFocus(e.target.id as Focus)
      },
      { rootMargin: '-45% 0px -45% 0px' },
    )
    els.forEach((el) => el && io.observe(el))
    return () => io.disconnect()
  }, [])

  return (
    <div className="landing">
      <header className="topbar">
        <span className="brand">Dark-Store Robot</span>
        <nav>
          <a href="#base">Base</a>
          <a href="#arm">Arm</a>
          <a href="#eyes">Camera</a>
          <a href="#brain">Brain</a>
          <Link className="cta dark" to="/model">
            Drive it
          </Link>
        </nav>
      </header>

      <div className="stage" aria-hidden>
        <RobotScene twist={STILL} joints={ARM_POSE[focus]} story={{ focus }} theme="light" />
        <div className="stage-foot">
          <span>FIG. {focus === 'hero' ? '00' : SECTIONS.find((s) => s.id === focus)!.fig} / {focus === 'hero' ? 'Overview' : SECTIONS.find((s) => s.id === focus)!.label}</span>
          <span>PROTOTYPE · PLACEMENT ILLUSTRATIVE</span>
        </div>
      </div>

      <main className="story">
        <section id="hero" className="panel-hero">
          <p className="eyebrow">A mobile picker for quick-commerce dark stores</p>
          <h1>
            Stop walking the aisles.
            <br />
            <span className="muted">Let the robot find it.</span>
          </h1>
          <p className="sub">
            A compact mecanum-base robot with an SO-101 arm and a wrist camera. It reads the shelf, spots what is out of place, and learns to pick it up.
          </p>
          <div className="buttons">
            <Link className="cta dark" to="/model">
              Drive the robot
            </Link>
            <a className="cta ghost" href="#base">
              How it is built
            </a>
          </div>
          <dl className="proof">
            <div>
              <dt>4</dt>
              <dd>mecanum wheels</dd>
            </div>
            <div>
              <dt>6</dt>
              <dd>arm joints</dd>
            </div>
            <div>
              <dt>2</dt>
              <dd>cameras</dd>
            </div>
          </dl>
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
          </section>
        ))}

        <section className="step end">
          <h2>
            Drive it yourself.
            <br />
            <span className="muted">Move the base. Pose the arm.</span>
          </h2>
          <div className="buttons">
            <Link className="cta dark" to="/model">
              Open the 3D model
            </Link>
          </div>
          <p className="fineprint">
            Camera and computer placement are illustrative while the build is in progress. Built at the Awign Physical AI Hackathon, Bengaluru, 10–11 Oct 2026.
          </p>
        </section>
      </main>
    </div>
  )
}
