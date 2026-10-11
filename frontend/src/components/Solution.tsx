import { Link } from 'react-router-dom'

// Where "Join the waitlist" goes (e.g. a Google Form). Set VITE_WAITLIST_URL in frontend/.env.
const WAITLIST_URL = (import.meta.env.VITE_WAITLIST_URL as string | undefined)?.trim() || null

// Product copy. Every feature listed exists in this repo today; what the hardware does not do yet
// is listed under ROADMAP instead of being implied.

const PROBLEMS = [
  {
    title: 'Every item is a walk.',
    body: 'Delivery is promised in minutes, but in a dark store a person still walks the aisles for each item, picks it and carries it to packing.',
  },
  {
    title: 'Shelves drift.',
    body: 'Items put back in the wrong slot stay invisible until a pick fails. Someone then has to find the item and restock it by hand.',
  },
  {
    title: 'Checks are manual.',
    body: 'Making sure the right item went into the right tote depends on a person looking. Mistakes surface when the customer opens the bag.',
  },
]

const PIPELINE = [
  { step: '01', title: 'Order in', body: 'Each order becomes one task per item, assigned to a robot on the dashboard.' },
  { step: '02', title: 'Drive to the shelf', body: 'The mecanum base strafes sideways into the slot, no three-point turns in narrow aisles.' },
  { step: '03', title: 'Look', body: 'A camera on the gripper sees the item from up close, exactly the view the picking policy uses.' },
  { step: '04', title: 'Pick', body: 'A learned policy turns the image and joint angles into arm motion, on board.' },
  { step: '05', title: 'Place', body: 'The item goes to the packing tray, or back into the rack it belongs in.' },
  { step: '06', title: 'Verify', body: 'Rules check the result and keep a camera frame as evidence. Anything odd goes to a person.' },
]

const FEATURES = [
  {
    title: 'Moves in any direction',
    body: 'Four mecanum wheels drive forward, sideways or rotate in place, so the robot lines up with a slot without turning around.',
    facts: ['4 independent motor channels', 'one velocity command', 'fits narrow aisles'],
  },
  {
    title: 'Learns to pick',
    body: 'An ACT transformer policy from the open LeRobot ecosystem runs on the robot itself. It sees the wrist camera and controls all six joints.',
    facts: ['runs on the Raspberry Pi', 'camera + 6 joints in', 'no cloud needed to run'],
  },
  {
    title: 'Teach it by showing',
    body: 'Move the arm by hand, or with a gamepad in the browser, to record demonstrations. Fine-tune the policy on a GPU and load it back.',
    facts: ['hand-guided or gamepad', 'LeRobot dataset format', 'one-script fine-tuning'],
  },
  {
    title: 'One control panel',
    body: 'A web page served by the robot drives the base and the arm together, with the live camera. Save arm poses and play them back.',
    facts: ['keyboard or Xbox controller', 'pose record / playback', 'any browser on the network'],
  },
  {
    title: 'Operations dashboard',
    body: 'Orders, per-item tasks, robot status and quality checks in one place, with human review for anything flagged.',
    facts: ['order → task → QC', 'approve / reject review', 'analytics from your records'],
  },
  {
    title: 'Camera from anywhere',
    body: 'The robot streams its camera to the dashboard over a secure tunnel, so a supervisor can watch from outside the store.',
    facts: ['no port forwarding', 'reconnects by itself', 'token-protected'],
  },
  {
    title: 'Safe by default',
    body: 'Try a new policy in preview mode first: it shows what it would do without moving. In motion, every step and the servo force are capped.',
    facts: ['preview mode, no motion', 'max 10° per control step', 'servo torque limit'],
  },
  {
    title: 'Tested in simulation first',
    body: 'A physics simulation of the same robot picks from shelves and restocks misplaced items, so new layouts are tried before the floor.',
    facts: ['MuJoCo digital twin', 'friction-only grasps', 'exports training data'],
  },
]

const INCLUDED = [
  'Mecanum base with motor controller',
  'SO-101 6-axis arm with gripper',
  'Wrist camera',
  'Raspberry Pi on-board computer',
  'Robot control panel',
  'Learned picking + teaching tools',
  'UthaLe operations dashboard',
]

const ROADMAP = [
  'Autonomous navigation on the robot. SLAM navigation works in simulation; today the base is driven from the panel.',
  'Dashboard dispatching to the real robot. The order workflow currently runs against a simulated robot.',
  'Weight and tag checks at packing. The load cell and NFC reader are simulated for now.',
  'Battery telemetry.',
]

const FAQ = [
  {
    q: 'Does it drive itself today?',
    a: 'Picking runs on the robot. Driving between shelves is done from the control panel for now. Fully autonomous navigation is proven in simulation and is the next step on the hardware.',
  },
  {
    q: 'How do I teach it a new item?',
    a: 'Record a few demonstrations by guiding the arm, fine-tune the policy on any GPU (a free notebook works) and load it onto the robot. Test it in preview mode before letting it move.',
  },
  {
    q: 'What can it pick?',
    a: 'Cans, bottles and cartons that fit the gripper. Very flat items lying on a shelf, like a roll of tape or a box cutter, are too thin for the fingers today.',
  },
  {
    q: 'Does it need the internet?',
    a: 'No for running: the policy and control panel run on the robot over your local network. Remote camera viewing uses an internet tunnel, and fine-tuning needs a GPU.',
  },
]

export function Problem() {
  return (
    <section id="problem" className="block">
      <p className="eyebrow">Built for dark stores</p>
      <h2>
        Ten-minute delivery runs on walking.
        <br />
        <span className="muted">Every pick is a person, every time.</span>
      </h2>
      <div className="grid three">
        {PROBLEMS.map((p) => (
          <article key={p.title} className="card glass">
            <h3>{p.title}</h3>
            <p>{p.body}</p>
          </article>
        ))}
      </div>
    </section>
  )
}

export default function Solution() {
  return (
    <div className="details">
      <section id="solution" className="block">
        <p className="eyebrow">How it works</p>
        <h2>
          From order to tote.
          <br />
          <span className="muted">Robot, policy and dashboard in one loop.</span>
        </h2>
        <p className="sub wide">
          UthaLe pairs a compact mobile manipulator with software that learns from your own staff's demonstrations, and a dashboard that turns orders into robot tasks and checks every result.
        </p>
        <ol className="pipeline">
          {PIPELINE.map((s) => (
            <li key={s.step} className="glass">
              <span className="step-no">{s.step}</span>
              <h3>{s.title}</h3>
              <p>{s.body}</p>
            </li>
          ))}
        </ol>
      </section>

      <section id="features" className="block">
        <p className="eyebrow">Features</p>
        <h2>
          Everything in the box works together.
          <br />
          <span className="muted">Open hardware, open models, your data.</span>
        </h2>
        <div className="grid four">
          {FEATURES.map((f) => (
            <article key={f.title} className="card glass feature">
              <h3>{f.title}</h3>
              <p>{f.body}</p>
              <ul>
                {f.facts.map((x) => (
                  <li key={x}>{x}</li>
                ))}
              </ul>
            </article>
          ))}
        </div>
      </section>

      <section id="pricing" className="block">
        <p className="eyebrow">Pricing</p>
        <h2>
          One price per robot.
          <br />
          <span className="muted">Add robots as your store grows.</span>
        </h2>
        <div className="pricing">
          <article className="card glass-strong price-card">
            <p className="plan">UthaLe robot</p>
            <p className="price">
              $1,500 <span>per robot</span>
            </p>
            <ul className="checks">
              {INCLUDED.map((x) => (
                <li key={x}>{x}</li>
              ))}
            </ul>
            {WAITLIST_URL ? (
              <a className="cta primary" href={WAITLIST_URL} target="_blank" rel="noopener noreferrer">
                Join the waitlist
              </a>
            ) : (
              <button className="cta primary" disabled title="Waitlist link not set (VITE_WAITLIST_URL)">
                Join the waitlist
              </button>
            )}
          </article>
          <article className="card glass roadmap">
            <p className="plan">On the roadmap</p>
            <p>Shipping as pilot units. Here is what is not on the hardware yet, stated plainly:</p>
            <ul>
              {ROADMAP.map((x) => (
                <li key={x}>{x}</li>
              ))}
            </ul>
          </article>
        </div>
      </section>

      <section id="faq" className="block">
        <p className="eyebrow">Questions</p>
        <h2>Before you buy.</h2>
        <div className="grid two">
          {FAQ.map((f) => (
            <article key={f.q} className="card glass">
              <h3>{f.q}</h3>
              <p>{f.a}</p>
            </article>
          ))}
        </div>
      </section>

      <section className="block end">
        <h2>
          Take the wheel.
          <br />
          <span className="muted">Pack orders against the clock in Pick Rush.</span>
        </h2>
        <div className="buttons">
          <Link className="cta primary" to="/model">
            Play Pick Rush
          </Link>
        </div>
        <p className="fineprint">
          Camera and computer placement on the 3D model are illustrative. Built at the Awign Physical AI Hackathon, Bengaluru, 10–11 Oct 2026.
        </p>
      </section>
    </div>
  )
}
