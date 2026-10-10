# -*- coding: utf-8 -*-
"""Combined SO101 arm + mecanum drive panel.

One page for both devices: joint sliders, trajectory record/playback, and
the drive pad with the webcam. WASD and an Xbox-style controller on the
computer running the browser both send body-velocity commands. Arm speed
scales how fast the joints chase the sliders. Arm power writes the
STS3215 torque limit.

Do not run this at the same time as so101-arm/web_arm.py or
mecanum-drive/web_drive.py — they use the same serial ports.

  conda activate lerobot
  python combined-web/web_combined.py
  open http://<pi>:8003
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import serial
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_link import MecanumLink  # noqa: E402

ARM_PORT = '/dev/ttyACM1'
ARM_ID = 'so_follower'
HTTP_PORT = 8003

SERIAL_PORT = '/dev/ttyACM0'
BAUD = 115200
WATCHDOG_MS = 500
CAMERA_INDEX = 0
CONFIG_PATH = os.path.expanduser('~/.mecanum/motors.json')
TRAJ_DIR = os.path.expanduser('~/.so101_arm/trajs')
os.makedirs(TRAJ_DIR, exist_ok=True)

# Boot at full speed and full torque so a restart can hold the arm.
# Session-only on purpose: a saved low torque could let the arm sag on the next launch.
ARM_SPEED_DEFAULT = 100
ARM_POWER_DEFAULT = 100
ARM_SPEED_MIN = 5
ARM_POWER_MIN = 20
# Gripper Max_Torque_Limit is set to 500 in SOFollower.configure (burnout cap).
BODY_TORQUE_MAX = 1000
GRIPPER_TORQUE_MAX = 500

REST_POSE = {
    'shoulder_pan': 0, 'shoulder_lift': -66, 'elbow_flex': 98,
    'wrist_flex': 77, 'wrist_roll': 0, 'gripper': 1,
}
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

DEFAULT_MOTORS = [
    {'id': 1, 'name': 'Front left', 'invert': False, 'speed': 0},
    {'id': 2, 'name': 'Front right', 'invert': False, 'speed': 0},
    {'id': 3, 'name': 'Rear left', 'invert': False, 'speed': 0},
    {'id': 4, 'name': 'Rear right', 'invert': False, 'speed': 0},
]


def clamp_speed(value) -> int:
    return max(ARM_SPEED_MIN, min(100, int(value)))


def clamp_power(value) -> int:
    return max(ARM_POWER_MIN, min(100, int(value)))


def torque_map(power_pct: int) -> dict[str, int]:
    """Percent of each joint's allowed torque. Gripper stays inside the 500 cap."""
    power_pct = clamp_power(power_pct)
    body = int(round(BODY_TORQUE_MAX * power_pct / 100))
    grip = int(round(GRIPPER_TORQUE_MAX * power_pct / 100))
    return {j: grip if j == 'gripper' else body for j in JOINTS}


def safe_traj_name(name: str) -> str:
    name = str(name or '').strip()
    if not name or name in ('.', '..') or '/' in name or '\\' in name or '\x00' in name:
        return f'traj_{int(time.time())}'
    return name


def load_motors():
    if not os.path.exists(CONFIG_PATH):
        print(f'WARNING: config not found at {CONFIG_PATH}; using defaults')
        return [dict(m) for m in DEFAULT_MOTORS]
    with open(CONFIG_PATH) as f:
        data = json.load(f)
    raw = data.get('motors', data) if isinstance(data, dict) else data
    by_id = {}
    for m in raw:
        mid = int(m['id'])
        by_id[mid] = {
            'id': mid,
            'name': str(m.get('name') or f'Motor {mid}'),
            'invert': bool(m.get('invert', False)),
            'speed': 0,
        }
    motors = []
    for default in DEFAULT_MOTORS:
        motors.append(by_id.get(default['id'], dict(default)))
    print(f'Loaded config from {CONFIG_PATH}')
    for m in motors:
        print(f"  M{m['id']}  {m['name']!r}  invert={m['invert']}")
    return motors


def step_toward(current: dict, goal: dict, speed_pct: int) -> dict:
    """One 20 ms tick toward goal.

    100% matches so101-arm/web_arm.py: step = max(0.5, |error| * 0.15).
    Lower values scale that step. The step never passes the goal.
    """
    scale = clamp_speed(speed_pct) / 100.0
    out = {}
    for j in JOINTS:
        diff = goal[j] - current[j]
        if abs(diff) <= 0.1:
            out[j] = goal[j]
            continue
        step = min(abs(diff), max(0.5 * scale, abs(diff) * 0.15 * scale))
        out[j] = current[j] + (step if diff > 0 else -step)
    return out


def interp(frames, t):
    """Joint positions at time t. None once t is past the last frame."""
    if t <= frames[0]['t']:
        return dict(frames[0]['pos'])
    if t >= frames[-1]['t']:
        return None
    for a, b in zip(frames, frames[1:]):
        if a['t'] <= t < b['t']:
            k = (t - a['t']) / (b['t'] - a['t'])
            return {j: a['pos'][j] + (b['pos'][j] - a['pos'][j]) * k for j in JOINTS}
    return dict(frames[-1]['pos'])


def read_pose() -> dict[str, float]:
    obs = robot.get_observation()
    return {j: float(obs[f'{j}.pos']) for j in JOINTS}


def write_torque(power_pct: int) -> None:
    robot.bus.sync_write('Torque_Limit', torque_map(power_pct), normalize=False, num_retry=2)


robot = SO101Follower(SO101FollowerConfig(port=ARM_PORT, id=ARM_ID, num_read_retries=3))
try:
    robot.connect()
except Exception as e:
    print(f'Could not connect to the arm on {ARM_PORT}: {e}')
    print('Stop so101-arm/web_arm.py if it is running — only one process can use the arm.')
    try:
        robot.disconnect()
    except Exception:
        pass
    raise SystemExit(1) from e
print('Robot arm connected')

target = read_pose()
print('Initial pose:', {j: round(target[j], 1) for j in JOINTS})
lock = threading.Lock()
initial_pose = dict(target)
arm_tune = {'speed': ARM_SPEED_DEFAULT, 'power': ARM_POWER_DEFAULT}

mode = 'idle'  # idle | record | play
record_buf = []
rec_t0 = 0.0
rec_name = ''
play_frames = []
play_t0 = 0.0


motion_stop = threading.Event()


def motion_loop():
    global mode, target
    current = dict(initial_pose)
    applied_power = None
    last_torque_try = 0.0
    last_torque_log = 0.0
    while not motion_stop.is_set():
        with lock:
            speed = arm_tune['speed']
            power = arm_tune['power']
            run_mode = mode
            goal = dict(target)
            frames = play_frames
            t0 = play_t0
        now = time.time()
        if power != applied_power and now - last_torque_try >= 0.15:
            last_torque_try = now
            try:
                write_torque(power)
                applied_power = power
            except Exception as e:
                if now - last_torque_log >= 1.0:
                    print('torque write failed:', e)
                    last_torque_log = now
        if run_mode == 'play' and frames:
            pose = interp(frames, time.time() - t0)
            if pose is None:
                with lock:
                    if mode == 'play':
                        mode = 'idle'
                        target = dict(current)
            else:
                current = pose
        else:
            current = step_toward(current, goal, speed)
            if run_mode == 'record':
                try:
                    pos = read_pose()
                except Exception as e:
                    print('record read failed:', e)
                    pos = {j: round(current[j], 1) for j in JOINTS}
                sample = {
                    't': round(time.time() - rec_t0, 3),
                    'pos': {j: round(pos[j], 1) for j in JOINTS},
                }
                with lock:
                    if mode == 'record':
                        record_buf.append(sample)
        try:
            robot.send_action({f'{j}.pos': current[j] for j in JOINTS})
        except Exception as e:
            print('send failed:', e)
        time.sleep(0.02)


motion_thread = threading.Thread(target=motion_loop, daemon=True)
motion_thread.start()

motors = load_motors()
cmd = {'vx': 0, 'vy': 0, 'wz': 0, 'scale': 160}
frame_lock = threading.Lock()
latest_jpeg = None
camera_ok = False


def reinit_arduino(link: MecanumLink) -> None:
    flags = ' '.join('1' if m['invert'] else '0' for m in motors)
    link.send_line(f'I {flags}', retry=False)
    link.send_line(f'H {WATCHDOG_MS}', retry=False)
    if cmd['vx'] == 0 and cmd['vy'] == 0 and cmd['wz'] == 0:
        link.send_line('S', retry=False)
    else:
        link.send_line(f"V {cmd['vx']} {cmd['vy']} {cmd['wz']}", retry=False)


link = MecanumLink(baud=BAUD, preferred_port=SERIAL_PORT, on_reconnect=reinit_arduino)


def send_line(line: str) -> str:
    return link.send_line(line)


def drive_body(vx: int, vy: int, wz: int) -> str:
    vx = max(-255, min(255, int(vx)))
    vy = max(-255, min(255, int(vy)))
    wz = max(-255, min(255, int(wz)))
    cmd['vx'], cmd['vy'], cmd['wz'] = vx, vy, wz
    if vx == 0 and vy == 0 and wz == 0:
        return send_line('S')
    return send_line(f'V {vx} {vy} {wz}')


def keepalive_loop():
    while True:
        time.sleep(0.2)
        if cmd['vx'] == 0 and cmd['vy'] == 0 and cmd['wz'] == 0:
            continue
        try:
            send_line(f"V {cmd['vx']} {cmd['vy']} {cmd['wz']}")
        except serial.SerialException as e:
            print('keepalive failed:', e)


def camera_loop():
    global latest_jpeg, camera_ok
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'WARNING: could not open camera index {CAMERA_INDEX}')
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    camera_ok = True
    print(f'Camera open on index {CAMERA_INDEX} (/dev/video{CAMERA_INDEX})')
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue
            with frame_lock:
                latest_jpeg = buf.tobytes()
            time.sleep(0.03)
    finally:
        camera_ok = False
        cap.release()


link.start()
threading.Thread(target=keepalive_loop, daemon=True).start()
threading.Thread(target=camera_loop, daemon=True).start()
print(f'Arduino link on {link.port or SERIAL_PORT} @ {BAUD}')

HTML = '''<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SO101 + Mecanum</title>
<style>
:root{
  --bg:#1e1e2e;--surface:#313244;--text:#cdd6f4;--muted:#a6adc8;
  --green:#a6e3a1;--red:#f38ba8;--blue:#89b4fa;--peach:#fab387;
  --yellow:#f9e2af;--key:#45475a;
}
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--text);margin:0}
.wrap{max-width:1180px;margin:0 auto;padding:20px 16px 48px}
h1{font-size:1.4em;text-align:center;margin:0 0 4px}
.sub{text-align:center;color:var(--muted);font-size:.9em;margin:0 0 18px}
.layout{display:grid;grid-template-columns:minmax(300px,1fr) minmax(340px,1.15fr);gap:16px}
@media(max-width:900px){.layout{grid-template-columns:1fr}}
.panel{background:var(--surface);border-radius:12px;padding:14px 16px}
.panel h2{margin:0 0 4px;font-size:1.05em;color:var(--peach)}
.hint{color:var(--muted);font-size:.8em;margin:0 0 12px;line-height:1.4}
.cam{width:100%;aspect-ratio:4/3;background:#111;border-radius:8px;object-fit:contain;display:block}
.keys{display:grid;grid-template-columns:repeat(3,52px);gap:8px;justify-content:center;margin:12px 0}
.k{width:52px;height:52px;border-radius:10px;background:var(--key);display:flex;align-items:center;
  justify-content:center;font-weight:700;font-family:ui-monospace,monospace;user-select:none}
.k.on{background:var(--blue);color:#111}
.field{margin:10px 0}
.field label{display:block;font-size:.8em;color:var(--muted);margin-bottom:4px}
.row{display:flex;align-items:center;gap:8px}
.row input[type=range]{flex:1;min-width:0}
.val{font-family:ui-monospace,monospace;min-width:3.2em;text-align:right}
.joint{display:grid;grid-template-columns:8.6rem 1fr 3.4rem;gap:8px;align-items:center;margin:8px 0}
.joint label{font-size:.92em}
button{padding:8px 14px;border:none;border-radius:8px;cursor:pointer;font-weight:600;color:#111}
button.stop{background:var(--red)}
button.go{background:var(--green)}
button.rec{background:var(--yellow)}
button.play{background:var(--blue)}
button.wide{width:100%;padding:12px;margin-top:8px}
.actions{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin-top:8px}
select,input[type=text]{background:#1e1e2e;color:var(--text);border:none;border-radius:8px;padding:6px 8px}
select{width:100%;margin-top:8px}
.motors{font-size:.82em;color:var(--muted);line-height:1.45;margin-top:10px}
#armStatus,#driveStatus,#padStatus{font-family:ui-monospace,monospace;font-size:.8em;color:var(--muted);
  min-height:1.2em;margin-top:8px}
#padStatus{margin-top:0;margin-bottom:8px}
</style></head><body>
<div class="wrap">
  <h1>SO101 + Mecanum</h1>
  <p class="sub">Arm sliders and mecanum drive on one page. Run this instead of the separate arm and drive servers.</p>
  <div class="layout">
    <section class="panel" id="drive">
      <h2>Mecanum Drive</h2>
      <p class="hint">Left stick or WASD moves · right stick or Q/E rotates · B or Space stops. Plug the Xbox pad into the computer that has this page open, then press any button.</p>
      <div id="padStatus">Xbox pad: waiting for a button press.</div>
      <img class="cam" id="cam" alt="webcam" src="/stream">
      <div class="keys">
        <div></div><div class="k" id="k_q">Q</div><div class="k" id="k_w">W</div>
        <div class="k" id="k_a">A</div><div class="k" id="k_s">S</div><div class="k" id="k_d">D</div>
        <div></div><div class="k" id="k_e">E</div><div></div>
      </div>
      <div class="field">
        <label>Drive speed</label>
        <div class="row">
          <input type="range" id="driveScale" min="40" max="255" value="160">
          <span class="val" id="driveScaleVal">160</span>
        </div>
      </div>
      <button class="stop wide" onclick="emergencyStop()">Stop drive</button>
      <div class="motors" id="motors"></div>
      <div id="driveStatus"></div>
    </section>
    <section class="panel" id="arm">
      <h2>SO101 Robot Arm Control</h2>
      <p class="hint">Speed is how fast the joints chase the sliders (100% matches the arm-only panel). Power is servo torque. Low power can let the arm sag. Playback keeps the timing it was recorded with.</p>
      <div class="field">
        <label>Arm speed</label>
        <div class="row">
          <input type="range" id="armSpeed" min="5" max="100" value="100">
          <span class="val" id="armSpeedVal">100%</span>
        </div>
      </div>
      <div class="field">
        <label>Arm power</label>
        <div class="row">
          <input type="range" id="armPower" min="20" max="100" value="100">
          <span class="val" id="armPowerVal">100%</span>
        </div>
      </div>
      <div id="sliders"></div>
      <div class="actions">
        <button class="go" onclick="sendPreset()">Rest Pose</button>
        <button class="stop" onclick="zeroAll()">Zero All</button>
      </div>
      <div class="actions">
        <input type="text" id="trajname" placeholder="name" size="10" autocomplete="off">
        <button class="rec" id="recbtn" onclick="toggleRec()">Record</button>
        <button class="play" onclick="playSel()">Play</button>
      </div>
      <select id="trajlist" size="5"></select>
      <div id="armStatus"></div>
    </section>
  </div>
</div>
<script>
const joints=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'];
const ranges={'shoulder_pan':[-160,160],'shoulder_lift':[-100,100],'elbow_flex':[-100,100],
'wrist_flex':[-100,100],'wrist_roll':[-160,160],'gripper':[0,100]};
const restPose={'shoulder_pan':0,'shoulder_lift':-66,'elbow_flex':98,'wrist_flex':77,'wrist_roll':0,'gripper':1};
const s=document.getElementById('sliders');
joints.forEach(j=>{
 const[r0,r1]=ranges[j];
 const d=document.createElement('div');d.className='joint';
 d.innerHTML=`<label>${j}</label><input type="range" min="${r0}" max="${r1}" value="0" step="1" id="${j}"><span class="val" id="v_${j}">0</span>`;
 s.appendChild(d);
 d.querySelector('input').addEventListener('input',e=>{
  document.getElementById('v_'+j).textContent=e.target.value;
  fetch('/move',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({joint:j,value:parseFloat(e.target.value)})});
 });
});
function setArmStatus(t){document.getElementById('armStatus').textContent=t||'';}
function setDriveStatus(t){document.getElementById('driveStatus').textContent=t||'';}
function sendPreset(){joints.forEach(j=>{const el=document.getElementById(j);
 el.value=restPose[j];el.dispatchEvent(new Event('input'));});}
function zeroAll(){joints.forEach(j=>{const el=document.getElementById(j);
 el.value=0;el.dispatchEvent(new Event('input'));});}
async function syncPose(){
 const r=await fetch('/pose');const p=await r.json();
 joints.forEach(j=>{const el=document.getElementById(j);el.value=p[j];
 document.getElementById('v_'+j).textContent=Math.round(p[j]);});
}
async function refreshList(){
 const r=await fetch('/trajs');const list=await r.json();
 const sel=document.getElementById('trajlist');sel.innerHTML='';
 list.forEach(n=>{const o=document.createElement('option');o.value=n;o.textContent=n;sel.appendChild(o);});
}
function showTune(d){
 const sp=document.getElementById('armSpeed'), pw=document.getElementById('armPower');
 if(document.activeElement!==sp){sp.value=d.speed;document.getElementById('armSpeedVal').textContent=d.speed+'%';}
 if(document.activeElement!==pw){pw.value=d.power;document.getElementById('armPowerVal').textContent=d.power+'%';}
 if(d.mode==='record'){recording=true;document.getElementById('recbtn').textContent='Stop & Save';
  setArmStatus('Recording: '+(d.name||''));}
}
async function syncTune(){
 const r=await fetch('/tune');const d=await r.json();showTune(d);
}
document.getElementById('armSpeed').addEventListener('input',async e=>{
 document.getElementById('armSpeedVal').textContent=e.target.value+'%';
 try{
  const r=await fetch('/tune',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({speed:+e.target.value})});
  const d=await r.json();
  if(!d.ok) setArmStatus(d.error||'speed failed');
 }catch(err){setArmStatus('ERR '+err);}
});
document.getElementById('armPower').addEventListener('input',async e=>{
 document.getElementById('armPowerVal').textContent=e.target.value+'%';
 try{
  const r=await fetch('/tune',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({power:+e.target.value})});
  const d=await r.json();
  if(!d.ok) setArmStatus(d.error||'power failed');
 }catch(err){setArmStatus('ERR '+err);}
});
let recording=false;
async function toggleRec(){
 const btn=document.getElementById('recbtn');
 if(!recording){
  const name=document.getElementById('trajname').value||('traj_'+Date.now());
  const r=await fetch('/rec',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'start',name})});
  const d=await r.json();
  if(!d.ok){setArmStatus(d.error||'record failed');return;}
  recording=true;btn.textContent='Stop & Save';setArmStatus('Recording: '+d.name);
 }else{
  const r=await fetch('/rec',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({action:'stop'})});
  const d=await r.json();
  recording=false;btn.textContent='Record';
  if(!d.ok){setArmStatus(d.error||'save failed');return;}
  setArmStatus('Saved '+d.name+' ('+d.frames+' frames / '+d.seconds+'s)');refreshList();
 }
}
async function playSel(){
 const sel=document.getElementById('trajlist');
 if(!sel.value){setArmStatus('Select a trajectory first');return;}
 const r=await fetch('/play',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({name:sel.value})});
 const d=await r.json();
 if(!d.ok){setArmStatus(d.error||'play failed');return;}
 setArmStatus('Playing: '+sel.value);
}
const pressed=new Set();
const PAD_DZ=0.18;
let scale=160,busy=false,pending=null;
let padIndex=null, prevButtons=[], holdStop=false;
let lastSent={vx:0,vy:0,wz:0};
function syncKeys(){
 const cmd=lastSent;
 const on={
  w:pressed.has('w')||cmd.vx>0, s:pressed.has('s')||cmd.vx<0,
  a:pressed.has('a')||cmd.vy>0, d:pressed.has('d')||cmd.vy<0,
  q:pressed.has('q')||cmd.wz>0, e:pressed.has('e')||cmd.wz<0,
 };
 ['q','w','e','a','s','d'].forEach(k=>{
  const el=document.getElementById('k_'+k);
  if(el) el.classList.toggle('on',!!on[k]);
 });
}
function compute(){
 const s=scale;let vx=0,vy=0,wz=0;
 if(pressed.has('w')) vx+=s;
 if(pressed.has('s')) vx-=s;
 if(pressed.has('a')) vy+=s;
 if(pressed.has('d')) vy-=s;
 if(pressed.has('q')) wz+=s;
 if(pressed.has('e')) wz-=s;
 return {vx,vy,wz};
}
function dzAxis(v){
 const a=Math.abs(v);
 if(a<PAD_DZ) return 0;
 return Math.sign(v)*(a-PAD_DZ)/(1-PAD_DZ);
}
function padButton(pad,i){
 const b=pad&&pad.buttons&&pad.buttons[i];
 if(!b) return false;
 return typeof b==='object' ? !!(b.pressed||b.value>0.5) : !!b;
}
function mapPad(pad, maxSpeed){
 if(!pad) return null;
 const s=maxSpeed;
 let lx,ly,rx;
 if(pad.mapping==='standard'||pad.axes.length<6){
  lx=pad.axes[0]||0; ly=pad.axes[1]||0; rx=pad.axes[2]||0;
 }else{
  lx=pad.axes[0]||0; ly=pad.axes[1]||0; rx=pad.axes[3]||0;
 }
 let vx=Math.round(-dzAxis(ly)*s);
 let vy=Math.round(-dzAxis(lx)*s);
 let wz=Math.round(-dzAxis(rx)*s);
 if(padButton(pad,12)) vx=s;
 if(padButton(pad,13)) vx=-s;
 if(padButton(pad,14)) vy=s;
 if(padButton(pad,15)) vy=-s;
 const clamp=v=>Math.max(-255,Math.min(255,v));
 return {vx:clamp(vx),vy:clamp(vy),wz:clamp(wz)};
}
function currentPad(){
 if(!navigator.getGamepads) return null;
 const list=navigator.getGamepads();
 if(padIndex!=null && list[padIndex]) return list[padIndex];
 for(let i=0;i<list.length;i++) if(list[i]) return list[i];
 return null;
}
function desired(){
 if(holdStop) return {vx:0,vy:0,wz:0};
 const pad=mapPad(currentPad(), scale);
 if(pad && (pad.vx||pad.vy||pad.wz)) return pad;
 return compute();
}
async function sendDrive(body){
 if(busy){pending=body;return;}
 busy=true;
 try{
  const r=await fetch('/drive',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)});
  const d=await r.json();
  if(d.reply) setDriveStatus(d.reply);
  if(d.error) setDriveStatus('ERR '+d.error);
 }catch(e){setDriveStatus('ERR '+e);}
 finally{busy=false;if(pending){const p=pending;pending=null;sendDrive(p);}}
}
function publish(cmd){
 if(cmd.vx===lastSent.vx && cmd.vy===lastSent.vy && cmd.wz===lastSent.wz) return;
 lastSent={vx:cmd.vx,vy:cmd.vy,wz:cmd.wz};
 syncKeys();
 sendDrive(lastSent);
}
function pushState(){publish(desired());}
function emergencyStop(){
 pressed.clear();
 holdStop=true;
 publish({vx:0,vy:0,wz:0});
 syncKeys();
}
function nudgeScale(delta){
 scale=Math.max(40,Math.min(255,scale+delta));
 document.getElementById('driveScale').value=scale;
 document.getElementById('driveScaleVal').textContent=scale;
}
function notePad(pad){
 const el=document.getElementById('padStatus');
 if(!navigator.getGamepads){el.textContent='This browser has no gamepad API.';return;}
 if(pad){
  const m=mapPad(pad, scale);
  const name=String(pad.id||'controller').slice(0,48);
  el.textContent='Pad: '+name+(m&&(m.vx||m.vy||m.wz)?` · vx ${m.vx} vy ${m.vy} wz ${m.wz}`:' · centered');
  return;
 }
 if(!window.isSecureContext){
  el.textContent='Chrome hides gamepads on this address. On the computer with the pad: ssh -L 8003:127.0.0.1:8003 <pi> then open http://127.0.0.1:8003';
  return;
 }
 el.textContent='Xbox pad: plug it into this computer, then press any button.';
}
function pollPad(now){
 const pad=currentPad();
 const buttons=[];
 if(pad){
  for(let i=0;i<16;i++) buttons[i]=padButton(pad,i);
  if(buttons[1] && !prevButtons[1]) emergencyStop();
  if(buttons[4] && !prevButtons[4]) nudgeScale(-10);
  if(buttons[5] && !prevButtons[5]) nudgeScale(10);
 }
 prevButtons=buttons;
 const mapped=mapPad(pad, scale);
 const neutral=!mapped || (!mapped.vx && !mapped.vy && !mapped.wz);
 if(holdStop && neutral && pressed.size===0) holdStop=false;
 if(!pollPad.last || now-pollPad.last>=40){
  pollPad.last=now;
  publish(desired());
  notePad(pad);
 }
 requestAnimationFrame(pollPad);
}
window.addEventListener('gamepadconnected',e=>{padIndex=e.gamepad.index;});
window.addEventListener('gamepaddisconnected',e=>{
 if(padIndex===e.gamepad.index) padIndex=null;
 emergencyStop();
});
function isTyping(el){
 if(!el) return false;
 if(el.tagName==='TEXTAREA'||el.isContentEditable) return true;
 return el.tagName==='INPUT'&&el.type!=='range';
}
window.addEventListener('keydown',e=>{
 if(isTyping(e.target)) return;
 const k=e.key.toLowerCase();
 if(k===' '){e.preventDefault();emergencyStop();return;}
 if(!['w','a','s','d','q','e'].includes(k)) return;
 e.preventDefault();
 if(pressed.has(k)) return;
 holdStop=false;
 pressed.add(k);pushState();
});
window.addEventListener('keyup',e=>{
 const k=e.key.toLowerCase();
 if(!pressed.has(k)) return;
 pressed.delete(k);pushState();
});
window.addEventListener('blur',()=>emergencyStop());
document.addEventListener('visibilitychange',()=>{if(document.hidden) emergencyStop();});
document.getElementById('driveScale').addEventListener('input',e=>{
 scale=+e.target.value;
 document.getElementById('driveScaleVal').textContent=scale;
 publish(desired());
});
requestAnimationFrame(pollPad);
window.addEventListener('pagehide',()=>{
 navigator.sendBeacon('/drive',JSON.stringify({vx:0,vy:0,wz:0}));
});
function esc(s){return String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));}
async function bootDrive(){
 const r=await fetch('/status');const d=await r.json();
 if(d.scale){scale=d.scale;document.getElementById('driveScale').value=scale;
  document.getElementById('driveScaleVal').textContent=scale;}
 const box=document.getElementById('motors');
 box.innerHTML='<strong style="color:var(--text)">Calibration</strong><br>'+
  (d.motors||[]).map(m=>`M${m.id} ${esc(m.name)} — ${m.invert?'reversed':'normal'}`).join('<br>');
 if(d.camera===false) setDriveStatus('Camera not available');
 else if(!d.connected) setDriveStatus('Arduino disconnected — reconnecting…');
 else if(d.reply) setDriveStatus(d.reply);
}
setInterval(async()=>{
 try{
  const r=await fetch('/status');const d=await r.json();
  if(!d.connected) setDriveStatus('Arduino disconnected — reconnecting…');
 }catch(e){}
},2000);
syncPose();refreshList();syncTune();bootDrive();
</script></body></html>'''


def tune_state() -> dict:
    with lock:
        return {
            'ok': True,
            'speed': arm_tune['speed'],
            'power': arm_tune['power'],
            'mode': mode,
            'name': rec_name,
            'torque': torque_map(arm_tune['power']),
        }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        raw = self.rfile.read(int(self.headers.get('Content-Length') or 0))
        if not raw:
            return {}
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {}

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/trajs':
            files = sorted(f[:-5] for f in os.listdir(TRAJ_DIR) if f.endswith('.json'))
            self._json(files)
            return
        if path == '/pose':
            with lock:
                pose = {j: round(target[j], 1) for j in JOINTS}
            self._json(pose)
            return
        if path == '/tune':
            self._json(tune_state())
            return
        if path == '/status':
            self._json({
                'motors': motors,
                'config_path': CONFIG_PATH,
                'cmd': dict(cmd),
                'scale': cmd['scale'],
                'reply': link.last_reply,
                'camera': camera_ok,
                'connected': link.connected,
                'port': link.port,
                'reconnects': link.reconnect_count,
                'last_error': link.last_error,
                'arm': tune_state(),
            })
            return
        if path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        jpeg = latest_jpeg
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            return
        body = HTML.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        global mode, rec_t0, record_buf, rec_name, play_frames, play_t0
        path = self.path.split('?', 1)[0]
        data = self._read_json()
        try:
            if path == '/move':
                joint = data['joint']
                if joint not in JOINTS:
                    self._json({'ok': False, 'error': 'unknown joint'}, 400)
                    return
                with lock:
                    target[joint] = float(data['value'])
                self._json({'ok': True})
            elif path == '/tune':
                with lock:
                    if 'speed' in data:
                        arm_tune['speed'] = clamp_speed(data['speed'])
                    if 'power' in data:
                        arm_tune['power'] = clamp_power(data['power'])
                self._json(tune_state())
            elif path == '/rec':
                if data.get('action') == 'start':
                    with lock:
                        if mode == 'play':
                            blocked = 'stop playback first'
                        else:
                            blocked = ''
                            record_buf = []
                            rec_t0 = time.time()
                            rec_name = safe_traj_name(data.get('name'))
                            mode = 'record'
                            name = rec_name
                    if blocked:
                        self._json({'ok': False, 'error': blocked}, 409)
                    else:
                        self._json({'ok': True, 'name': name})
                else:
                    with lock:
                        mode = 'idle'
                        frames = list(record_buf)
                        name = safe_traj_name(data.get('name') or rec_name)
                        record_buf = []
                    if frames:
                        with open(os.path.join(TRAJ_DIR, name + '.json'), 'w') as f:
                            json.dump(frames, f)
                        secs = frames[-1]['t']
                        count = len(frames)
                        print(f'Saved {name}: {count} frames / {secs:.1f}s')
                    else:
                        secs, count = 0.0, 0
                        print('Nothing recorded (stopped too early)')
                    self._json({
                        'ok': True, 'name': name,
                        'frames': count, 'seconds': round(secs, 1),
                    })
            elif path == '/play':
                name = safe_traj_name(data.get('name'))
                path_on_disk = os.path.join(TRAJ_DIR, name + '.json')
                if not os.path.exists(path_on_disk):
                    self._json({'ok': False, 'error': 'trajectory not found'}, 404)
                    return
                with open(path_on_disk) as f:
                    raw = json.load(f)
                if not raw:
                    self._json({'ok': False, 'error': 'trajectory is empty'}, 400)
                    return
                with lock:
                    if mode == 'record':
                        blocked = 'stop recording first'
                    else:
                        blocked = ''
                        cur = {j: target[j] for j in JOINTS}
                        t_off = raw[0]['t'] + 1.0
                        play_frames = [{'t': 0.0, 'pos': cur}] + [
                            {'t': fr['t'] + t_off, 'pos': fr['pos']} for fr in raw
                        ]
                        play_t0 = time.time()
                        mode = 'play'
                if blocked:
                    self._json({'ok': False, 'error': blocked}, 409)
                else:
                    self._json({'ok': True})
            elif path == '/drive':
                if 'scale' in data:
                    cmd['scale'] = max(40, min(255, int(data['scale'])))
                reply = drive_body(
                    int(data.get('vx', 0)),
                    int(data.get('vy', 0)),
                    int(data.get('wz', 0)),
                )
                self._json({'ok': True, 'reply': reply, 'cmd': dict(cmd)})
            else:
                self._json({'ok': False, 'error': 'not found'}, 404)
        except (KeyError, TypeError, ValueError) as e:
            self._json({'ok': False, 'error': str(e)}, 400)
        except serial.SerialException as e:
            self._json({'ok': False, 'error': f'serial: {e}'}, 500)
        except OSError as e:
            self._json({'ok': False, 'error': str(e)}, 500)


def _urls_for_port(port: int) -> list[str]:
    urls = [f'http://127.0.0.1:{port}']
    seen = {'127.0.0.1'}

    def add(ip: str) -> None:
        if not ip or ip in seen or ip.startswith('127.') or ':' in ip:
            return
        seen.add(ip)
        urls.append(f'http://{ip}:{port}')

    try:
        import socket
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            add(info[4][0])
    except OSError:
        pass
    # This Pi's hostname often resolves only to localhost.
    try:
        import subprocess
        out = subprocess.check_output(['hostname', '-I'], text=True, timeout=2)
        for ip in out.split():
            add(ip)
    except (OSError, subprocess.SubprocessError):
        pass
    return urls


class ReuseHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


server = None
try:
    server = ReuseHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print('Combined arm + drive panel (keep this process running):')
    for url in _urls_for_port(HTTP_PORT):
        print(f'  {url}')
    server.serve_forever()
except KeyboardInterrupt:
    pass
except OSError as e:
    if getattr(e, 'errno', None) == 98:
        print(f'Port {HTTP_PORT} is already in use. Stop the other process, then start again.')
    else:
        raise
finally:
    motion_stop.set()
    motion_thread.join(timeout=2.0)
    try:
        send_line('S')
        send_line('H 0')
    except Exception:
        pass
    try:
        write_torque(100)
    except Exception:
        pass
    if server is not None:
        server.server_close()
    link.close()
    robot.disconnect()
    print('Disconnected')
