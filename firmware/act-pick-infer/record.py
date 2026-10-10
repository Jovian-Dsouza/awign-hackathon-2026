# -*- coding: utf-8 -*-
"""Record demos for fine-tuning the wrist ACT policy.

Two control modes (toggle on the page):

  Kinesthetic — arm torque off; move joints by hand. Gripper via slider or S/D.
  Joystick    — gamepad on the *client* machine (e.g. MacBook browser) drives
                all six joints over the network. Uses the browser Gamepad API;
                nothing is plugged into the Pi.

The LG Smart Cam on /dev/video0 is stored as observation.images.wrist at
640x480 @ 30 fps — same layout as TakuyaHiraoka/so101_pick_diverse_objects.

Each frame saves the measured joint pose as both observation.state and
action, so ACT learns the trajectory you demonstrated.

    conda activate lerobot
    python act-pick-infer/record.py \\
      --repo-id YOUR_HF_USER/so101_pick_wrist_finetune \\
      --task "Pick up the object and lift it"

Open http://127.0.0.1:8005 (or the Pi LAN URL) from the MacBook. Record,
demonstrate, Stop and save, then Push. Use --resume to append later.
"""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.opencv.configuration_opencv import Cv2Backends
from lerobot.configs.video import RGBEncoderConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features

ARM_PORT = "/dev/ttyACM1"
ARM_ID = "so_follower"
CAMERA_PATH = "/dev/video0"
CAMERA_W = 640
CAMERA_H = 480
FPS = 30
VIEW_PORT = 8005

JOINTS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]
ARM_MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
JOINT_LIMITS = {
    "shoulder_pan.pos": (-160.0, 160.0),
    "shoulder_lift.pos": (-100.0, 100.0),
    "elbow_flex.pos": (-100.0, 100.0),
    "wrist_flex.pos": (-100.0, 100.0),
    "wrist_roll.pos": (-160.0, 160.0),
    "gripper.pos": (0.0, 100.0),
}


def _urls(port: int) -> list[str]:
    urls = [f"http://127.0.0.1:{port}"]
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    seen: set[str] = set()
    out: list[str] = []
    for url in urls:
        if url not in seen:
            seen.add(url)
            out.append(url)
    return out


def _retry_bus(robot: SO101Follower, retries: int = 5) -> None:
    """Feetech status packets drop occasionally on this USB bus. Retry writes."""
    bus = robot.bus
    orig_write = bus.write
    orig_enable = bus.enable_torque
    orig_disable = bus.disable_torque

    def write(data_name, motor, value, *, normalize=True, num_retry=0):
        return orig_write(
            data_name, motor, value, normalize=normalize, num_retry=max(num_retry, retries)
        )

    def enable_torque(motors=None, num_retry=0):
        return orig_enable(motors, num_retry=max(num_retry, retries))

    def disable_torque(motors=None, num_retry=0):
        return orig_disable(motors, num_retry=max(num_retry, retries))

    bus.write = write
    bus.enable_torque = enable_torque
    bus.disable_torque = disable_torque


def make_robot() -> SO101Follower:
    cameras = {
        "wrist": OpenCVCameraConfig(
            index_or_path=CAMERA_PATH,
            fps=FPS,
            width=CAMERA_W,
            height=CAMERA_H,
            fourcc="MJPG",
            backend=Cv2Backends.V4L2,
        ),
    }
    robot = SO101Follower(
        SO101FollowerConfig(
            port=ARM_PORT,
            id=ARM_ID,
            cameras=cameras,
            num_read_retries=3,
            max_relative_target=None,
        )
    )
    _retry_bus(robot)
    robot.connect(calibrate=False)
    if robot.calibration and not robot.is_calibrated:
        print(f"Writing saved calibration {robot.calibration_fpath}")
        robot.bus.write_calibration(robot.calibration)
    # Kinesthetic demo: free the arm, keep gripper powered for the slider.
    robot.bus.disable_torque(ARM_MOTORS)
    robot.bus.enable_torque("gripper")
    return robot


def make_features() -> dict:
    obs_hw = {j: float for j in JOINTS}
    obs_hw["wrist"] = (CAMERA_H, CAMERA_W, 3)
    action_hw = {j: float for j in JOINTS}
    features = {}
    features.update(hw_to_dataset_features(obs_hw, OBS_STR, use_video=True))
    features.update(hw_to_dataset_features(action_hw, ACTION, use_video=True))
    return features


def open_dataset(repo_id: str, resume: bool) -> LeRobotDataset:
    rgb = RGBEncoderConfig(vcodec="h264")
    kwargs = dict(
        batch_encoding_size=1,
        rgb_encoder=rgb,
        image_writer_processes=0,
        image_writer_threads=4,
        streaming_encoding=True,
        encoder_queue_maxsize=30,
        encoder_threads=2,
    )
    if resume:
        print(f"Resuming dataset {repo_id}")
        return LeRobotDataset.resume(repo_id, **kwargs)
    print(f"Creating dataset {repo_id}")
    return LeRobotDataset.create(
        repo_id=repo_id,
        fps=FPS,
        features=make_features(),
        robot_type="so_follower",
        use_videos=True,
        **kwargs,
    )


# Shared UI / control state
lock = threading.Lock()
state = {
    "jpeg": None,
    "pose": {},
    "gripper": 20.0,
    "grip_speed": 40.0,  # units/sec while holding S/D in the browser
    "arm_speed": 45.0,  # deg/sec for joystick stick axes
    "control": "kinesthetic",  # kinesthetic | joystick
    "gamepad": None,  # last reported pad id from the client
    "target": {},  # joint targets while in joystick mode
    "mode": "idle",  # idle | recording | pushed
    "frames": 0,
    "episodes": 0,
    "status": "ready",
    "task": "",
    "repo_id": "",
}
stop_event = threading.Event()
commands: list[dict] = []


HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ACT record — wrist demos</title>
<style>
body{font-family:sans-serif;background:#1e1e2e;color:#cdd6f4;margin:0}
main{display:flex;gap:24px;align-items:flex-start;padding:20px;flex-wrap:wrap}
img{width:640px;max-width:100%;background:#111;border-radius:8px}
h1{font-size:1.15em;margin:0 0 8px}
#status{font-family:monospace;margin:10px 0 16px;min-height:1.4em}
.row{margin:12px 0}
label{display:inline-block;width:100px}
input[type=range]{width:240px;vertical-align:middle}
.val{display:inline-block;width:48px;text-align:right;font-family:monospace}
button{padding:8px 16px;margin:4px 4px 4px 0;border:none;border-radius:6px;cursor:pointer;font-size:14px}
.rec{background:#f38ba8;color:#1e1e2e}
.stop{background:#a6e3a1;color:#1e1e2e}
.disc{background:#f9e2af;color:#1e1e2e}
.push{background:#89b4fa;color:#1e1e2e}
.modebtn{background:#45475a;color:#cdd6f4}
.modebtn.on{background:#cba6f7;color:#1e1e2e}
.dim{color:#a6adc8;font-size:13px;max-width:380px;line-height:1.4}
#pad{font-family:monospace;font-size:12px;color:#a6adc8;margin:6px 0 10px;min-height:1.2em}
table{border-collapse:collapse;font-family:monospace;font-size:13px;margin-top:12px}
td{padding:3px 12px 3px 0}
td.n{text-align:right;min-width:56px}
kbd{background:#313244;padding:1px 6px;border-radius:4px;font-size:12px}
</style></head><body>
<main>
<img id="cam" alt="wrist camera" src="/stream">
<div>
<h1>ACT record — demos</h1>
<div id="status">connecting…</div>
<div id="pad">gamepad: waiting (press any button on the MacBook pad)</div>
<p>
<button class="modebtn" id="btnKino" onclick="setControl('kinesthetic')">Kinesthetic</button>
<button class="modebtn" id="btnJoy" onclick="setControl('joystick')">Joystick teleop</button>
</p>
<div class="row">
 <label>gripper</label>
 <input type="range" id="grip" min="0" max="100" value="20" step="1">
 <span class="val" id="gval">20</span>
</div>
<div class="row">
 <label>grip speed</label>
 <input type="range" id="gspeed" min="5" max="120" value="40" step="1">
 <span class="val" id="sval">40</span>
 <span class="dim"> /s</span>
</div>
<div class="row">
 <label>arm speed</label>
 <input type="range" id="aspeed" min="10" max="120" value="45" step="1">
 <span class="val" id="aval">45</span>
 <span class="dim"> °/s</span>
</div>
<p class="dim"><kbd>S</kbd> close gripper &nbsp;·&nbsp; <kbd>D</kbd> open
&nbsp;·&nbsp; pad: L stick pan/lift, R stick elbow/wrist_flex, LB/RB roll, LT/RT gripper</p>
<p>
<button class="rec" id="recbtn" onclick="startRec()">Record</button>
<button class="stop" onclick="stopRec()">Stop and save</button>
<button class="disc" onclick="discard()">Discard</button>
<button class="push" onclick="pushHub()">Push to Hub</button>
</p>
<table>
<tr><td class="dim">joint</td><td class="n dim">pose</td></tr>
<tbody id="rows"></tbody>
</table>
<p class="dim">Open this page from the MacBook. The joystick stays on the MacBook;
the browser Gamepad API sends commands to the Pi. Kinesthetic frees the arm for hand guiding.</p>
</div>
</main>
<script>
const joints=__JOINTS__;
const limits=__LIMITS__;
const dead=0.12;
function row(j, pose){
  const name=j.replace('.pos','');
  const p=pose[j]==null?'—':Number(pose[j]).toFixed(1);
  return `<tr><td>${name}</td><td class="n">${p}</td></tr>`;
}
function clamp(j, v){
  const [lo,hi]=limits[j]||[-180,180];
  return Math.max(lo, Math.min(hi, v));
}
const grip=document.getElementById('grip');
const gspeed=document.getElementById('gspeed');
const aspeed=document.getElementById('aspeed');
let gripLocal=parseFloat(grip.value);
let keys={s:false,d:false};
let control='kinesthetic';
let target=null;  // absolute joint targets in joystick mode
let lastPose={};
let lastT=performance.now();
let lastTeleop=0;
let padName='';

function postJSON(path, body){
  return fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body)});
}
function postGrip(v){
  gripLocal=clamp('gripper.pos', v);
  grip.value=Math.round(gripLocal);
  document.getElementById('gval').textContent=Math.round(gripLocal);
  if(control==='joystick' && target){
    target['gripper.pos']=gripLocal;
  }
  postJSON('/gripper',{value:gripLocal});
}
grip.addEventListener('input',()=>{ postGrip(parseFloat(grip.value)); });
gspeed.addEventListener('input',()=>{
  document.getElementById('sval').textContent=gspeed.value;
  postJSON('/grip-speed',{value:parseFloat(gspeed.value)});
});
aspeed.addEventListener('input',()=>{
  document.getElementById('aval').textContent=aspeed.value;
  postJSON('/arm-speed',{value:parseFloat(aspeed.value)});
});

async function setControl(mode){
  control=mode;
  document.getElementById('btnKino').classList.toggle('on', mode==='kinesthetic');
  document.getElementById('btnJoy').classList.toggle('on', mode==='joystick');
  if(mode==='joystick'){
    target={};
    for(const j of joints) target[j]=lastPose[j]!=null?lastPose[j]:0;
    target['gripper.pos']=gripLocal;
  }else{
    target=null;
  }
  await postJSON('/control',{mode});
}

function onKey(e, down){
  if(e.repeat) return;
  const k=e.key.toLowerCase();
  if(k==='s' || k==='d'){
    e.preventDefault();
    keys[k]=down;
  }
}
window.addEventListener('keydown', e=>onKey(e,true));
window.addEventListener('keyup', e=>onKey(e,false));
window.addEventListener('blur', ()=>{ keys.s=false; keys.d=false; });

window.addEventListener('gamepadconnected', e=>{
  padName=e.gamepad.id;
  document.getElementById('pad').textContent='gamepad: '+padName;
});
window.addEventListener('gamepaddisconnected', ()=>{
  padName='';
  document.getElementById('pad').textContent='gamepad: disconnected';
});

function axis(gp, i){
  const v=gp.axes[i]||0;
  return Math.abs(v)<dead?0:v;
}
function btn(gp, i){
  const b=gp.buttons[i];
  if(!b) return 0;
  return typeof b.value==='number'?b.value:(b.pressed?1:0);
}
function pickPad(){
  const pads=navigator.getGamepads?navigator.getGamepads():[];
  for(const p of pads){ if(p) return p; }
  return null;
}

function controlLoop(now){
  const dt=Math.min(0.05, (now-lastT)/1000);
  lastT=now;
  const gSpd=parseFloat(gspeed.value)||40;
  const aSpd=parseFloat(aspeed.value)||45;

  // Keyboard gripper (both modes)
  let gDelta=0;
  if(keys.s) gDelta-=gSpd*dt;
  if(keys.d) gDelta+=gSpd*dt;
  if(gDelta!==0) postGrip(gripLocal+gDelta);

  const gp=pickPad();
  if(gp && !padName){
    padName=gp.id;
    document.getElementById('pad').textContent='gamepad: '+padName;
  }

  if(control==='joystick' && target){
    if(gp){
      // Standard mapping (Xbox / dual-stick):
      // Lx pan, Ly lift, Rx wrist_flex, Ry elbow, LB/RB roll, LT/RT gripper
      const lx=axis(gp,0), ly=axis(gp,1);
      const rx=axis(gp,2), ry=axis(gp,3);
      target['shoulder_pan.pos']=clamp('shoulder_pan.pos', target['shoulder_pan.pos']+lx*aSpd*dt);
      target['shoulder_lift.pos']=clamp('shoulder_lift.pos', target['shoulder_lift.pos']+(-ly)*aSpd*dt);
      target['elbow_flex.pos']=clamp('elbow_flex.pos', target['elbow_flex.pos']+(-ry)*aSpd*dt);
      target['wrist_flex.pos']=clamp('wrist_flex.pos', target['wrist_flex.pos']+rx*aSpd*dt);
      const roll=(btn(gp,5)-btn(gp,4)); // RB open direction +, LB -
      if(roll!==0) target['wrist_roll.pos']=clamp('wrist_roll.pos', target['wrist_roll.pos']+roll*aSpd*dt);
      const lt=btn(gp,6), rt=btn(gp,7);
      if(lt>0.05 || rt>0.05){
        postGrip(gripLocal + (rt-lt)*gSpd*dt);
        target['gripper.pos']=gripLocal;
      }
    }
    // ~30 Hz to the Pi (gamepad stays on the MacBook)
    if(now-lastTeleop>33){
      lastTeleop=now;
      postJSON('/teleop',{joints:target, gamepad:gp?gp.id:null});
    }
  }
  requestAnimationFrame(controlLoop);
}
requestAnimationFrame(controlLoop);

async function startRec(){
  await postJSON('/cmd',{action:'start'});
}
async function stopRec(){
  await postJSON('/cmd',{action:'stop'});
}
async function discard(){
  await postJSON('/cmd',{action:'discard'});
}
async function pushHub(){
  if(!confirm('Finalize and upload this private dataset to the Hub?')) return;
  await postJSON('/cmd',{action:'push'});
}
async function tick(){
  try{
    const r=await fetch('/state');
    const d=await r.json();
    lastPose=d.pose||{};
    control=d.control||control;
    document.getElementById('btnKino').classList.toggle('on', control==='kinesthetic');
    document.getElementById('btnJoy').classList.toggle('on', control==='joystick');
    document.getElementById('status').textContent=
      d.status+'  ·  '+d.control+'  ·  rec='+d.mode
      +'  ·  frames='+d.frames+'  ·  episodes='+d.episodes;
    if(d.mode==='recording') document.getElementById('recbtn').textContent='Recording…';
    else document.getElementById('recbtn').textContent='Record';
    const holding=keys.s||keys.d;
    if(control==='kinesthetic' && !holding && document.activeElement!==grip){
      gripLocal=d.gripper;
      grip.value=Math.round(d.gripper);
      document.getElementById('gval').textContent=Math.round(d.gripper);
    }
    if(document.activeElement!==gspeed){
      gspeed.value=Math.round(d.grip_speed);
      document.getElementById('sval').textContent=Math.round(d.grip_speed);
    }
    if(document.activeElement!==aspeed){
      aspeed.value=Math.round(d.arm_speed);
      document.getElementById('aval').textContent=Math.round(d.arm_speed);
    }
    document.getElementById('rows').innerHTML=
      joints.map(j=>row(j,d.pose)).join('');
  }catch(e){}
}
tick();
setInterval(tick, 200);
setControl('kinesthetic');
</script>
</body></html>""".replace("__JOINTS__", json.dumps(JOINTS)).replace(
    "__LIMITS__", json.dumps(JOINT_LIMITS)
)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/state":
            with lock:
                body = {
                    "pose": state["pose"],
                    "gripper": state["gripper"],
                    "grip_speed": state["grip_speed"],
                    "arm_speed": state["arm_speed"],
                    "control": state["control"],
                    "gamepad": state["gamepad"],
                    "mode": state["mode"],
                    "frames": state["frames"],
                    "episodes": state["episodes"],
                    "status": state["status"],
                    "task": state["task"],
                    "repo_id": state["repo_id"],
                }
            self._json(body)
            return
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while not stop_event.is_set():
                    with lock:
                        jpeg = state["jpeg"]
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
            return
        page = HTML.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(page)

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        raw = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        data = json.loads(raw) if raw else {}
        if path == "/gripper":
            with lock:
                state["gripper"] = float(np.clip(data.get("value", state["gripper"]), 0, 100))
                if state["target"]:
                    state["target"]["gripper.pos"] = state["gripper"]
            self._json({"ok": True})
            return
        if path == "/grip-speed":
            with lock:
                state["grip_speed"] = float(np.clip(data.get("value", state["grip_speed"]), 5, 120))
            self._json({"ok": True})
            return
        if path == "/arm-speed":
            with lock:
                state["arm_speed"] = float(np.clip(data.get("value", state["arm_speed"]), 10, 120))
            self._json({"ok": True})
            return
        if path == "/control":
            mode = data.get("mode")
            if mode not in ("kinesthetic", "joystick"):
                self._json({"ok": False, "error": "bad mode"}, 400)
                return
            with lock:
                commands.append({"action": "set_control", "mode": mode})
            self._json({"ok": True})
            return
        if path == "/teleop":
            joints = data.get("joints") or {}
            with lock:
                if state["control"] != "joystick":
                    self._json({"ok": False, "error": "not in joystick mode"}, 409)
                    return
                target = dict(state["target"] or {})
                for j in JOINTS:
                    if j in joints:
                        lo, hi = JOINT_LIMITS[j]
                        target[j] = float(np.clip(joints[j], lo, hi))
                state["target"] = target
                if "gripper.pos" in target:
                    state["gripper"] = target["gripper.pos"]
                state["gamepad"] = data.get("gamepad")
            self._json({"ok": True})
            return
        if path == "/cmd":
            action = data.get("action")
            with lock:
                commands.append({"action": action})
            self._json({"ok": True})
            return
        self.send_error(404)


class ViewServer(ThreadingHTTPServer):
    allow_reuse_address = True


def publish_preview(obs):
    bgr = cv2.cvtColor(obs["wrist"], cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    pose = {j: round(float(obs[j]), 1) for j in JOINTS}
    with lock:
        if ok:
            state["jpeg"] = buf.tobytes()
        state["pose"] = pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-id",
        required=True,
        help="Hugging Face dataset id, e.g. YOUR_HF_USER/so101_pick_wrist_finetune",
    )
    parser.add_argument(
        "--task",
        default="Pick up the object and lift it",
        help="Task string stored on every frame",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Append episodes to an existing local/Hub dataset with this repo-id",
    )
    args = parser.parse_args()

    if "/" not in args.repo_id:
        raise SystemExit("--repo-id must look like USERNAME/DATASET_NAME")

    with lock:
        state["task"] = args.task
        state["repo_id"] = args.repo_id

    dataset = open_dataset(args.repo_id, args.resume)
    with lock:
        state["episodes"] = dataset.num_episodes

    robot = make_robot()
    print("Arm + LG Smart Cam connected (kinesthetic: arm torque off, gripper on)")
    print("  port", ARM_PORT, "camera", CAMERA_PATH, f"{CAMERA_W}x{CAMERA_H}@{FPS} MJPG")
    print("  dataset", args.repo_id, "task:", args.task)
    print("  Open the page from the MacBook; joystick teleop uses the browser Gamepad API")

    try:
        server = ViewServer(("0.0.0.0", VIEW_PORT), Handler)
    except OSError as e:
        if getattr(e, "errno", None) == 98:
            print(f"Port {VIEW_PORT} is already in use. Stop the other process, then start again.")
        raise
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("Record panel (keep this process running):")
    for url in _urls(VIEW_PORT):
        print(f"  {url}")

    features = dataset.features
    finalized = False

    try:
        # Seed gripper target from the current pose so the jaw does not jump.
        obs0 = robot.get_observation()
        with lock:
            state["gripper"] = float(obs0["gripper.pos"])
            state["target"] = {j: float(obs0[j]) for j in JOINTS}
        publish_preview(obs0)

        while not stop_event.is_set():
            loop_t = time.perf_counter()

            # Drain UI commands on the control thread (dataset / bus are not thread-safe).
            with lock:
                pending = list(commands)
                commands.clear()
                mode = state["mode"]
                control = state["control"]
                gripper = state["gripper"]
                target = dict(state["target"]) if state["target"] else {}

            for cmd in pending:
                action = cmd.get("action")
                if action == "set_control":
                    new_control = cmd.get("mode")
                    if new_control == "joystick":
                        obs = robot.get_observation()
                        seeded = {j: float(obs[j]) for j in JOINTS}
                        seeded["gripper.pos"] = float(gripper)
                        try:
                            robot.bus.enable_torque()
                        except Exception as e:
                            print("enable_torque failed:", e)
                        with lock:
                            state["control"] = "joystick"
                            state["target"] = seeded
                            state["gripper"] = seeded["gripper.pos"]
                            state["status"] = "joystick teleop — use the MacBook gamepad"
                        control = "joystick"
                        target = seeded
                        print("Control: joystick (torque on)")
                    else:
                        try:
                            robot.bus.disable_torque(ARM_MOTORS)
                            robot.bus.enable_torque("gripper")
                        except Exception as e:
                            print("kinesthetic torque switch failed:", e)
                        with lock:
                            state["control"] = "kinesthetic"
                            state["status"] = "kinesthetic — move arm by hand; gripper S/D or slider"
                        control = "kinesthetic"
                        print("Control: kinesthetic (arm torque off)")
                elif action == "start":
                    if mode == "pushed":
                        with lock:
                            state["status"] = "already pushed — restart with --resume to add more"
                        continue
                    if mode == "recording":
                        continue
                    with lock:
                        state["mode"] = "recording"
                        state["frames"] = 0
                        state["status"] = (
                            "recording — joystick teleop"
                            if control == "joystick"
                            else "recording — move the arm by hand"
                        )
                    mode = "recording"
                    print("Recording episode…")
                elif action == "stop":
                    if mode != "recording":
                        with lock:
                            state["status"] = "not recording"
                        continue
                    if not dataset.has_pending_frames():
                        with lock:
                            state["mode"] = "idle"
                            state["status"] = "nothing to save (no frames)"
                        mode = "idle"
                        continue
                    dataset.save_episode()
                    with lock:
                        state["mode"] = "idle"
                        state["frames"] = 0
                        state["episodes"] = dataset.num_episodes
                        state["status"] = f"saved episode — total {dataset.num_episodes}"
                    mode = "idle"
                    print(f"Saved episode. Total episodes: {dataset.num_episodes}")
                elif action == "discard":
                    if mode != "recording" and not dataset.has_pending_frames():
                        with lock:
                            state["status"] = "nothing to discard"
                        continue
                    dataset.clear_episode_buffer()
                    with lock:
                        state["mode"] = "idle"
                        state["frames"] = 0
                        state["status"] = "discarded episode buffer"
                    mode = "idle"
                    print("Discarded episode buffer")
                elif action == "push":
                    if mode == "recording":
                        with lock:
                            state["status"] = "stop recording before push"
                        continue
                    if dataset.has_pending_frames():
                        with lock:
                            state["status"] = "unsaved frames — Stop and save or Discard first"
                        continue
                    with lock:
                        state["status"] = "finalizing and uploading…"
                    print("Finalizing and pushing to Hub (private)…")
                    dataset.finalize()
                    finalized = True
                    dataset.push_to_hub(private=True, tags=["so101", "act", "wrist", "finetune"])
                    with lock:
                        state["mode"] = "pushed"
                        state["status"] = f"pushed private dataset {args.repo_id}"
                    mode = "pushed"
                    print(f"Pushed https://huggingface.co/datasets/{args.repo_id}")

            if mode == "pushed":
                time.sleep(0.1)
                continue

            obs = robot.get_observation()
            try:
                if control == "joystick" and target:
                    robot.send_action(target)
                else:
                    # Gripper only — arm joints stay free for hand guidance.
                    robot.send_action({"gripper.pos": gripper})
            except Exception as e:
                print("send_action failed:", e)

            publish_preview(obs)

            if mode == "recording":
                obs_frame = build_dataset_frame(features, obs, prefix=OBS_STR)
                # action = measured pose (what the arm actually did)
                action_vals = {j: float(obs[j]) for j in JOINTS}
                action_frame = build_dataset_frame(features, action_vals, prefix=ACTION)
                dataset.add_frame({**obs_frame, **action_frame, "task": args.task})
                with lock:
                    state["frames"] += 1

            remain = (1.0 / FPS) - (time.perf_counter() - loop_t)
            if remain > 0:
                time.sleep(remain)
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        stop_event.set()
        server.shutdown()
        if dataset.has_pending_frames():
            print("Discarding unsaved episode buffer")
            dataset.clear_episode_buffer()
        if not finalized:
            print("Finalizing local dataset (not pushed — use the Push button next time, or --resume then Push)")
            try:
                dataset.finalize()
            except Exception as e:
                print("finalize:", e)
        robot.disconnect()
        print("Disconnected")


if __name__ == "__main__":
    main()
