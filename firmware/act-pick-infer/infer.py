# -*- coding: utf-8 -*-
"""Run TakuyaHiraoka/act_so101_pick_diverse_objects on this Pi.

The policy is ACT (wrist camera + 6 joint positions -> 6 joint targets),
trained at 640x480 @ 30 fps. This machine has one USB camera:

    LG Smart Cam  /dev/video0  (uvcvideo, MJPG or YUYV, 640x480 @ 30)

That device is the wrist observation. The arm is the same SO101 follower
used by so101-arm/web_arm.py: /dev/ttyACM1, id so_follower.

This Pi has no CUDA, so weights and the preprocessor run on CPU. ACT
predicts 100 actions per forward pass and then plays them at 30 Hz.
The first pass is slow; later ticks only pop the queue.

    conda activate lerobot
    python act-pick-infer/infer.py --check     # live view, no motion
    python act-pick-infer/infer.py             # drive the arm until Ctrl-C
    python act-pick-infer/infer.py --seconds 20
    python act-pick-infer/infer.py --policy YOUR_HF_USER/act_so101_pick_finetune

While it runs, open the printed URL (port 8004). The page shows the wrist
frame the policy is using, the measured pose, and the joint command.
"""
import argparse
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import torch

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.opencv.configuration_opencv import Cv2Backends
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import (
    make_robot_action,
    prepare_observation_for_inference,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.feature_utils import build_dataset_frame

POLICY_ID = "TakuyaHiraoka/act_so101_pick_diverse_objects"
ARM_PORT = "/dev/ttyACM1"
ARM_ID = "so_follower"
# Only USB capture device on this Pi (see v4l2-ctl --list-devices).
CAMERA_PATH = "/dev/video0"
CAMERA_W = 640
CAMERA_H = 480
FPS = 30
VIEW_PORT = 8004

JOINTS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

# Names and shapes match the training dataset
# (TakuyaHiraoka/so101_pick_diverse_objects, meta/info.json).
DATASET_FEATURES = {
    "observation.state": {"dtype": "float32", "shape": [6], "names": JOINTS},
    "observation.images.wrist": {
        "dtype": "video",
        "shape": [CAMERA_H, CAMERA_W, 3],
        "names": ["height", "width", "channels"],
    },
    "action": {"dtype": "float32", "shape": [6], "names": JOINTS},
}


def load_policy(policy_id: str = POLICY_ID):
    """Load a Hub (or local) ACT checkpoint onto CPU and attach its norm processors."""
    torch.set_num_threads(4)
    cfg = PreTrainedConfig.from_pretrained(policy_id)
    cfg.device = "cpu"
    print(f"Loading {policy_id} on {cfg.device} (chunk={cfg.chunk_size}, n_action_steps={cfg.n_action_steps})")
    policy = ACTPolicy.from_pretrained(policy_id, config=cfg)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy.config,
        pretrained_path=policy_id,
        preprocessor_overrides={"device_processor": {"device": "cpu"}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )
    return policy, preprocessor, postprocessor


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


def make_robot(max_delta: float) -> SO101Follower:
    cameras = {
        # Key must be "wrist": the policy input is observation.images.wrist.
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
            # Degrees (gripper is 0-100) allowed to change on one control tick.
            max_relative_target=max_delta,
        )
    )
    _retry_bus(robot)
    # Calibration file already exists for this id. Do not prompt.
    robot.connect(calibrate=False)
    if robot.calibration and not robot.is_calibrated:
        print(f"Writing saved calibration {robot.calibration_fpath}")
        robot.bus.write_calibration(robot.calibration)
    return robot


view_lock = threading.Lock()
view = {
    "jpeg": None,
    "pose": {},
    "action": {},
    "infer_ms": None,
    "mode": "starting",
    "tick": 0,
}
view_stop = threading.Event()


def publish_view(obs, action, infer_ms, mode, tick):
    """Store the policy's wrist frame and the numbers beside it."""
    bgr = cv2.cvtColor(obs["wrist"], cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        return
    pose = {j: round(float(obs[j]), 1) for j in JOINTS}
    cmd = {j: round(float(action[j]), 1) for j in JOINTS}
    with view_lock:
        view["jpeg"] = buf.tobytes()
        view["pose"] = pose
        view["action"] = cmd
        view["infer_ms"] = round(infer_ms * 1000)
        view["mode"] = mode
        view["tick"] = tick


def _urls(port: int) -> list[str]:
    urls = [f"http://127.0.0.1:{port}"]
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    # de-dupe while keeping order
    seen = set()
    out = []
    for url in urls:
        if url not in seen:
            seen.add(url)
            out.append(url)
    return out


HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ACT pick — wrist view</title>
<style>
body{font-family:sans-serif;background:#1e1e2e;color:#cdd6f4;margin:0}
main{display:flex;gap:24px;align-items:flex-start;padding:20px;flex-wrap:wrap}
img{width:640px;max-width:100%;background:#111;border-radius:8px}
h1{font-size:1.15em;margin:0 0 8px}
#mode{font-family:monospace;margin-bottom:14px}
table{border-collapse:collapse;font-family:monospace;font-size:14px}
td{padding:4px 12px 4px 0}
td.n{text-align:right;min-width:64px}
.dim{color:#a6adc8}
</style></head><body>
<main>
<img id="cam" alt="wrist camera" src="/stream">
<div>
<h1>ACT pick — wrist camera</h1>
<div id="mode">connecting…</div>
<table>
<tr><td class="dim">joint</td><td class="n dim">pose</td><td class="n dim">command</td></tr>
<tbody id="rows"></tbody>
</table>
<p class="dim">The image is the 640×480 frame passed to the policy. Ctrl-C in the terminal stops it.</p>
</div>
</main>
<script>
const joints=__JOINTS__;
function row(j, pose, cmd){
  const name=j.replace('.pos','');
  const p=pose[j]==null?'—':pose[j].toFixed(1);
  const c=cmd[j]==null?'—':cmd[j].toFixed(1);
  return `<tr><td>${name}</td><td class="n">${p}</td><td class="n">${c}</td></tr>`;
}
async function tick(){
  try{
    const r=await fetch('/state');
    const d=await r.json();
    const moving=d.mode==='driving';
    document.getElementById('mode').textContent=
      (moving?'driving the arm':'preview — not moving')
      +'  ·  last inference '+d.infer_ms+' ms';
    document.getElementById('rows').innerHTML=
      joints.map(j=>row(j,d.pose,d.action)).join('');
  }catch(e){}
}
tick();
setInterval(tick, 200);
</script>
</body></html>""".replace("__JOINTS__", json.dumps(JOINTS))


class ViewHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/state":
            with view_lock:
                body = json.dumps({
                    "pose": view["pose"],
                    "action": view["action"],
                    "infer_ms": view["infer_ms"],
                    "mode": view["mode"],
                    "tick": view["tick"],
                }).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/frame.jpg":
            with view_lock:
                jpeg = view["jpeg"]
            if jpeg is None:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while not view_stop.is_set():
                    with view_lock:
                        jpeg = view["jpeg"]
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


class ViewServer(ThreadingHTTPServer):
    allow_reuse_address = True


def start_view():
    try:
        server = ViewServer(("0.0.0.0", VIEW_PORT), ViewHandler)
    except OSError as e:
        if getattr(e, "errno", None) == 98:
            print(f"Port {VIEW_PORT} is already in use. Stop the other viewer, then start again.")
        raise
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("Wrist view (keep this process running):")
    for url in _urls(VIEW_PORT):
        print(f"  {url}")
    return server


def predict(policy, preprocessor, postprocessor, observation: dict) -> dict[str, float]:
    frame = build_dataset_frame(DATASET_FEATURES, observation, prefix="observation")
    frame = prepare_observation_for_inference(
        frame, torch.device("cpu"), task="", robot_type="so_follower"
    )
    frame = preprocessor(frame)
    with torch.inference_mode():
        action = policy.select_action(frame)
    action = postprocessor(action)
    return make_robot_action(action, DATASET_FEATURES)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Show the live wrist view and predictions, but do not move the arm",
    )
    parser.add_argument("--seconds", type=float, default=0.0, help="Stop after N seconds (0 = until Ctrl-C)")
    parser.add_argument(
        "--max-delta",
        type=float,
        default=10.0,
        help="Max joint step per tick, in degrees (gripper uses the same number on its 0-100 scale)",
    )
    parser.add_argument(
        "--policy",
        default=POLICY_ID,
        help="Hub repo or local path of the ACT checkpoint (default: base pick policy)",
    )
    args = parser.parse_args()

    policy, preprocessor, postprocessor = load_policy(args.policy)
    policy.reset()
    robot = make_robot(args.max_delta)
    print("Arm + LG Smart Cam connected")
    print("  port", ARM_PORT, "camera", CAMERA_PATH, f"{CAMERA_W}x{CAMERA_H}@{FPS} MJPG")

    try:
        obs = robot.get_observation()
        image = obs["wrist"]
        pose = {j: round(float(obs[j]), 1) for j in JOINTS}
        print(f"wrist frame {tuple(image.shape)} dtype={image.dtype}")
        print("pose", pose)
        if image.shape[:2] != (CAMERA_H, CAMERA_W):
            raise RuntimeError(f"Expected {(CAMERA_H, CAMERA_W)} wrist frame, got {image.shape}")

        mode = "preview" if args.check else "driving"
        t0 = time.perf_counter()
        action = predict(policy, preprocessor, postprocessor, obs)
        dt = time.perf_counter() - t0
        print(f"first action in {dt:.2f}s:", {k: round(v, 1) for k, v in action.items()})
        publish_view(obs, action, dt, mode, 0)
        server = start_view()

        if args.check:
            print("Preview only — no command will be sent. Ctrl-C stops.")
        else:
            print("Driving the arm. Ctrl-C stops and disables torque.")
        deadline = time.perf_counter() + args.seconds if args.seconds > 0 else None
        tick = 0
        while deadline is None or time.perf_counter() < deadline:
            loop_t = time.perf_counter()
            obs = robot.get_observation()
            infer_t = time.perf_counter()
            action = predict(policy, preprocessor, postprocessor, obs)
            infer_dt = time.perf_counter() - infer_t
            if not args.check:
                robot.send_action(action)
            tick += 1
            publish_view(obs, action, infer_dt, mode, tick)
            if infer_dt > 0.05 or tick % FPS == 0:
                pose = {j: round(float(obs[j]), 1) for j in JOINTS}
                print(f"t={tick / FPS:5.1f}s infer={infer_dt * 1000:6.0f}ms pose={pose}")
            remain = (1.0 / FPS) - (time.perf_counter() - loop_t)
            if remain > 0:
                time.sleep(remain)
    finally:
        view_stop.set()
        if "server" in locals():
            server.shutdown()
        robot.disconnect()
        print("Disconnected")


if __name__ == "__main__":
    main()
