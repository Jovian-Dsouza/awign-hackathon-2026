# -*- coding: utf-8 -*-
"""Mecanum whole-robot drive panel.

Loads calibration from ~/.mecanum/motors.json (names + invert flags),
applies invert over serial, then drives the chassis with body-velocity
commands. Keyboard WASD (plus Q/E yaw) in the browser; live USB webcam
as an MJPEG stream. Stdlib HTTP + pyserial + OpenCV.
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_link import MecanumLink  # noqa: E402

SERIAL_PORT = '/dev/ttyACM0'
BAUD = 115200
HTTP_PORT = 8002
WATCHDOG_MS = 500
CAMERA_INDEX = 0  # /dev/video0 — LG Smart Cam
CONFIG_PATH = os.path.expanduser('~/.mecanum/motors.json')

# Channel order matches the Arduino shield (M1..M4), not display names.
DEFAULT_MOTORS = [
    {'id': 1, 'name': 'Front left', 'invert': False, 'speed': 0},
    {'id': 2, 'name': 'Front right', 'invert': False, 'speed': 0},
    {'id': 3, 'name': 'Rear left', 'invert': False, 'speed': 0},
    {'id': 4, 'name': 'Rear right', 'invert': False, 'speed': 0},
]


def load_motors():
    """Read ~/.mecanum/motors.json; keep invert/name keyed by shield id 1–4."""
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
        mid = default['id']
        motors.append(by_id.get(mid, dict(default)))
    print(f'Loaded config from {CONFIG_PATH}')
    for m in motors:
        print(f"  M{m['id']}  {m['name']!r}  invert={m['invert']}")
    return motors


motors = load_motors()
# Last commanded body velocity (logical, before Arduino invert).
cmd = {'vx': 0, 'vy': 0, 'wz': 0, 'scale': 160}
frame_lock = threading.Lock()
latest_jpeg = None
camera_ok = False


def reinit_arduino(link: MecanumLink) -> None:
    """After USB brownout/reboot: restore invert, watchdog, last motion."""
    flags = ' '.join('1' if m['invert'] else '0' for m in motors)
    link.send_line(f'I {flags}', retry=False)
    link.send_line(f'H {WATCHDOG_MS}', retry=False)
    if cmd['vx'] == 0 and cmd['vy'] == 0 and cmd['wz'] == 0:
        link.send_line('S', retry=False)
    else:
        link.send_line(f"V {cmd['vx']} {cmd['vy']} {cmd['wz']}", retry=False)


link = MecanumLink(
    baud=BAUD,
    preferred_port=SERIAL_PORT,
    on_reconnect=reinit_arduino,
)


def send_line(line: str) -> str:
    return link.send_line(line)


def send_inverts():
    flags = ' '.join('1' if m['invert'] else '0' for m in motors)
    return send_line(f'I {flags}')


def drive_body(vx: int, vy: int, wz: int) -> str:
    vx = max(-255, min(255, int(vx)))
    vy = max(-255, min(255, int(vy)))
    wz = max(-255, min(255, int(wz)))
    cmd['vx'], cmd['vy'], cmd['wz'] = vx, vy, wz
    if vx == 0 and vy == 0 and wz == 0:
        return send_line('S')
    return send_line(f'V {vx} {vy} {wz}')


def stop_all() -> str:
    return drive_body(0, 0, 0)


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
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Mecanum Drive</title>
<style>
:root{
  --bg:#1e1e2e;--surface:#313244;--text:#cdd6f4;--muted:#a6adc8;
  --green:#a6e3a1;--red:#f38ba8;--blue:#89b4fa;--peach:#fab387;--key:#45475a;
}
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--text);
  margin:0;min-height:100vh}
.wrap{max-width:1100px;margin:0 auto;padding:20px 16px 40px}
h1{font-size:1.35em;text-align:center;margin:0 0 6px}
.sub{text-align:center;color:var(--muted);font-size:.9em;margin-bottom:18px}
.layout{display:grid;grid-template-columns:1.4fr 1fr;gap:16px}
@media(max-width:820px){.layout{grid-template-columns:1fr}}
.panel{background:var(--surface);border-radius:12px;padding:14px}
.panel h2{margin:0 0 12px;font-size:1em;color:var(--peach)}
.cam{width:100%;aspect-ratio:4/3;background:#111;border-radius:8px;object-fit:contain;display:block}
.keys{display:grid;grid-template-columns:repeat(3,56px);gap:8px;justify-content:center;margin:14px 0}
.k{width:56px;height:56px;border-radius:10px;background:var(--key);display:flex;align-items:center;
  justify-content:center;font-weight:700;font-family:ui-monospace,monospace;user-select:none}
.k.on{background:var(--blue);color:#111}
.field{margin:12px 0}
.field label{display:block;font-size:.8em;color:var(--muted);margin-bottom:4px}
.row{display:flex;align-items:center;gap:10px}
.row input[type=range]{flex:1}
.val{font-family:ui-monospace,monospace;min-width:2.5em;text-align:right}
button.stop{width:100%;padding:12px;border:none;border-radius:10px;background:var(--red);
  color:#111;font-weight:700;cursor:pointer;margin-top:8px}
.motors{font-size:.85em;color:var(--muted);line-height:1.5;margin-top:12px}
#status{text-align:center;font-family:ui-monospace,monospace;margin-top:14px;
  min-height:1.2em;color:var(--muted);font-size:.85em}
.hint{color:var(--muted);font-size:.85em;text-align:center;margin-top:8px}
</style></head><body>
<div class="wrap">
  <h1>Mecanum Drive</h1>
  <p class="sub">WASD translate · Q/E rotate · Space stop · holds repeat over serial</p>
  <div class="layout">
    <div class="panel">
      <h2>Camera</h2>
      <img class="cam" id="cam" alt="webcam" src="/stream">
    </div>
    <div class="panel">
      <h2>Controls</h2>
      <div class="keys">
        <div></div><div class="k" id="k_q">Q</div><div class="k" id="k_w">W</div>
        <div class="k" id="k_a">A</div><div class="k" id="k_s">S</div><div class="k" id="k_d">D</div>
        <div></div><div class="k" id="k_e">E</div><div></div>
      </div>
      <div class="field">
        <label>Speed</label>
        <div class="row">
          <input type="range" id="scale" min="40" max="255" value="160">
          <span class="val" id="scaleVal">160</span>
        </div>
      </div>
      <button class="stop" onclick="emergencyStop()">Stop</button>
      <div class="motors" id="motors"></div>
      <p class="hint">Click the page once so keyboard focus is active.</p>
    </div>
  </div>
  <div id="status"></div>
</div>
<script>
const pressed=new Set();
let scale=160;
let busy=false;
let pending=null;
function setStatus(t){document.getElementById('status').textContent=t||'';}
function syncKeys(){
  ['q','w','e','a','s','d'].forEach(k=>{
    const el=document.getElementById('k_'+k);
    if(el) el.classList.toggle('on', pressed.has(k));
  });
}
function compute(){
  const s=scale;
  let vx=0,vy=0,wz=0;
  if(pressed.has('w')) vx+=s;
  if(pressed.has('s')) vx-=s;
  if(pressed.has('a')) vy+=s;
  if(pressed.has('d')) vy-=s;
  if(pressed.has('q')) wz+=s;
  if(pressed.has('e')) wz-=s;
  return {vx,vy,wz};
}
async function sendDrive(body){
  if(busy){pending=body;return;}
  busy=true;
  try{
    const r=await fetch('/drive',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify(body)});
    const d=await r.json();
    if(d.reply) setStatus(d.reply);
    if(d.error) setStatus('ERR '+d.error);
  }catch(e){setStatus('ERR '+e);}
  finally{
    busy=false;
    if(pending){const p=pending;pending=null;sendDrive(p);}
  }
}
function pushState(){
  syncKeys();
  sendDrive(compute());
}
function emergencyStop(){
  pressed.clear();
  syncKeys();
  sendDrive({vx:0,vy:0,wz:0});
}
function isTyping(el){
  return el && (el.tagName==='INPUT'||el.tagName==='TEXTAREA'||el.isContentEditable);
}
window.addEventListener('keydown',e=>{
  if(isTyping(e.target)) return;
  const k=e.key.toLowerCase();
  if(k===' '){e.preventDefault();emergencyStop();return;}
  if(!['w','a','s','d','q','e'].includes(k)) return;
  e.preventDefault();
  if(pressed.has(k)) return;
  pressed.add(k);
  pushState();
});
window.addEventListener('keyup',e=>{
  const k=e.key.toLowerCase();
  if(!pressed.has(k)) return;
  pressed.delete(k);
  pushState();
});
window.addEventListener('blur',()=>emergencyStop());
document.getElementById('scale').addEventListener('input',e=>{
  scale=+e.target.value;
  document.getElementById('scaleVal').textContent=scale;
  if(pressed.size) pushState();
});
window.addEventListener('pagehide',()=>{
  navigator.sendBeacon('/drive',JSON.stringify({vx:0,vy:0,wz:0}));
});
async function boot(){
  const r=await fetch('/status');
  const d=await r.json();
  if(d.scale){scale=d.scale;document.getElementById('scale').value=scale;
    document.getElementById('scaleVal').textContent=scale;}
  const box=document.getElementById('motors');
  box.innerHTML='<strong style="color:var(--text)">Calibration</strong><br>'+
    (d.motors||[]).map(m=>`M${m.id} ${m.name} — ${m.invert?'reversed':'normal'}`).join('<br>')+
    `<br><span style="opacity:.7">${d.config_path||''}</span>`;
  if(d.camera===false) setStatus('Camera not available');
  else if(!d.connected) setStatus('Arduino disconnected — reconnecting…');
  else if(d.reply) setStatus(d.reply);
}
setInterval(async ()=>{
  try{
    const r=await fetch('/status'); const d=await r.json();
    if(!d.connected) setStatus('Arduino disconnected — reconnecting…');
  }catch(e){}
}, 2000);
boot();
</script></body></html>'''


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
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
        if self.path == '/status':
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
            })
            return
        if self.path == '/stream':
            self.send_response(200)
            self.send_header(
                'Content-Type', 'multipart/x-mixed-replace; boundary=frame'
            )
            self.send_header('Cache-Control', 'no-cache, no-store')
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
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(HTML.encode())

    def do_POST(self):
        data = self._read_json()
        try:
            if self.path == '/drive':
                if 'scale' in data:
                    cmd['scale'] = max(40, min(255, int(data['scale'])))
                vx = int(data.get('vx', 0))
                vy = int(data.get('vy', 0))
                wz = int(data.get('wz', 0))
                reply = drive_body(vx, vy, wz)
                self._json({'ok': True, 'reply': reply, 'cmd': dict(cmd)})
            else:
                self._json({'ok': False, 'error': 'not found'}, 404)
        except (KeyError, TypeError, ValueError) as e:
            self._json({'ok': False, 'error': str(e)}, 400)
        except serial.SerialException as e:
            self._json({'ok': False, 'error': f'serial: {e}'}, 500)


def _urls_for_port(port: int) -> list[str]:
    urls = [f'http://127.0.0.1:{port}']
    try:
        import socket
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith('127.'):
                urls.append(f'http://{ip}:{port}')
    except OSError:
        pass
    return urls


class ReuseHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


server = None
try:
    server = ReuseHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print('Mecanum drive panel (keep this process running):')
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
    try:
        send_line('S')
        send_line('H 0')
    except Exception:
        pass
    if server is not None:
        server.server_close()
    link.close()
    print('Disconnected')
