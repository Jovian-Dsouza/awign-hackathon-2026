# -*- coding: utf-8 -*-
"""Mecanum motor calibration dashboard.

Live web UI on the Raspberry Pi that names each shield channel, flips
direction, and sets speed. Commands go over serial to the Arduino
running arduino/mecanum/mecanum.ino. Stdlib HTTP server + pyserial —
same shape as so101-arm/web_arm.py.
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import serial

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_link import MecanumLink  # noqa: E402

SERIAL_PORT = '/dev/ttyACM0'
BAUD = 115200
HTTP_PORT = 8001
WATCHDOG_MS = 500
CONFIG_DIR = os.path.expanduser('~/.mecanum')
CONFIG_PATH = os.path.join(CONFIG_DIR, 'motors.json')

DEFAULT_MOTORS = [
    {'id': 1, 'name': 'Front left', 'invert': False, 'speed': 0},
    {'id': 2, 'name': 'Front right', 'invert': False, 'speed': 0},
    {'id': 3, 'name': 'Rear left', 'invert': False, 'speed': 0},
    {'id': 4, 'name': 'Rear right', 'invert': False, 'speed': 0},
]


def _normalize(motors):
    """Ensure four motor entries with sane fields."""
    by_id = {int(m.get('id', i + 1)): m for i, m in enumerate(motors[:4])}
    out = []
    for i, default in enumerate(DEFAULT_MOTORS):
        mid = i + 1
        src = by_id.get(mid, default)
        out.append({
            'id': mid,
            'name': str(src.get('name') or default['name']),
            'invert': bool(src.get('invert', False)),
            'speed': int(max(0, min(255, int(src.get('speed', 0) or 0)))),
        })
    return out


def load_config():
    os.makedirs(CONFIG_DIR, exist_ok=True)
    if not os.path.exists(CONFIG_PATH):
        return [dict(m) for m in DEFAULT_MOTORS]
    try:
        with open(CONFIG_PATH) as f:
            data = json.load(f)
        if isinstance(data, dict):
            data = data.get('motors', DEFAULT_MOTORS)
        return _normalize(data)
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
        return [dict(m) for m in DEFAULT_MOTORS]


def save_config(motors_to_save, *, zero_speeds=True):
    """Persist names and invert flags. Speeds default to 0 so reload starts stopped."""
    os.makedirs(CONFIG_DIR, exist_ok=True)
    payload = _normalize(motors_to_save)
    if zero_speeds:
        for m in payload:
            m['speed'] = 0
    with open(CONFIG_PATH, 'w') as f:
        json.dump({'motors': payload, 'path': CONFIG_PATH}, f, indent=2)
        f.write('\n')
    return payload


motors = load_config()
# Live speeds are for testing only; start stopped after load.
for _m in motors:
    _m['speed'] = 0
dirty = False


def reinit_arduino(link: MecanumLink) -> None:
    """After USB brownout/reboot: restore invert, watchdog, live speeds."""
    flags = ' '.join('1' if m['invert'] else '0' for m in motors)
    link.send_line(f'I {flags}', retry=False)
    link.send_line(f'H {WATCHDOG_MS}', retry=False)
    if any(m['speed'] for m in motors):
        speeds = ' '.join(str(m['speed']) for m in motors)
        link.send_line(f'W {speeds}', retry=False)
    else:
        link.send_line('S', retry=False)


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


def send_motor_speed(motor_id: int, speed: int):
    speed = max(0, min(255, int(speed)))
    return send_line(f'M {motor_id} {speed}')


def send_all_speeds():
    speeds = ' '.join(str(m['speed']) for m in motors)
    return send_line(f'W {speeds}')


def stop_all_motors():
    for m in motors:
        m['speed'] = 0
    return send_line('S')


def keepalive_loop():
    """Refresh the Arduino watchdog while any motor is spinning."""
    while True:
        time.sleep(0.2)
        if not any(m['speed'] for m in motors):
            continue
        try:
            send_all_speeds()
        except serial.SerialException as e:
            print('keepalive failed:', e)


link.start()
threading.Thread(target=keepalive_loop, daemon=True).start()
print(f'Arduino link on {link.port or SERIAL_PORT} @ {BAUD}')

HTML = '''<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Mecanum Motor Calibration</title>
<style>
:root{
  --bg:#1e1e2e;--surface:#313244;--text:#cdd6f4;--muted:#a6adc8;
  --green:#a6e3a1;--red:#f38ba8;--blue:#89b4fa;--peach:#fab387;
}
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:var(--bg);color:var(--text);
  max-width:960px;margin:24px auto;padding:0 16px}
h1{font-size:1.35em;text-align:center;margin:0 0 8px}
.sub{text-align:center;color:var(--muted);font-size:.9em;margin-bottom:20px}
.toolbar{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin-bottom:18px}
button{padding:8px 16px;border:none;border-radius:8px;cursor:pointer;font-weight:600;color:#111}
button:disabled{opacity:.5;cursor:default}
.stop{background:var(--red)}.step{background:var(--blue)}.save{background:var(--green)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:14px}
.card{background:var(--surface);border-radius:12px;padding:14px 16px}
.card h2{margin:0 0 10px;font-size:1em;color:var(--peach)}
.field{margin:10px 0}
.field label{display:block;font-size:.8em;color:var(--muted);margin-bottom:4px}
.field input[type=text]{width:100%;padding:8px 10px;border:none;border-radius:8px;
  background:#1e1e2e;color:var(--text)}
.row{display:flex;align-items:center;gap:8px;flex-wrap:wrap}
.row input[type=range]{flex:1;min-width:120px}
.val{font-family:ui-monospace,monospace;min-width:2.5em;text-align:right}
.toggle{display:flex;gap:6px}
.toggle button{flex:1;background:#45475a;color:var(--text)}
.toggle button.on{background:var(--peach);color:#111}
#status{text-align:center;font-family:ui-monospace,monospace;margin:16px 0;
  min-height:1.2em;color:var(--muted);font-size:.85em}
#dirty{text-align:center;font-size:.85em;color:var(--peach);min-height:1.2em;margin-bottom:8px}
</style></head><body>
<h1>Mecanum Motor Calibration</h1>
<p class="sub">Name each channel, flip direction, set speed. Save when the directions look right.</p>
<div class="toolbar">
  <button class="stop" onclick="stopAll()">Stop all</button>
  <button class="save" onclick="saveConfig()">Save config</button>
</div>
<div id="dirty"></div>
<div class="grid" id="cards"></div>
<div id="status"></div>
<script>
let motors=[];
let dirty=false;
function setStatus(t){document.getElementById('status').textContent=t||'';}
function setDirty(v){
  dirty=!!v;
  document.getElementById('dirty').textContent=dirty?'Unsaved changes':'';
}
async function api(path,body){
  const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body||{})});
  const d=await r.json();
  if(d.reply) setStatus(d.reply);
  if(d.error) setStatus('ERR '+d.error);
  if(typeof d.dirty==='boolean') setDirty(d.dirty);
  return d;
}
function render(){
  const root=document.getElementById('cards');
  root.innerHTML='';
  motors.forEach(m=>{
    const card=document.createElement('div');
    card.className='card';
    card.innerHTML=`
      <h2>M${m.id}</h2>
      <div class="field">
        <label>Name</label>
        <input type="text" value="${m.name.replace(/"/g,'&quot;')}" data-id="${m.id}" class="name">
      </div>
      <div class="field">
        <label>Direction</label>
        <div class="toggle">
          <button type="button" class="dir-normal ${m.invert?'':'on'}" data-id="${m.id}">Normal</button>
          <button type="button" class="dir-rev ${m.invert?'on':''}" data-id="${m.id}">Reversed</button>
        </div>
      </div>
      <div class="field">
        <label>Speed</label>
        <div class="row">
          <button type="button" class="step" data-id="${m.id}" data-delta="-10">−10</button>
          <button type="button" class="step" data-id="${m.id}" data-delta="-1">−</button>
          <input type="range" min="0" max="255" value="${m.speed}" data-id="${m.id}" class="speed">
          <span class="val" id="v_${m.id}">${m.speed}</span>
          <button type="button" class="step" data-id="${m.id}" data-delta="1">+</button>
          <button type="button" class="step" data-id="${m.id}" data-delta="10">+10</button>
        </div>
      </div>
      <button type="button" class="stop" data-id="${m.id}" style="width:100%;margin-top:6px">Stop motor</button>
    `;
    root.appendChild(card);
  });
  root.querySelectorAll('input.name').forEach(el=>{
    el.addEventListener('change',async e=>{
      const id=+e.target.dataset.id;
      const d=await api('/name',{id,name:e.target.value});
      if(d.motors) motors=d.motors;
    });
  });
  root.querySelectorAll('.dir-normal').forEach(el=>{
    el.addEventListener('click',()=>setInvert(+el.dataset.id,false));
  });
  root.querySelectorAll('.dir-rev').forEach(el=>{
    el.addEventListener('click',()=>setInvert(+el.dataset.id,true));
  });
  root.querySelectorAll('input.speed').forEach(el=>{
    el.addEventListener('input',e=>{
      document.getElementById('v_'+e.target.dataset.id).textContent=e.target.value;
      setSpeed(+e.target.dataset.id,+e.target.value,false);
    });
  });
  root.querySelectorAll('button.step').forEach(el=>{
    el.addEventListener('click',()=>{
      const id=+el.dataset.id;
      const m=motors.find(x=>x.id===id);
      const next=Math.max(0,Math.min(255,m.speed+(+el.dataset.delta)));
      setSpeed(id,next);
    });
  });
  root.querySelectorAll('button.stop[data-id]').forEach(el=>{
    el.addEventListener('click',()=>setSpeed(+el.dataset.id,0));
  });
}
async function setInvert(id,invert){
  const d=await api('/invert',{id,invert});
  if(d.motors){motors=d.motors;render();}
}
async function setSpeed(id,speed,rerender=true){
  const m=motors.find(x=>x.id===id);
  if(m) m.speed=speed;
  const d=await api('/speed',{id,speed});
  if(d.motors){
    motors=d.motors;
    if(rerender) render();
    else{
      const el=document.querySelector(`input.speed[data-id="${id}"]`);
      const val=document.getElementById('v_'+id);
      if(el) el.value=speed;
      if(val) val.textContent=speed;
    }
  }
}
async function stopAll(){
  const d=await api('/stop',{});
  if(d.motors){motors=d.motors;render();}
}
async function saveConfig(){
  const d=await api('/save',{});
  if(d.ok){
    setStatus('Saved '+ (d.path||'config'));
    if(d.motors) motors=d.motors;
  }
}
async function boot(){
  const r=await fetch('/config');
  const d=await r.json();
  motors=d.motors||[];
  if(typeof d.dirty==='boolean') setDirty(d.dirty);
  if(d.reply) setStatus(d.reply);
  render();
}
window.addEventListener('pagehide',()=>{navigator.sendBeacon('/stop',JSON.stringify({}));});
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
        if self.path == '/config':
            self._json({
                'motors': motors,
                'reply': link.last_reply,
                'dirty': dirty,
                'path': CONFIG_PATH,
                'connected': link.connected,
                'port': link.port,
                'reconnects': link.reconnect_count,
            })
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(HTML.encode())

    def do_POST(self):
        global motors, dirty
        data = self._read_json()
        try:
            if self.path == '/name':
                mid = int(data['id'])
                name = str(data.get('name') or '').strip() or f'Motor {mid}'
                for m in motors:
                    if m['id'] == mid:
                        if m['name'] != name:
                            dirty = True
                        m['name'] = name
                        break
                self._json({'ok': True, 'motors': motors, 'dirty': dirty})
            elif self.path == '/invert':
                mid = int(data['id'])
                invert = bool(data.get('invert'))
                for m in motors:
                    if m['id'] == mid:
                        if m['invert'] != invert:
                            dirty = True
                        m['invert'] = invert
                        break
                reply = send_inverts()
                # Re-send this motor's speed so a spinning wheel reverses now
                # even if the Arduino firmware is older than the I re-apply.
                for m in motors:
                    if m['id'] == mid and m['speed']:
                        reply = send_motor_speed(mid, m['speed'])
                        break
                self._json({
                    'ok': True, 'motors': motors, 'reply': reply, 'dirty': dirty,
                })
            elif self.path == '/speed':
                mid = int(data['id'])
                speed = max(0, min(255, int(data.get('speed', 0))))
                for m in motors:
                    if m['id'] == mid:
                        m['speed'] = speed
                        break
                reply = send_motor_speed(mid, speed)
                self._json({
                    'ok': True, 'motors': motors, 'reply': reply, 'dirty': dirty,
                })
            elif self.path == '/stop':
                reply = stop_all_motors()
                self._json({
                    'ok': True, 'motors': motors, 'reply': reply, 'dirty': dirty,
                })
            elif self.path == '/save':
                saved = save_config(motors, zero_speeds=True)
                dirty = False
                print(f'Saved config to {CONFIG_PATH}')
                self._json({
                    'ok': True,
                    'motors': motors,
                    'saved': saved,
                    'path': CONFIG_PATH,
                    'dirty': dirty,
                })
            else:
                self._json({'ok': False, 'error': 'not found'}, 404)
        except (KeyError, TypeError, ValueError) as e:
            self._json({'ok': False, 'error': str(e)}, 400)
        except serial.SerialException as e:
            self._json({'ok': False, 'error': f'serial: {e}'}, 500)
        except OSError as e:
            self._json({'ok': False, 'error': f'save: {e}'}, 500)


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


class ReuseHTTPServer(HTTPServer):
    allow_reuse_address = True


server = None
try:
    server = ReuseHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
    print('Mecanum calibration panel (keep this process running):')
    for url in _urls_for_port(HTTP_PORT):
        print(f'  {url}')
    server.serve_forever()
except KeyboardInterrupt:
    pass
except OSError as e:
    if getattr(e, 'errno', None) == 98:  # EADDRINUSE
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
