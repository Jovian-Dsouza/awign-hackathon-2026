"""Control panel: the live simulation with real buttons, in your browser. No terminal needed once it is up.

  uv run dsr-panel            # the page at http://127.0.0.1:8765 - the only window you need
  uv run dsr-panel --native   # optional: also a native MuJoCo window mirroring the run (debugging)

Everything is controlled from the page: pick the rack / drop the item anywhere on the table, pick the camera
(overview / robot / shelf / tray / wrist), then press Run.

Each Run button resets everything to the start (rover on the dock, arm folded, items in place),
runs the whole task, and drives back to the start position. Clicking another button mid-run
aborts the current run and starts the new one from zero. Plain `python` is enough (no mjpython).
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import mujoco
from PIL import Image

from dsr_sim.constants import DEFAULT_SCENE
from dsr_sim.pick import VIEWS, Closed, Restart, RunControl, Task
from dsr_sim.run import reset_scene
from dsr_sim.scenarios import SCENARIOS, Scenario
from dsr_sim.world import ParamError, Params, layout_info, validate

WIDTH, HEIGHT = 960, 600
FPS = 20
WRIST_FPS = 12


def _under_mjpython() -> bool:
    """True when running inside MuJoCo's `mjpython` launcher (what the viewer needs on macOS)."""
    try:
        from mujoco import viewer as mj_viewer

        return getattr(mj_viewer, "_MJPYTHON", None) is not None
    except Exception:  # noqa: BLE001
        return False


def _native_possible() -> bool:
    """The native MuJoCo window can be opened from here (mjpython on macOS, plain python elsewhere)."""
    return sys.platform != "darwin" or _under_mjpython()


READY = Scenario("ready", "Ready", "", run=lambda task: True)  # base scene, nothing run


class Tee(io.TextIOBase):
    """Captures printed stage messages for the page while still echoing to the terminal."""

    def __init__(self, sink: deque, real) -> None:
        self.sink, self.real, self._buf = sink, real, ""

    def write(self, text: str) -> int:
        self.real.write(text)
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line.strip() and "[ik]" not in line and "      [" not in line:
                self.sink.append(line.strip())
        return len(text)

    def flush(self) -> None:
        self.real.flush()


# Fixed on-robot cameras the page can switch to at any time (before or during a run).
CAMERA_VIEWS = {"5": "wrist_cam"}


class Worker:
    """Owns the model, the data and the renderer; everything physical happens on this one thread."""

    def __init__(self, scene: Path) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.control = RunControl(speed=1.0)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.native = None  # optional native MuJoCo window mirroring the same simulation
        self.view = "1"
        self.set_view("1")
        self.lock = threading.Lock()
        self.frame_ready = threading.Condition()
        self.jpeg: bytes = b""
        self.frame_id = 0
        self.wrist_jpeg: bytes = b""  # the robot's only camera, shown in its own panel
        self.wrist_id = 0
        self.viewers = 0  # open /stream connections; nothing is rendered while nobody is watching
        self._wrist_renderer: mujoco.Renderer | None = None
        self._last_wrist = 0.0
        self.pending: tuple[str, Params, bool] | None = None  # (scenario name or __reset__, params, record)
        self.params = Params()
        self.result: dict | None = None
        self.want_native = False
        self.native_ok = _native_possible()
        self.state = "ready"
        self.title = ""
        self.log: deque[str] = deque(maxlen=14)
        self.stop = False
        self._last_render = 0.0
        self._renderer: mujoco.Renderer | None = None
        self.native = None  # optional native MuJoCo window mirroring the same simulation
        self.last_task: str | None = None

    # ---- camera (called from HTTP threads; a stale read just means a one-frame lag)
    def set_view(self, key: str) -> None:
        self.view = key
        if key in VIEWS:
            az, el, dist, look = VIEWS[key]
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.azimuth, self.cam.elevation, self.cam.distance = az, el, dist
            self.cam.lookat[:] = look
        self._apply_native_view()

    def _apply_native_view(self) -> None:
        """Make the MuJoCo window show the same camera as the page (wrist, or the free camera)."""
        if self.native is None:
            return
        try:
            if self.view in CAMERA_VIEWS:
                self.native.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                self.native.cam.fixedcamid = self.model.camera(CAMERA_VIEWS[self.view]).id
            else:
                self.native.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                self.native.cam.azimuth, self.native.cam.elevation, self.native.cam.distance = (
                    self.cam.azimuth,
                    self.cam.elevation,
                    self.cam.distance,
                )
                self.native.cam.lookat[:] = self.cam.lookat
        except Exception:  # noqa: BLE001 - the window may be closing
            pass

    def orbit(self, dx: float, dy: float) -> None:
        if self.view in CAMERA_VIEWS:
            return
        self.cam.azimuth -= dx * 0.4
        self.cam.elevation = max(-89.0, min(-3.0, self.cam.elevation - dy * 0.4))

    def zoom(self, steps: float) -> None:
        if self.view in CAMERA_VIEWS:
            return
        self.cam.distance = max(0.2, min(6.0, self.cam.distance * (1.12 ** steps)))

    # ---- commands
    def request(self, name: str, params: Params | None = None, record: bool = False) -> None:
        if name in SCENARIOS:
            self.last_task = name
            self.params = params or self.params
        with self.lock:
            self.pending = (name, params or self.params, record)
        self.control.paused = False
        self.control.reset_requested = True  # aborts a run in progress; harmless when idle

    def toggle_pause(self) -> None:
        self.control.paused = not self.control.paused

    # ---- keys for the native MuJoCo window (the buttons live on the web page)
    def native_key(self, keycode: int) -> None:
        names = list(SCENARIOS)
        if keycode in (49, 50, 51, 52) and keycode - 49 < len(names):  # 1, 2: run that simulation
            self.last_task = names[keycode - 49]
            self.request(self.last_task)
        elif keycode in (82, 259):  # R / Backspace: reset to the start
            self.request("__reset__")
        elif keycode == 32:  # Space: start when idle, otherwise pause / resume
            if self.state == "running":
                self.toggle_pause()
            else:
                self.request(self.last_task or names[0])
                self.last_task = self.last_task or names[0]

    # ---- the "viewer" interface Task expects
    def is_running(self) -> bool:
        return not self.stop

    def sync(self) -> None:
        self._sync_native()
        self.render()

    def _sync_native(self) -> None:
        if self.native is None:
            if self.want_native and self.native_ok:
                self._open_native()
            return
        if self.native.is_running():
            self.native.sync()
        else:
            self.native = None  # the window was closed; the web page keeps working
            self.want_native = False

    def _open_native(self) -> None:
        try:
            from mujoco import viewer as mj_viewer

            self.native = mj_viewer.launch_passive(self.model, self.data, key_callback=self.native_key)
            self._apply_native_view()
        except Exception as exc:  # noqa: BLE001
            self.native_ok = False
            self.log.append(f"could not open the MuJoCo window: {exc}")

    def render(self, force: bool = False) -> None:
        if self.viewers <= 0 and self.frame_id > 0:
            return
        now = time.time()
        if not force and now - self._last_render < 1.0 / FPS:
            return
        self._last_render = now
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, HEIGHT, WIDTH)
        self._renderer.update_scene(self.data, camera=CAMERA_VIEWS.get(self.view, self.cam))
        buf = io.BytesIO()
        Image.fromarray(self._renderer.render()).save(buf, "JPEG", quality=82)
        with self.frame_ready:
            self.jpeg = buf.getvalue()
            self.frame_id += 1
            self.frame_ready.notify_all()
        if force or now - self._last_wrist >= 1.0 / WRIST_FPS:
            self._last_wrist = now
            if self._wrist_renderer is None:
                self._wrist_renderer = mujoco.Renderer(self.model, 480, 640)  # what the real USB camera streams
            self._wrist_renderer.update_scene(self.data, camera="wrist_cam")
            wbuf = io.BytesIO()
            Image.fromarray(self._wrist_renderer.render()).save(wbuf, "JPEG", quality=82)
            with self.frame_ready:
                self.wrist_jpeg = wbuf.getvalue()
                self.wrist_id += 1
                self.frame_ready.notify_all()

    # ---- main loop
    def _take_pending(self):
        with self.lock:
            req, self.pending = self.pending, None
        return req

    def run_forever(self) -> None:
        reset_scene(self.model, self.data, READY)
        while not self.stop:
            req = self._take_pending()
            if req is None:
                self._sync_native()
                self.render(force=True)
                time.sleep(1.0 / FPS)
                continue
            name, params, record = req
            self.control.reset_requested = False
            scenario = SCENARIOS.get(name, READY)
            reset_scene(self.model, self.data, scenario, params)  # back to zero, always
            if name == "__reset__":
                self.state, self.title = "ready", ""
                self.log.clear()
                self.log.append("reset to the start position")
                continue
            self.state, self.title = "running", scenario.title
            self.result = None
            self.log.clear()
            task = Task(self.model, self.data, self, self.control.speed, None, self.control)
            try:
                with contextlib.redirect_stdout(Tee(self.log, sys.__stdout__)):
                    task.run(0.5)
                    print(f"== {scenario.title} ==")
                    if record:
                        from dsr_sim import datalog

                        ok = datalog.run_recorded(task, scenario, params, record_images=True)
                    else:
                        ok = scenario.run(task, params)
                self.state = "done" if ok else "failed"
                self.result = {"ok": bool(ok), "notes": task.notes}
                self.log.append("finished: back at the start position" if ok else "finished, but it did not work out (see above)")
            except Restart:
                self.state = "ready"
                self.log.append("stopped")
            except Closed:
                break

    # ---- status for the page
    def status(self) -> dict:
        return {
            "state": self.state,
            "title": self.title,
            "paused": self.control.paused,
            "speed": round(self.control.speed, 2),
            "log": list(self.log),
            "params": self.params.as_dict(),
            "view": self.view,
            "native_ok": self.native_ok,
            "native_open": self.native is not None,
            "result": self.result,
            "tasks": [{"name": s.name, "title": s.title, "description": s.description} for s in SCENARIOS.values()],
        }


PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><title>Dark-Store Robot Simulation</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#f3f4f6;--card:#fff;--ink:#10141c;--muted:#6b7587;--line:#dcdfe6;--accent:#1c8fc4;--ok:#1f9d55;--bad:#c0392b}
@media (prefers-color-scheme:dark){:root{--bg:#0d0f14;--card:#141720;--ink:#e6e8ee;--muted:#9aa3b5;--line:#232838}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px system-ui,-apple-system,sans-serif}
.wrap{display:grid;grid-template-columns:1fr 380px;gap:16px;padding:16px;align-items:start}
@media(max-width:980px){.wrap{grid-template-columns:1fr}}
.view{position:relative;background:#000;border-radius:12px;overflow:hidden}
.view img{width:100%;display:block;cursor:grab;user-select:none;-webkit-user-drag:none}
.stagebox{display:flex;flex-direction:column;gap:8px}
.stagebar{display:flex;align-items:center;gap:12px}.stagebar .hint{flex:1}.stagebar button{padding:6px 10px}
.stage{display:grid;grid-template-columns:2fr 1fr;gap:12px;align-items:start}
@media(max-width:1250px){.stage{grid-template-columns:1fr}}
.vtitle{position:absolute;right:12px;top:12px;background:#000a;color:#fff;border-radius:8px;padding:4px 10px;font-size:12px;z-index:1}
.wristview img{cursor:default}
.fsbar{display:none}
body.fs .panel,body.fs .stagebar{display:none}
body.fs .wrap{grid-template-columns:minmax(0,1fr);padding:0;height:100vh;width:100vw;background:#000}
body.fs .stagebox{height:100vh;min-width:0;width:100vw}
body.fs .stage{display:block;position:relative;height:100vh;width:100vw;background:#000}
body.fs .stage>.view:first-child{position:absolute;inset:0;border-radius:0}
body.fs .stage>.view:first-child img{width:100%;height:100%;object-fit:cover}
body.fs .wristview{position:absolute;top:16px;right:16px;width:min(30vw,520px);z-index:3;border:2px solid #ffffff55;border-radius:12px;box-shadow:0 6px 24px #0009}
body.fs .wristview img{width:100%;height:auto;aspect-ratio:4/3;object-fit:cover}
body.fs .wristview .vtitle{top:8px;right:8px;font-size:11px}
body.fs .fsbar{display:flex;gap:8px;position:fixed;left:50%;bottom:14px;transform:translateX(-50%);z-index:5;opacity:.25;transition:opacity .2s}
body.fs .fsbar:hover{opacity:1}

.badge{position:absolute;left:12px;top:12px;background:#000a;color:#fff;border-radius:8px;padding:6px 10px;font-size:13px}
.panel{display:flex;flex-direction:column;gap:12px}
h1{font-size:1.05rem;margin:0}h2{font-size:.95rem;margin:0 0 8px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
button{font:inherit;border:0;border-radius:10px;padding:10px 12px;cursor:pointer;color:var(--ink);background:var(--line)}
button:disabled{opacity:.45;cursor:not-allowed}
.run{background:var(--accent);color:#fff;font-weight:600;width:100%;text-align:left;margin-top:10px}.run.active{outline:3px solid var(--ink)}
.row{display:flex;gap:8px}.row button{flex:1;text-align:center}
.racks{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:6px 0}
.racks button{padding:12px 6px;font-size:13px;line-height:1.2;border:2px solid transparent}
.racks button.sel{border-color:var(--accent);background:#1c8fc433;font-weight:600}
.racks button.occ{opacity:.55}.racks button small{display:block;color:var(--muted);font-size:11px}
canvas{width:100%;border-radius:8px;background:var(--bg);cursor:crosshair;border:1px solid var(--line)}
.hint{color:var(--muted);font-size:12px;margin:6px 0 0}.err{color:var(--bad);font-size:13px;margin-top:6px;min-height:1.2em}
.opts label{display:flex;gap:8px;align-items:center;margin:4px 0;font-size:14px}
.log{font:12px ui-monospace,Menlo,monospace;color:var(--muted);min-height:9em;white-space:pre-wrap}
.status{font-weight:600}.status.done{color:var(--ok)}.status.failed{color:var(--bad)}
</style></head><body><div class="wrap">
<div class="stagebox">
  <div class="stagebar"><b>Live</b><span class="hint" style="margin:0">drag the environment to orbit, scroll to zoom</span>
    <button id="fs" title="Full screen (Esc to leave)">⛶ Full screen</button></div>
  <div class="stage">
    <div class="view"><div class="vtitle">Environment</div><img id="v" src="/stream" alt="environment"><div class="badge" id="badge">connecting…</div></div>
    <div class="view wristview"><div class="vtitle">Wrist camera · what the robot sees (640×480)</div><img id="w" src="/stream_wrist" alt="wrist camera"></div>
  </div>
</div>
<div class="panel">
  <h1>Dark-store robot</h1>

  <div class="card"><h2>1 · Shelf to tray</h2>
    <div class="hint">Pick the item from this rack and carry it to the tray.</div>
    <div class="racks" id="pickRacks"></div>
    <button class="run" id="run-shelf_to_tray">▶ Run: shelf to tray</button>
  </div>

  <div class="card"><h2>2 · Restock a misplaced item</h2>
    <div class="hint">Click the table to leave the item there.</div>
    <canvas id="map" width="340" height="150"></canvas>
    <div class="hint" id="mapinfo"></div><div class="err" id="maperr"></div>
    <div class="hint">Put it back in rack:</div>
    <div class="racks" id="destRacks"></div>
    <button class="run" id="run-restock_misplaced">▶ Run: restock</button>
  </div>

  <div class="card opts"><h2>Options</h2>
    <label><input type="checkbox" id="assist" checked> Grasp assist (carry the item with a weld; off = friction only, less reliable)</label>
    <label><input type="checkbox" id="record"> Record data (trajectory + camera frames → data/ folder)</label>
    <div class="err" id="runerr"></div>
  </div>

  <div class="card"><div class="row"><button id="pause">⏸ Pause</button><button id="reset">⟲ Reset</button></div>
    <div class="row" style="margin-top:8px"><button id="slow">− Slower</button><button id="speed" disabled>1×</button><button id="fast">+ Faster</button></div></div>
  <div class="card"><div class="row"><button data-v="1">Overview</button><button data-v="2">Robot</button><button data-v="3">Shelf</button><button data-v="4">Tray</button></div>
        <div class="row" style="margin-top:8px;display:none"><button id="native">Open MuJoCo window</button></div>
    <div class="hint">The wrist camera (the robot's only camera) is always shown in its own panel.</div></div>
  <div class="card"><div class="status" id="state">Ready</div><div class="log" id="log"></div></div>
</div></div>
<div class="fsbar"><button id="fs-exit">✕ Exit full screen</button><button id="fs-pause">⏸ Pause</button><button id="fs-reset">⟲ Reset</button></div>
<script>
const $=id=>document.getElementById(id), post=(u,b)=>fetch(u,{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(b||{})});
let L=null, rack=[0,2], dest=null, item=[0.30,-0.02];
const NAMES={shelf_to_tray:'run-shelf_to_tray',restock_misplaced:'run-restock_misplaced'};

function rackGrid(el, selected, onPick, opts){
  el.innerHTML='';
  for(let lv=L.levels-1; lv>=0; lv--) for(let sl=0; sl<L.slots; sl++){
    const b=document.createElement('button'), occ=L.occupied.find(o=>o[0]===lv&&o[1]===sl), home=(L.home_rack[0]===lv&&L.home_rack[1]===sl);
    b.innerHTML=`L${lv}·S${sl}<small>${home?'its home':occ?occ[2]:'empty'}</small>`;
    if(selected&&selected[0]===lv&&selected[1]===sl) b.classList.add('sel');
    if(occ){b.classList.add('occ'); if(opts&&opts.blockOccupied) b.disabled=true;}
    b.onclick=()=>onPick([lv,sl]); el.appendChild(b);
  }
}
function drawMap(){
  const c=$('map'), g=c.getContext('2d'), [x0,y0,x1,y1]=L.table, W=c.width, H=c.height, pad=8;
  const s=Math.min((W-2*pad)/(x1-x0),(H-2*pad)/(y1-y0)), ox=(W-s*(x1-x0))/2, oy=(H-s*(y1-y0))/2;
  const X=x=>ox+(x-x0)*s, Y=y=>oy+(y1-y)*s, R=(r,f,st)=>{g.beginPath();g.rect(X(r[0]),Y(r[3]),(r[2]-r[0])*s,(r[3]-r[1])*s);if(f){g.fillStyle=f;g.fill()}if(st){g.strokeStyle=st;g.stroke()}};
  c._t={X,Y,s,x0,y1,ox,oy}; g.clearRect(0,0,W,H);
  R(L.table,'#ffffff','#9aa3b5');
  L.keep_out.forEach(r=>R(r,'#c0392b22'));
  R(L.shelf,'#b98a55','#7a5a33'); R(L.tray,'#d8b98a','#7a5a33');
  g.fillStyle='#1c8fc4'; g.fillRect(X(L.dock[0])-6,Y(L.dock[1])-6,12,12);
  g.fillStyle='#10141c'; g.font='10px system-ui'; g.fillText('shelf',X(L.shelf[0])+4,Y(L.shelf[3])+12); g.fillText('tray',X(L.tray[0])+4,Y(L.tray[3])+12); g.fillText('start',X(L.dock[0])-12,Y(L.dock[1])+18);
  g.beginPath(); g.arc(X(item[0]),Y(item[1]),6,0,7); g.fillStyle='#e67e22'; g.fill(); g.strokeStyle='#000'; g.stroke();
  $('mapinfo').textContent=`item at x ${item[0].toFixed(2)} m, y ${item[1].toFixed(2)} m`;
}
$('map').addEventListener('click',e=>{
  const c=$('map'), r=c.getBoundingClientRect(), t=c._t, px=(e.clientX-r.left)*c.width/r.width, py=(e.clientY-r.top)*c.height/r.height;
  const x=t.x0+(px-t.ox)/t.s, y=t.y1-(py-t.oy)/t.s, [tx0,ty0,tx1,ty1]=L.table, m=L.inset;
  let why='';
  if(x<tx0+m||x>tx1-m||y<ty0+m||y>ty1-m) why='too close to the edge of the table';
  else if(L.keep_out.some(k=>x>=k[0]&&x<=k[2]&&y>=k[1]&&y<=k[3])) why='too close to the shelf or tray';
  else if(Math.hypot(x-L.dock[0],y-L.dock[1])<0.18) why="that is the rover's start spot";
  $('maperr').textContent=why; if(!why){item=[+x.toFixed(2),+y.toFixed(2)];drawMap()}
});
function refreshGrids(){
  rackGrid($('pickRacks'),rack,r=>{rack=r;refreshGrids()});
  rackGrid($('destRacks'),dest||L.home_rack,r=>{dest=(r[0]===L.home_rack[0]&&r[1]===L.home_rack[1])?null:r;refreshGrids()},{blockOccupied:true});
}
async function run(name){
  $('runerr').textContent='';
  const params=name==='shelf_to_tray'?{rack}:{item_xy:item,dest_rack:dest};
  const r=await post('/api/run',{name,params,assist:$('assist').checked,record:$('record').checked});
  if(!r.ok){const j=await r.json();$('runerr').textContent=j.error||'could not start'}
}
$('run-shelf_to_tray').onclick=()=>run('shelf_to_tray'); $('run-restock_misplaced').onclick=()=>run('restock_misplaced');
$('pause').onclick=()=>post('/api/pause');$('reset').onclick=()=>post('/api/reset');$('native').onclick=()=>post('/api/native');
$('slow').onclick=()=>post('/api/speed',{f:1/1.5});$('fast').onclick=()=>post('/api/speed',{f:1.5});
const goFull=()=>document.fullscreenElement?document.exitFullscreen():document.documentElement.requestFullscreen().catch(()=>document.body.classList.add('fs'));
$('fs').onclick=goFull;$('fs-exit').onclick=()=>{document.fullscreenElement?document.exitFullscreen():document.body.classList.remove('fs')};
$('fs-pause').onclick=()=>post('/api/pause');$('fs-reset').onclick=()=>post('/api/reset');
document.addEventListener('fullscreenchange',()=>document.body.classList.toggle('fs',!!document.fullscreenElement));
document.addEventListener('keydown',e=>{if(e.key==='f'&&e.target.tagName!=='INPUT')goFull()});
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>post('/api/view',{key:b.dataset.v}));
let drag=null,acc={x:0,y:0},sending=false;
$('v').addEventListener('mousedown',e=>{drag=[e.clientX,e.clientY];e.preventDefault();$('v').style.cursor='grabbing'});
addEventListener('mouseup',()=>{drag=null;$('v').style.cursor='grab'});
addEventListener('mousemove',e=>{if(!drag)return;acc.x+=e.clientX-drag[0];acc.y+=e.clientY-drag[1];drag=[e.clientX,e.clientY];
 if(!sending){sending=true;setTimeout(()=>{post('/api/orbit',{dx:acc.x,dy:acc.y});acc={x:0,y:0};sending=false},40)}});
$('v').addEventListener('wheel',e=>{e.preventDefault();post('/api/zoom',{steps:e.deltaY>0?1:-1})},{passive:false});
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;if(e.code==='Space'){e.preventDefault();post('/api/pause')}else if('1234'.includes(e.key))post('/api/view',{key:e.key})});
async function tick(){try{const s=await (await fetch('/api/status')).json();
 const names={ready:'Ready',running:'Running',done:'Done ✓',failed:'Did not work out'};
 $('state').className='status '+s.state; $('state').textContent=(s.paused&&s.state==='running'?'Paused — ':'')+(names[s.state]||s.state)+(s.title?': '+s.title:'');
 $('badge').textContent=(s.paused&&s.state==='running')?'paused':s.state==='running'?s.title:s.state==='done'?'done':s.state==='failed'?'did not work out':'ready';
 $('log').textContent=s.log.join('\n');$('pause').textContent=s.paused?'▶ Resume':'⏸ Pause';$('speed').textContent=s.speed+'×';
 document.querySelectorAll('[data-v]').forEach(b=>{b.style.outline=b.dataset.v===s.view?'2px solid #5aa9ff':'none'});
 $('native').disabled=!s.native_ok||s.native_open; $('native').textContent=s.native_open?'MuJoCo window is open':s.native_ok?'Open MuJoCo window':'MuJoCo window unavailable here';
 s.tasks.forEach(x=>{const b=$(NAMES[x.name]); if(b) b.classList.toggle('active',s.state==='running'&&s.title===x.title)});}catch(e){}}
(async()=>{L=await (await fetch('/api/layout')).json(); rack=L.defaults.rack; item=L.defaults.item_xy; refreshGrids(); drawMap(); setInterval(tick,400); tick();})();
</script></body></html>"""


def make_handler(worker: Worker):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args) -> None:  # keep the terminal quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                self._send(200, PAGE.encode(), "text/html; charset=utf-8")
            elif self.path == "/api/status":
                self._send(200, json.dumps(worker.status()).encode(), "application/json")
            elif self.path == "/api/layout":
                self._send(200, json.dumps(layout_info()).encode(), "application/json")
            elif self.path in ("/stream", "/stream_wrist"):
                wrist = self.path == "/stream_wrist"
                worker.viewers += 1
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = -1
                try:
                    while not worker.stop:
                        with worker.frame_ready:
                            if wrist:
                                worker.frame_ready.wait_for(lambda: worker.wrist_id != last or worker.stop, timeout=1.0)
                                frame, last = worker.wrist_jpeg, worker.wrist_id
                            else:
                                worker.frame_ready.wait_for(lambda: worker.frame_id != last or worker.stop, timeout=1.0)
                                frame, last = worker.jpeg, worker.frame_id
                        if frame:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(frame))
                            self.wfile.write(frame + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    worker.viewers -= 1
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self) -> None:
            length = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(length) or b"{}") if length else {}
            if self.path == "/api/run" and body.get("name") in SCENARIOS:
                try:
                    params = Params.from_dict({**(body.get("params") or {}), "assist": body.get("assist", True)})
                    validate(body["name"], params)
                except (ParamError, ValueError, TypeError) as exc:
                    self._send(400, json.dumps({"error": str(exc)}).encode(), "application/json")
                    return
                worker.request(body["name"], params, bool(body.get("record")))
            elif self.path == "/api/native":
                worker.want_native = True
            elif self.path == "/api/reset":
                worker.request("__reset__", worker.params)
            elif self.path == "/api/pause":
                worker.toggle_pause()
            elif self.path == "/api/speed":
                worker.control.speed = max(0.1, min(8.0, worker.control.speed * float(body.get("f", 1.0))))
            elif self.path == "/api/view" and body.get("key") in VIEWS or body.get("key") in CAMERA_VIEWS:
                worker.set_view(body["key"])
            elif self.path == "/api/orbit":
                worker.orbit(float(body.get("dx", 0)), float(body.get("dy", 0)))
            elif self.path == "/api/zoom":
                worker.zoom(float(body.get("steps", 0)))
            else:
                self._send(404, b"{}", "application/json")
                return
            self._send(200, b"{}", "application/json")

    return Handler


def _relaunch_under_mjpython(argv: list[str]) -> None:
    """On macOS the native MuJoCo window needs `mjpython`; re-run ourselves under it (once)."""
    if sys.platform != "darwin" or _under_mjpython():
        return
    mjpython = Path(sys.executable).parent / "mjpython"
    if mjpython.exists():
        import os

        os.execv(str(mjpython), [str(mjpython), "-m", "dsr_sim.panel", *argv])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true", help="do not open the page automatically")
    parser.add_argument("--native", action="store_true", help="also open a native MuJoCo window mirroring the run (debugging)")
    parser.add_argument("--no-native", action="store_true", help=argparse.SUPPRESS)  # old flag; web only is the default now
    args = parser.parse_args(argv)
    args.no_native = args.no_native or not args.native
    if not args.no_native:
        _relaunch_under_mjpython(list(sys.argv[1:] if argv is None else argv))

    worker = Worker(args.scene)
    worker.want_native = not args.no_native
    server = None
    for port in range(args.port, args.port + 10):
        try:
            server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(worker))
            break
        except OSError:
            continue
    if server is None:
        print(f"[error] no free port near {args.port}", file=sys.stderr)
        return 1
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Control panel: {url}   (Ctrl+C here to quit)")
    if worker.want_native and worker.native_ok:
        print("Native window keys: 1 / 2 run a simulation, Space start or pause, R reset")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        worker.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        worker.stop = True
        if worker.native is not None:
            with contextlib.suppress(Exception):
                worker.native.close()
        server.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
