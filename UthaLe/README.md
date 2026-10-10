# UthaLe

**From Shelf to Dispatch. Intelligently.**

UthaLe is an operations platform for robotic picking in quick-commerce dark stores. A
mobile manipulator locates products, picks them, carries them to the packing
station and places them in the order's dispatch tote. The dashboard shows the
live USB camera, robot state, orders, task execution and verification (QC).

This is a hackathon prototype. Here is what's real and what isn't:

| Part | Status |
|---|---|
| USB camera feed | **Real.** OpenCV captures the device and serves it as MJPEG. |
| Orders, tasks, QC records, activity | **Real records**, persisted in SQLite. They survive restarts. |
| Product catalog | **Real** SKUs and shelf slots from the MuJoCo warehouse's `config/inventory.yaml`. |
| Robot execution | **Simulated.** `SimulatedRobotAdapter` waits out plausible step durations. No hardware is commanded. |
| NFC tag reader, load cell | **Simulated**, labelled `SIM` everywhere. Nominal item weights are illustrative. |
| Battery telemetry | **Not available**, shown as "No telemetry" and never invented. |
| "Software stop" button | Halts the simulated mission only. **It is not an emergency stop.** |

## Run it

Prerequisites: Python 3.12+ and Node 20+.

```bash
# Backend: http://127.0.0.1:8000 (API docs at /docs)
cd UthaLe/backend
python -m venv .venv
.venv\Scripts\activate            # macOS/Linux: source .venv/bin/activate
pip install -r requirements.txt
python -m app

# Frontend: http://localhost:5173 (second terminal)
cd UthaLe/frontend
npm install
npm run dev
```

The Vite dev server proxies `/api` to the backend, so the browser talks to one
origin (the camera stream included). Configuration is through environment
variables or `backend/.env`. See [`.env.example`](.env.example).

Tests: `cd backend && .venv\Scripts\python -m pytest` covers the QC rules, the full
order-to-dispatch workflow, anomaly review, stop/reset and restart persistence.
Tests never open the real camera.

## Demo walkthrough

1. **Overview**: the live camera, robot status (marked `DEMO / SIM`), KPIs, active orders and activity.
2. **New order** (or **Demo order**): choose SKUs, a dispatch bay and a priority. The order is assigned to `SIM-STRETCH-01` and starts automatically.
3. Open the order: watch it move from queued to picking, picked, verification and dispatched. Each line runs *navigate, identify, pick, transport, place, verify*. Click any task to see its execution history.
4. **Live Robot**: pause, resume, software stop and reset. Under *Inject simulated anomaly*, arm e.g. "Wrong item placed" before the next line.
5. **QC & Verification**: the flagged record shows the expected and observed values, the simulated sensor readings and a **real camera frame** captured at verification. Approve it (and the order dispatches) or reject it (and the order goes to *exception*).
6. **Analytics**: charts computed from the stored records.
7. Restart the backend. Orders, tasks and QC records are still there, and a step that was mid-flight is re-run.

`SIM_SPEED=3` makes the simulated robot faster for short demos.

## Architecture

```
UthaLe/
  backend/app/
    main.py              FastAPI wiring, lifespan (camera start/release, workflow loop)
    __main__.py          `python -m app` (uvicorn with graceful-shutdown timeout)
    config.py            env-driven settings
    database.py          SQLite schema + connection
    repository.py        all SQL (swap here for PostgreSQL/Supabase)
    api/camera.py        /api/camera/{status,stream,start,stop,config}
    api/operations.py    orders, tasks, qc, robot, activity, overview, analytics, settings
    services/camera.py   one capture thread, shared JPEG buffer, reconnect with backoff
    services/robot.py    RobotAdapter interface + SimulatedRobotAdapter
    services/sensors.py  SensorHub interface + simulated NFC/load cell
    services/qc.py       deterministic verification rules
    services/workflow.py order -> tasks, execution loop, QC records, human review
    services/analytics.py
  frontend/src/
    pages/               Overview, LiveRobot, Orders, OrderDetail, Tasks, Qc, Analytics, Settings
    components/          CameraFeed, RobotStatusCard, RobotControls, QcCard, TaskList, ...
    lib/api.ts           typed API client
```

### Workflow

Creating an order creates 6 tasks per line plus a final `dispatch` task. A
background loop takes the next assigned order (in-progress orders first, then
by priority, then FIFO) and runs each step through the `RobotAdapter`. Pick
failures are retried once. If the pick still fails, the line's transport and
place steps are skipped and QC records a FAIL.

### QC rules (`services/qc.py`)

A robot reporting "placed" only means its motion finished. A record **PASS**es
only when independent evidence agrees: the NFC tag matches the SKU *and* the
load-cell delta is within ±10% (minimum ±25 g) of the expected weight. Missing
or ambiguous evidence gives **REVIEW**. Contradicting evidence (wrong tag, an
empty tote, a failed pick, the wrong bay) gives **FAIL**. With no sensors at
all, every placement goes to REVIEW. FAIL and REVIEW need a signed human
decision. Approving every flagged line dispatches the order. Rejecting any line
puts the order in `exception`.

### Integrating real hardware

- **Robot**: implement `RobotAdapter` (`execute`, `pause`/`resume`/`stop`/`reset`,
  `telemetry`) on top of the Stretch SDK or ROS 2 actions, or on top of the MuJoCo
  `qc_warehouse` simulator in the repository root. Select it in `build_services()`.
  Wire `stop()` only to a vetted stop path. The dashboard button never replaces
  the robot's physical E-stop.
- **Sensors**: implement `SensorHub.read_placement()` for the real NFC reader and
  load cell. Return `source: "hardware"` so the UI drops the SIM labels.

## Camera troubleshooting

- **Find the index.** Run this with the backend stopped, or after clicking *Stop & release*:
  ```bash
  cd backend
  .venv\Scripts\python -c "import cv2;[print(i, cv2.VideoCapture(i, cv2.CAP_DSHOW).read()[0]) for i in range(5)]"
  ```
  You can also change the index live in **Settings** and watch the feed.
- **Indices shift.** On the dev machine, the LG Smart Cam is index 0 when plugged
  in. OBS Virtual Camera then moves to index 1, but takes index 0 when the USB camera is
  absent. If you see the grey OBS "camera inactive" card, you're on the virtual camera.
- **"Could not open camera"**: another app (Teams, Zoom, OBS, a browser tab) holds
  the device; close it. Then check Windows *Settings → Privacy & security → Camera →
  Let desktop apps access your camera*. On macOS, grant camera access to the terminal app.
- **Slow open or black frames on Windows**: keep `CAMERA_BACKEND=auto` (DirectShow).
  MSMF fails to open some USB cameras.
- **Unplugged mid-stream**: the service releases the device and retries with
  backoff (1, 2, 4, 8 s). The dashboard shows *No signal* with guidance and never
  shows fake footage. The feed resumes on its own when the camera returns.
- The backend opens the device **once**; every browser tab shares that capture.
  `Ctrl+C` releases the camera within about 2 s, even with streams open.
