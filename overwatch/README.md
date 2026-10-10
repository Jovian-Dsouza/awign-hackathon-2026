# Overwatch

A separate monitoring-and-decision service for the dark-store robot. An **external,
fixed camera** (a USB camera, an RTSP stream, or existing CCTV footage) watches the
arena. When something goes wrong it raises an incident, a fast decision model judges
it, and the **admin approves** what the robot may do about it.

It is deliberately its own service: it does not import or modify UthaLe. It talks to
UthaLe through a webhook and to the Raspberry Pi through the Pi's existing HTTP API.

```
 USB cam / RTSP / video file
          │
   ┌──────▼───────┐   observations    ┌───────────────┐   decision    ┌──────────────┐
   │  Detectors   ├──────────────────►│  Decision     ├──────────────►│  Incidents   │
   │  health      │  (structured,     │  rules floor  │  (typed)      │  SQLite      │
   │  scene change│   never verdicts) │  + Laya       │               └──┬────────┬──┘
   └──────────────┘                   └───────────────┘                  │        │
                                                                admin UI │        │ webhook
                                                  approve / escalate /   │        ▼
                                                  dismiss  ◄─────────────┘     UthaLe backend
                                                       │
                                                       ▼
                                              Responder ──► Raspberry Pi (:8003)
                                              hold base · play recovery trajectory
                                                       │
                              every verdict ───────────┴──► training data for Laya
```

## Models

| Model | Role here | Status |
|---|---|---|
| **Laya** · <https://laya.convaiinnovations.com/> | Fast typed decisions over the camera's structured observations: *is it recoverable? how severe? what next?* Apache-2.0, open weights, ~33 ms/question on a GPU (author's figure). [GitHub](https://github.com/NandhaKishorM/laya) · [Hugging Face](https://huggingface.co/convaiinnovations/laya) · [typed-decisions checkpoint](https://huggingface.co/convaiinnovations/laya-typed-decisions) · [PyPI](https://pypi.org/project/laya/) | **Installed and running** (`uv sync --extra laya`; weights were already in `~/.cache/huggingface`). Measured here on an M2 Pro (MPS): loads in ~11 s, ~85-125 ms per decision once warm (first call slower). |
| **JEV** · <https://jevmodel.org/> | Same idea as Laya (typed choice / score / probability), hosted by TypeSafe AI. API key required, per-token pricing; no weights or licence published on that page. The site calls itself an independent guide, not TypeSafe's own. | **Not wired.** No key, and its API docs were not read. It would be one more class with `decide(observation, ctx) -> Decision` in `decision/`. |

**Laya cannot see.** It reads text and JSON only. The camera side turns frames into
structured observations (`area_pct`, `solidity`, `stationary_s`, zone…) and Laya
decides on those. Its own docs say zero-shot accuracy on typed decisions is close to
random, so:

1. Rules (`decision/rules.py`) are the **safety floor** and always decide in `shadow` mode.
2. Laya answers alongside them and is scored against the admin's verdicts
   (`GET /api/feedback/summary`).
3. In `advisory` mode the more cautious of the two wins. A model can add caution,
   never remove it: "ignore" from Laya cannot hide something the rules flagged.
4. Fine-tune Laya on the exported verdicts, watch the agreement number, then decide
   how much authority it has earned.

## Quick start

```bash
cd overwatch
uv sync --extra laya            # opencv, fastapi, torch + laya (omit --extra laya to run rules-only)
uv run overwatch --list-cameras # e.g.  0 FaceTime HD Camera   1 HD camera
# put the camera's EXACT name in overwatch/.env (see below), then:
uv run overwatch                # http://127.0.0.1:8010
```

`.env` (gitignored) holds your settings; the camera is chosen **by exact device name**:

```
OVERWATCH_SOURCE=name:HD camera
OVERWATCH_WIDTH=640
OVERWATCH_HEIGHT=480
OVERWATCH_LAYA=shadow
```

**There is deliberately no default camera.** On macOS the device numbering differs between
ffmpeg and OpenCV and changes between runs, and one real run opened the laptop's own camera
by index. So a name is the only selector: it is captured through `ffmpeg` by name (macOS) or
`/dev/videoN` found via sysfs (Linux). A name that matches nothing, or matches two cameras
(`HD` matches both `FaceTime HD Camera` and `HD camera`), is **refused**, never substituted.
Other sources:

```bash
uv run overwatch --source "rtsp://user:pass@10.0.0.5/stream1"   # a store's CCTV
uv run overwatch --source ./footage/aisle3.mp4                  # recorded footage, looped
uv run overwatch --source remote                                # frames pushed by camera-agent/
```

On macOS the terminal needs camera permission, and the camera must not be open in another app.

### Set up the scene (do this once per camera position)

1. **Fix the camera** (tripod, clamp). It must not move after the next step.
2. Open the page, tidy the arena, take the robot out of view, press
   **Capture clean reference**.
3. Optional: drag **watch zones** (e.g. `floor`, `tote`) and **ignore zones** (the dock,
   a screen). Without zones the whole frame is watched.
4. Optional: stick an ArUco tag (`DICT_4X4_50`) on the robot and set
   `OVERWATCH_ROBOT_MARKER_ID` so the robot itself is never read as an object.

Do not capture the reference while something is already on the floor: it becomes
part of the "clean" scene.

## What it detects today

| Observation | How | Notes |
|---|---|---|
| `foreign_object` | A region differs from the reference (colour-aware, exposure-compensated) **and sits still** for `PERSIST_S` (4 s). Reported once per object; closed as "no longer visible" when it goes away. | Reports geometry only. It does not know *what* the object is. |
| `camera_fault: scene_shift` | Global shift or poor match against the reference (phase correlation), or most of the scene changed at once. Detection pauses and says why. | Fixes a real failure seen with a hand-held camera: see below. |
| `camera_fault: black / frozen / no_scene_detail` | Image statistics held for 5 s. | `no_scene_detail` = lens covered, too close, or pointed at a blank wall. A monitor that cannot see must say so. |

Moving things (robot, hands) restart the stillness clock and are never reported.

> **Known limit:** it assumes a **fixed** camera. During first live testing the camera was
> being handled, and the first version logged 13 "objects": every edge that moved
> relative to the reference looked like a new object that had sat still. The camera-moved
> check above is the fix, with regression tests. It was verified live (the same movement
> now produces one `scene_shift` alert), but **not yet against a real dropped item**,
> because the camera was never held still at the arena.

## Integration

```
                                   ┌────────────────────────── same machine ───────────────────────────┐
 camera ─ USB ─► camera-agent ─ws─►│ UthaLe backend :8000  ──relay──►  Overwatch :8010 ─► Laya          │
 (any device)    (on that device)  │   /api/monitor/ingest              /api/ingest      + rules        │
                                   │   /api/monitor/stream  ◄── proxy ── /stream                         │
 browser ───────── dashboard ─────►│   /api/monitor/*       ◄── proxy ── status, incidents, verdicts     │
                                   │   activity feed        ◄─ webhook ─ one call per incident          │
                                   └────────────────────────────────────────────────────────────────────┘
```

| Peer | How | Config |
|---|---|---|
| **Dashboard** | "External monitor" panel on the *Live robot* page: live annotated feed, snapshots, approve / needs a person / false alarm. Everything goes through the UthaLe backend, so one ngrok tunnel is enough. | `UthaLe/backend/app/api/monitor.py`, `MonitorFeed.tsx` |
| **UthaLe activity feed** | One webhook per new incident → an entry of type `monitor`. | `OVERWATCH_WEBHOOK_URL`, `OVERWATCH_WEBHOOK_TOKEN` = UthaLe's `MONITOR_INGEST_TOKEN` |
| **Remote camera** | `camera-agent/` pushes JPEGs to `/api/ingest` here, or to UthaLe's `/api/monitor/ingest` which relays them. | `OVERWATCH_SOURCE=remote`, `OVERWATCH_INGEST_TOKEN` = UthaLe's `MONITOR_INGEST_TOKEN` |
| **Raspberry Pi** | `firmware/combined-web/web_combined.py` `:8003`: `POST /drive` zeros (hold base), `POST /play {name}` (recorded arm trajectory), `GET /status` | `OVERWATCH_RESPONDER=pi`, `OVERWATCH_PI_URL`, `OVERWATCH_PI_RECOVER_TRAJ` |

One shared secret (`MONITOR_INGEST_TOKEN` in UthaLe = `OVERWATCH_INGEST_TOKEN` =
`OVERWATCH_WEBHOOK_TOKEN` here) covers the relay, the ingest and the webhook. Generate it with
`python3 -c "import secrets; print(secrets.token_urlsafe(24))"`. With it unset, all three are refused.

### Where to run what

| Situation | Run |
|---|---|
| Camera on the same laptop as everything | just `uv run overwatch` (local source). No agent, no token. |
| Camera on another device, same network | on that device: `camera-agent --server http://<laptop>:8010 --token … --source "name:…"` |
| Camera on another device, other network | `ngrok http 8000` on the laptop (the tunnel the dashboard already uses); on the device: `camera-agent --server https://….ngrok-free.app --via-uthale --token … --source "name:…"` |
| A store's existing CCTV | `OVERWATCH_SOURCE=rtsp://…` (or a recorded file) |

Check the agent's logged MB/hour against your tunnel's data limits before relying on a tunnel
(`camera-agent --dry-run 10` measures it without a server).

What the responder can and cannot do, plainly:

- **Hold** stops the wheels only. It does not freeze the arm and is **not an emergency stop**.
- **Recover** plays one *fixed* recorded motion. It cannot reach an arbitrary dropped
  item. With no trajectory configured it refuses rather than pretending.
- Nothing moves the robot without an admin click, except `OVERWATCH_AUTO_HOLD=1`,
  which only stops the base.

## Configuration

All `OVERWATCH_*`, from the environment or a `.env` in the working directory.

| Variable | Default | |
|---|---|---|
| `SOURCE` | *(none: required)* | `name:<exact device name>`, `index:N` (unstable on macOS), RTSP URL, file path, or `remote` |
| `INGEST_TOKEN` | – | shared secret a camera agent must present (needed for `SOURCE=remote`) |
| `WIDTH` / `HEIGHT` | 1280 / 960 | requested; a size the camera lacks falls back to its default (the `HD camera` offers only 320×240 and 640×480) |
| `ANALYZE_FPS` | 6 | detector + annotated stream rate |
| `PERSIST_S` | 4 | seconds an object must sit still before it is reported |
| `DIFF_THRESHOLD` / `MIN_AREA_FRAC` / `MAX_AREA_FRAC` | 28 / 0.0015 / 0.45 | change sensitivity |
| `ROBOT_MARKER_ID` | -1 (off) | ArUco id on the robot |
| `LAYA` | `off` | `off` · `shadow` · `advisory` |
| `LAYA_MODEL` / `LAYA_PATH` | `typed-decisions` / – | checkpoint name, or a fine-tuned directory |
| `RESPONDER` | `null` | `null` (dry run) · `pi` |
| `AUTO_HOLD` | 0 | stop the base on any camera-confirmed incident |
| `WEBHOOK_URL` / `WEBHOOK_TOKEN` | – | outbound alert |
| `HOST` / `PORT` / `DATA_DIR` | 127.0.0.1 / 8010 / `./data` | `data/` holds the DB, reference, zones, snapshots |

## API

`GET /` page · `GET /stream` annotated MJPEG · `GET /api/frame.jpg` ·
`GET /api/status` · `POST|DELETE /api/reference` · `GET|PUT /api/zones` ·
`GET /api/incidents[?status=open]` · `GET /api/incidents/{id}` ·
`GET /api/incidents/{id}/snapshot` · `POST /api/incidents/{id}/verdict`
`{verdict: approve_recovery|escalate|dismiss, severity?, note?}` ·
`POST /api/robot/hold` · `GET /api/feedback/summary` · `GET /api/feedback.jsonl`

## The feedback loop (how the system improves)

Every admin verdict is a label. `GET /api/feedback.jsonl` exports rows in Laya's own
fine-tuning format, `{"state", "questions", "expected"}`, using the same three
questions Laya is asked at inference (`decision/questions.py`), so training and use
cannot drift apart. Fine-tuning goes through `laya.train.finetune` (Laya's docs point
to a Kaggle 2×T4 notebook and an Apple-Silicon script). Do not start a long training
run on the Mac without deciding that deliberately.

A dismissed false alarm labels `action=ignore` and `severity=harmless` but
deliberately leaves `recoverable` unlabelled, since it says nothing about that.

## Layout

```
src/overwatch/
  camera.py            sources: exact-name capture, RTSP/file, RemoteFrameSource (agent ingest)
  detectors/           health.py · scene_change.py · robot_marker.py
  events.py            Observation, Decision
  decision/            rules.py (floor) · laya_decider.py · questions.py · combine()
  responders/          base.py (dry run) · pi.py
  notifiers.py         webhook
  store.py             incidents, verdicts, training export, agreement
  pipeline.py          frames → detectors → decisions → incidents
  app.py · static/     API and admin page
tests/                 45 tests: synthetic scenes, decisions, camera selection, API, ingest over a real socket
```

```bash
uv run pytest
```

## Verified vs not

| Verified | Not verified |
|---|---|
| New `HD camera` opens by exact name and streams 640×480 at ~16 fps; Laya loads and answers | Detecting a real dropped item with the camera fixed at the arena |
| agent → UthaLe relay → Overwatch → incident → webhook → activity feed → dashboard panel, end to end (with a generated video as the camera) | The Pi responder against the real Pi |
| Proxy allow-list, path-traversal attempts, token refusal, second-sender refusal | Dashboard behaviour through a real ngrok tunnel |

**Laya's zero-shot answers do not separate cases yet**: a bottle-sized item, a spill-like patch
and a camera fault all came back "escalate" at ~45 %, P(recoverable) ≈ 0.3. That is the
caution its docs warn about, so it stays in shadow mode until it has been fine-tuned on
admin verdicts. Its library also warns that this checkpoint ships calibration values outside
the valid range for some entries, so treat its confidence numbers as uncalibrated.

## Roadmap / not built

- **Where is it?** Pixel → table coordinates (4 ArUco markers + homography), so an
  incident carries a position the robot can drive to. Today `table_xy` is always `None`.
- **Real recovery.** Today only a fixed trajectory. Targeted recovery means feeding that
  position to the team's ACT pick pipeline. Deformable items (plastic bags) are unlikely
  to work with the SO-101 gripper; rigid items are the realistic class.
- **What is it?** Telling a bottle from a spill needs a vision model or a labelled clip
  set. The rules use geometry only and err toward "escalate".
- **JEV adapter** once there is a key and its API docs.
- **UthaLe endpoint** to receive the webhook and show incidents in its own dashboard.
