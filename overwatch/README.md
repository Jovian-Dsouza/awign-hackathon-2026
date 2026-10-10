# Overwatch

One job: watch the robot with an external camera, and when it **drops something or spills
something**, raise an incident and say whether the robot can fix it.

```
camera ─► is the robot in view and working? ─no─► do nothing
                       │ yes
                       ▼
          picture of the table (baseline)
                       │
     something NEW appears and stays put?
         small, compact ─► DROPPED ITEM ─► "the robot can pick it up"   (admin: Ask the robot)
         large, irregular ─► SPILL ──────► "a person needs to clean it"
                       │
              Laya: second opinion (real? robot can fix it?)
                       ▼
          dashboard: External monitor panel
```

Nothing else is an incident. Not people, not shadows, not you adjusting the camera. Those are
counted (`ignored_changes` in `/api/status`) so you can see why nothing fired.

## Run it

```bash
cd overwatch
uv sync --extra laya          # first time (Laya weights are already in ~/.cache/huggingface)
uv run overwatch --list-cameras
uv run overwatch              # http://127.0.0.1:8010, and in the dashboard's "External monitor" panel
```

`overwatch/.env` already has the camera (`name:HD camera`, selected by its exact name) and
`OVERWATCH_ROTATE=180` (the camera is mounted upside down: the picture is turned upright here only,
never in UthaLe's own "Live camera" panel).

## Tell it where the robot is

| Setup | What it does |
|---|---|
| **Print `robot_tag.png`** (about 5 cm), stick it flat on top of the robot, set `OVERWATCH_ROBOT_MARKER_ID=0` | Best. Knows exactly when the robot is in view, ignores the robot's whole body, and can find **spills**. |
| No tag (default) | "A robot-sized thing is moving" means the robot is working. A person walking by looks the same, and **spills are not detected** (a parked robot would look like one). |
| Also optional | `OVERWATCH_UTHALE_URL`, `OVERWATCH_PI_URL`, `OVERWATCH_ACT_URL`: the robot's own "I'm picking" signals. |

In the picture, draw **ignore zones** over the rack and the tray (items legitimately change there)
and a **watch zone** over the table: `PUT /api/zones` or the zone buttons on `:8010`.

## What Laya does

Laya reads text, so it is given a plain sentence describing the thing ("a tiny object appeared on the
table and stopped moving... compact and solid, sharp edges...") and answers two yes/no questions:

1. **real_drop**: did the robot really drop something, or is this a false alarm?
2. **recoverable**: can the arm safely pick it up (a packet) or not (a spill)?

Measured on this Mac: about 100 ms per decision. With raw numbers it could not tell cases apart
(AUC 0.32); with the sentence it ranks real drops above false alarms (AUC 0.90, on cases built from
the sim frames). Its probabilities are squeezed into 0.36-0.48, so **it is a second opinion, not the
filter**: `OVERWATCH_LAYA=shadow` (default) records it; `filter` lets it hide a drop only when it is
very sure (P(real) < 0.25). The deterministic rules and the size/shape/position gates are what keep
the spam out.

## Dashboard

UthaLe's *Live robot* page has an **External monitor** panel (live annotated picture, incidents with
a snapshot, buttons: *Ask the robot to pick it up* / *Needs a person* / *False alarm*). *Test: robot
picking* is only for trying it without the robot. The "Live camera" panel is UthaLe's own camera and is
not connected to this.

## Not done / not verified

- A real dropped item with the camera fixed on the real arena. Tested on the team's sim frames
  (successful pick = 0 incidents; item on the table = 1) and synthetic scenes (76 tests).
- Spill detection needs the tag and has only been tested on synthetic patches.
- The Pi responder ("ask the robot") is a dry run until `OVERWATCH_RESPONDER=pi` and a recorded
  trajectory are set; targeted recovery of an arbitrary position is not built.
- Camera trouble (covered lens, moved camera) is a note in the panel, never an incident.

Also here: `camera-agent/` (stream a camera from another device), JEV is not wired (needs an API key).
