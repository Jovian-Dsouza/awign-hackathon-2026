# Dark-store robot simulation (MuJoCo)

Simulation of **our** robot in a miniature quick-commerce dark store:

- **Rover:** 4-wheel mecanum chassis (holonomic planar drive, wheels spin from mecanum IK). The geometry is
  exported from the website model (`frontend/src/lib/robotModel.ts`), so the sim and the site show the identical robot
- **Arm:** SO-101 (stock MJCF from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), position actuators)
- **Compute:** Raspberry Pi (visual only)
- **Cameras:** `wrist_cam` on the gripper, `chassis_cam` on the front of the chassis
- **Scene:** a flat table (no walls), a 3×3 cardboard shelf, a drop tray and the rover's dock

Reference: the Stretch 3 simulation in `../simulation/` (SLAM, navigation, grasping). Scene builders are reused
from it and scaled to our robot; the robot, control, simulations, web panel and data logging are new.

## Run

```bash
uv sync
uv run python scripts/fetch_assets.py   # grocery item models into assets/third_party/ (pinned commits)
uv run dsr-build                        # regenerate assets/scene.xml from config/
uv run dsr-panel                        # the control page + the native MuJoCo window, same run
uv run pytest
```

`uv run dsr-panel` opens **http://127.0.0.1:8765** and, on the same simulation, the native MuJoCo window.
Everything is controlled from the page:

| On the page | Does |
|---|---|
| **1 · Shelf to tray** — rack grid, ▶ Run | picks the item from the chosen rack (level 0–2, slot 0–2) and carries it to the tray |
| **2 · Restock a misplaced item** — click the table map, choose a destination rack, ▶ Run | the item has been left there; the rover works out an approach direction, picks it up and puts it in a rack |
| Options | *Grasp assist* (default on) and *Record data* (writes the run to `../data/`) |
| Pause / Reset / speed / camera buttons | as named; drag the picture to orbit, scroll to zoom |
| **Open MuJoCo window** | brings the native window back if you closed it |

Every run resets to the start (rover on the dock, arm folded, item in its rack/at the chosen spot), does the
task, and drives back to the start. Starting a run while another is going aborts it and starts the new one.
Spots the rover cannot stand off from (near the tray, table edges) are refused with a reason.

Native-window keys: `1` / `2` run a simulation, `Space` start/pause, `R` reset.

## Simulations

| # | Name | What happens |
|---|---|---|
| 1 | `shelf_to_tray` | Pick the item out of the rack you choose and carry it across the table to the drop tray. |
| 2 | `restock_misplaced` | The item was left somewhere on the table instead of its rack: pick it up and put it back in a rack. |

Command line: `uv run dsr-run --list`, `uv run dsr-run --task 2 --item-xy 0.5 -0.2 --dest-rack 1 2 --headless`,
`uv run dsr-run --task 1 --rack 2 0 --headless`. Add `--record` to save the run.

## Data

`uv run dsr-collect --task restock_misplaced --episodes 20` generates many recorded episodes (random item
positions); `--task shelf_to_tray --sweep` visits every rack. Everything goes to `../data/` — see
[`../data/README.md`](../data/README.md) for the layout and, importantly, what carries over to the real robot
and what does not. `../data/tools/export_lerobot.py` converts the episodes to a LeRobot dataset for ACT.

## Grasp assist (important)

Holding a 5 cm can by friction with this gripper is marginal. With **Grasp assist** on (the default) the jaws
close around the item, the sim checks they are really around it, then a weld constraint carries it and the jaws
ignore the item's collision while it is carried, so a released item is never dragged out. A successful run
therefore shows the *motion plan* is feasible; it does **not** show that a friction grip would hold.
With assist off the full contact physics is used and it works much less often: in the recorded set below,
**3/9 racks** and **6/10 random table positions** (against 9/9 and 14/15 with assist on). Compare those runs
(not the assisted ones) with hardware when judging grip.

## What is and is not modelled

| Modelled | Not (yet) |
|---|---|
| Holonomic base with mecanum wheel IK and spinning wheels | Mecanum roller slip / wheel-floor contact |
| SO-101 kinematics, joint limits, servo-like position actuators | Servo backlash, real torque curves |
| Wrist camera rendered from its mount | Real camera intrinsics, exposure, USB latency |
| Scripted pick / carry / place with sideways correction and route planning | SLAM, a learned grasp policy |
| Friction-only grasp mode (assist off) | Packaging deformation (the item is a rigid can) |

Dimensions are approximate (from photos). The robot geometry lives in the frontend (`robotModel.ts`); edit it
there and re-export, not in `robot.py`:

```bash
(cd ../frontend && node scripts/export-robot.ts ../robot-sim/assets/robot)
```

## Code map

| File | Role |
|---|---|
| `scenarios.py` | the simulations (setup + scripted run), grasp / place skills |
| `world.py` | table facts, parameters + validation, rover footprint/route checks |
| `pick.py` | task machinery: stepping, drive, IK, reach planning, grasp assist |
| `panel.py` | web page + native window worker |
| `datalog.py` / `collect.py` | recording, calibration export, bulk episode generation |
| `robot.py`, `control.py` | robot model from the exported parts, command API |
| `shelf.py`, `station.py`, `room.py`, `items.py`, `config.py` | scene builders (reused) |


## Wrist camera

The wrist camera is the bar webcam from the real build: a printed post glued to the gripper top, a bracket platform with two cheeks, the bar camera with lens ring, and a thin USB lead (`WRIST_CAM_*` in `robot.py`; same numbers in `frontend/src/components/Arm.tsx`). The sim mounts it on the gripper's +y side (top of the gripper at the grasp roll of -90, like the real arm which reads about -92 when picking).

The page shows two live panels: the **environment** (drag to orbit, scroll to zoom, preset views Overview / Robot / Shelf / Tray) and the **wrist camera** (the robot's only camera, 640x480, what the policy sees). **Full screen** (button or key F) hides the controls: the environment fills the whole screen with the wrist camera as a picture-in-picture in the top-right corner. Esc or the faded bar at the bottom leaves it (Pause / Reset stay on that bar).

Safety check: `tests/test_scenarios.py::test_wrist_camera_never_rotates_underneath` fails if the camera ever swings under the gripper or dips below the table surface during a run.
