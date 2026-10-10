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
| Options | *Record data* (writes the run to `../data/`) |
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

## How items are held (no tricks)

Items are held by **contact and friction only**. There is no weld, no "ghost" jaws and no switch to make a grasp
easier: the jaws collide with the item, the item has a real mass (`mass_kg` in `config/inventory.yaml`, 60 g can,
60 g bottle, 100 g milk) and the gripper squeezes at a capped torque (0.8 N·m, like LeRobot's gripper current
limit) instead of the servo's full stall torque.

What that took:

- **Collision shapes.** MuJoCo collides a mesh as one convex hull, which filled the gap between the jaws and made
  the folded arm lie inside itself. `scripts/decompose_collision.py` splits the SO-101 meshes into convex pieces
  (CoACD) in `assets/so101/collision/`; `robot._convex_collision` uses them.
- **Stowed pose.** At the firmware REST_POSE the CAD parts of 7 link pairs overlap by 1-2 cm (the elbow sits at its
  limit); only those pairs have contact switched off (`robot.STOWED_OVERLAPS`). Everything else is real contact.
- **Grasp heights.** Fingers are +-7 mm thick and the gripper body is 24 mm below them, so the jaw axis stays
  >= 32 mm above the surface it picks from, and lower than the board above minus the camera when lifting out.

Honest limits you will see in the sim:

| Item | Result |
|---|---|
| soup can, water bottle, milk carton | picked from their racks and carried to the tray by friction |
| black tape (14 mm), box cutter (9 mm) | **fail**: lying flat, they are thinner than the fingers plus the body clearance, and the run says so |

Contacts are still soft (about 1 mm penetration on the item) and items are rigid, so this is a better upper bound
than a weld, not a measurement of real pad grip.

## What is and is not modelled

| Modelled | Not (yet) |
|---|---|
| Holonomic base with mecanum wheel IK and spinning wheels | Mecanum roller slip / wheel-floor contact |
| SO-101 kinematics, joint limits, servo-like position actuators | Servo backlash, real torque curves |
| Wrist camera rendered from its mount | Real camera intrinsics, exposure, USB latency |
| Scripted pick / carry / place with sideways correction and route planning | SLAM, a learned grasp policy |
| Friction-only grasping with item mass and a torque-limited gripper | Packaging deformation (items are rigid), soft pads |

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
| `pick.py` | task machinery: stepping, drive, IK, reach planning |
| `panel.py` | web page + native window worker |
| `datalog.py` / `collect.py` | recording, calibration export, bulk episode generation |
| `robot.py`, `control.py` | robot model from the exported parts, command API |
| `shelf.py`, `station.py`, `room.py`, `items.py`, `config.py` | scene builders (reused) |


## Wrist camera

The wrist camera is the bar webcam from the real build: a printed post glued to the gripper top, a bracket platform with two cheeks, the bar camera with lens ring, and a thin USB lead (`WRIST_CAM_*` in `robot.py`; same numbers in `frontend/src/components/Arm.tsx`). The sim mounts it on the gripper's +y side (top of the gripper at the grasp roll of -90, like the real arm which reads about -92 when picking).

The page shows two live panels: the **environment** (drag to orbit, scroll to zoom, preset views Overview / Robot / Shelf / Tray) and the **wrist camera** (the robot's only camera, 640x480, what the policy sees). **Full screen** (button or key F) hides the controls: the environment fills the whole screen with the wrist camera as a picture-in-picture in the top-right corner. Esc or the faded bar at the bottom leaves it (Pause / Reset stay on that bar).

Safety check: `tests/test_scenarios.py::test_wrist_camera_never_rotates_underneath` fails if the camera ever swings under the gripper or dips below the table surface during a run.

## Objects on the shelf

Five objects only (`config/inventory.yaml`): **water_bottle** (300 ml PET, green cap), **black_tape** (electrical tape roll), **box_cutter** (blue snap-off cutter), plus the milk carton and the soup can (`alphabet_soup`). The three new ones are built from primitives in `assets/custom/` at real size and scaled to fit the racks and the gripper. The scripted pick/place is tuned for the soup can; the other four are shelf stock the robot works around, not yet pick targets.

## Collisions

`test_robot_does_not_pass_through_shelf_or_tray` measures every arm, gripper and camera part against the shelf boards and the tray during a shelf-to-tray run and a restock run and fails if anything sinks in by more than 5 mm. Two things were changed to make it pass: the shelf levels are 18 cm apart (the bar camera sits about 9 cm above the jaw axis), and the item is let go 5 cm above the tray floor so the gripper body stays over the near wall. The camera and bracket are still visual-only geoms (they do not push on anything), which is why this is measured rather than simulated.
