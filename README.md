# Awign Hackathon 2026

A MuJoCo simulation of a quick-commerce "dark store": **3 shelving units**
stocked with grocery items, a separate **packing / dispatch station** with
delivery totes, and a **Hello Robot Stretch 3** that fulfils customer orders on
its own. For each ordered item it navigates to the shelf, finds the item with its
head RGB-D camera, chooses a grasp, picks the item, carries it to the station and
packs it into that order's tote.

The robot knows where it is only from its own sensors: wheel encoders, the base
IMU and the base 2-D lidar feed **ROS 2 slam_toolbox** (running in Docker), which
builds the map from scratch on every run and localizes the robot in it. The
simulator's ground truth is only used to grade the result.

The default 2-order run (5 items) packs 5/5 on SLAM in about 14 minutes of
simulated time (about 2–2.5 minutes of wall-clock time), with a mean
localization error of about 2 cm against ground truth.

```
           north wall
 ┌────────────────────────────────────────┐
 │   [Shelf A]     [Shelf B]     [Shelf C] │  y ≈ +3.65
 │  ─────────── yellow safety line ─────── │
 │                  ║   ║ green walkway    │
 │  [dock]      [ packing station ]        │  y ≈ -1.6
 │                                         │
 └────────────────────────────────────────┘
```

## Setup

```bash
uv sync
uv run python scripts/fetch_assets.py   # item meshes + Stretch 3 into assets/third_party/ (pinned commits)
uv run qcw-build                        # regenerates assets/scene.xml from config/
./scripts/setup_mjpython.sh             # macOS only, once: lets mjpython run inside the uv venv
uv run qcw-slam up                      # builds + starts slam_toolbox and rosbridge in Docker (first build ~2 min)
```

SLAM needs Docker (Docker Desktop on macOS). `uv run qcw-slam status|logs|down`
manage the service. rosbridge has no authentication; it listens on 127.0.0.1
only, but any local process (or a web page in your browser) could still connect
to it, so run `uv run qcw-slam down` when you're done simulating. Without Docker, add `--localization ground-truth` to
`qcw-pick` to run on the simulator's pose and scene map instead.

## Run the robot

```bash
uv run qcw-pick                                     # headless: fulfil config/orders.yaml, print a report
uv run mjpython -m qc_warehouse.pick --viewer       # watch it live (add --realtime for wall-clock pace)
uv run qcw-pick --orders my_orders.yaml             # your own orders
uv run qcw-pick --localization ground-truth         # baseline without SLAM
```

Every `qcw-pick` run starts a fresh SLAM session (the container is recreated so
the map starts empty; `--no-slam-restart` reuses the running one). Besides the
fulfilment report it prints the localization error versus ground truth and
writes `renders/slam_map.png`: the SLAM map with the estimated (red) and true
(green) paths.

Orders live in `config/orders.yaml`:

```yaml
orders:
  - id: ORD-1001
    items: [ketchup, cookies, milk]   # -> tote_0
  - id: ORD-1002
    items: [tomato_soup, crackers]    # -> tote_1
```

Each order is packed into its own tote, so there can be at most as many orders
as totes. Every SKU has one unit in stock. The run writes
`renders/pick_report.json` and final snapshots, and exits with code 0 only if
every item was packed. The report JSON has the per-item results under
`orders` and the SLAM statistics under `localization`.

## Other commands

```bash
uv run qcw-view            # interactive viewer of the static scene
uv run qcw-render          # headless PNGs from every camera -> renders/
uv run pytest -m "not slow"  # fast tests (~15 s; live-SLAM tests run only if `qcw-slam up`)
uv run pytest -m "not ros"   # everything that works without Docker
uv run pytest              # includes end-to-end order runs on ground truth and on SLAM (~2 min)
```

## How the robot works

| Step | What happens | Code |
|---|---|---|
| Order intake | YAML orders are checked against the inventory (unknown items, double-booked stock, tote count) | `autonomy/orders.py` |
| Localization & mapping (SLAM) | See the next section. | `autonomy/slam_*.py`, `odometry.py`, `lidar.py`, `localization.py` |
| Navigation | The live SLAM map is inflated by the robot's footprint; unmapped space counts as free. A* plans a path, line-of-sight smoothing simplifies it, and pure pursuit follows it on the diff-drive base. Every 2 s the map is refreshed and the path re-planned if it became blocked. The robot stops at a staging point and finishes with a straight approach. | `autonomy/mapping.py`, `navigation.py`, `motion.py` |
| Identification | The D435i head camera points at the slot the WMS gives. RGB, depth and segmentation are rendered, depth is back-projected to 3-D, and the item is identified and measured. | `autonomy/perception.py` |
| Grasp choice | Items under 6 cm tall get a top-down grasp, with the wrist rolled across the narrow side. Taller items get a side grasp. Checks cover gripper width, lift range and headroom. | `autonomy/grasping.py` |
| Pick | All positioning is closed-loop on the measured finger-pad pinch point. Lift, arm and wrist motion are ramped at real-robot speeds while the base holds its pose. The gripper uses a gentle squeeze and proprioception confirms the item is held. Side-held items are folded in front of the base for transport. | `autonomy/manipulation.py` |
| Pack | The robot drives to the station, reaches over the tote, lowers until contact and releases. Ground truth confirms the item is in the tote (grading only). | `manipulation.py`, `fulfill.py` |
| Recovery | Any failed step opens the gripper, stows the arm, backs the robot away, records the reason and moves on to the next item. At the end the robot returns to the dock. | `autonomy/fulfill.py` |

### SLAM localization and mapping

```
 MuJoCo (host)                                        Docker: ROS 2 Jazzy
 ─────────────                                        ───────────────────
 wheel encoders + gyro (500 Hz) ─► odometry ── /tf odom→base_link ─┐
 base lidar, 360 beams, 10 Hz   ──────────────── /scan ────────────┼─► rosbridge ─► slam_toolbox (online sync)
 sim clock                      ──────────────── /clock ───────────┘                │
                                                                                    │ /pose (per processed scan)
 robot pose = dock ∘ map→odom (SLAM) ∘ odom→base (odometry) ◄───────────────────────┤
 planner grid  ◄── /map (resampled to the layout frame, inflated) ◄─────────────────┘
```

- **Sensors.** The base lidar is the Menagerie `laser` body: 360 ray casts, 1 cm
  noise and 0.5 % dropouts, with hits on the robot itself discarded. The wheel
  encoders are quantised (4096 counts per revolution) and the wheel radius is
  miscalibrated by 0.3 %. The gyro has noise and bias and is integrated at the
  physics rate, like an IMU's delta-angle output.
- **Lockstep.** slam_toolbox processes a scan only after the robot moves 15 cm
  or turns 0.15 rad. The host predicts exactly which scans it will process
  (`scan_gate.py`) and waits for the matching `/pose`, so a run gives the same
  result however fast the host is. While the robot stands still nothing is
  processed, so manipulation costs no SLAM time.
- **Frames.** The map frame is wherever the robot starts, which is its dock. The
  dock's position in `layout.yaml` places the map in the warehouse, so shelf
  slots and totes (also taken from `layout.yaml`) can be targeted.
- **One frame for hand and eye.** The camera pose (for back-projecting
  detections) and the gripper pinch point both come from forward kinematics on
  the *estimated* base pose (`EstimatedKinematics`). Base pose error therefore
  cancels between "where I saw the item" and "where my hand is". SLAM
  corrections are held back from detection until the grasp ends, so the frame
  doesn't jump mid-pick.
- **Survey.** Each run starts with one full turn on the dock to fill in the
  lidar's mast shadow.

Simplifications, stated plainly:
- **Localization** comes from SLAM, as described above. `--localization ground-truth` is kept as a baseline.
- **Identification** uses MuJoCo's segmentation render as an oracle instance-segmentation model. A learned detector can replace `item_masks()` in `perception.py`.
- **Environment tweaks for a 2-D lidar** (it scans at 0.166 m):
  - The bottom shelf deck is at 0.10 m. At 0.15 m the scan plane grazed it and made scan matching unstable.
  - The packing bench has skirt panels. Otherwise the lidar sees only its legs, and a 2-D planner would route the mast under the table top.
- **Stretch 3 model tweaks** (in `robot.py`):
  - The wheel speed limit is raised to about 0.24 m/s. The real robot reaches about 0.3 m/s; Menagerie caps it at 0.1 m/s.
  - The rubber fingertip pads are mounted rigidly. In the Menagerie model they sit on near-free hinges, so held items pivot and fall out.

In the viewer, the camera dropdown has `overview`, `top_down`,
`shelf_{A,B,C}_cam` and `packing_station_cam`. Enable **site group 4** to see
the slot / drop-zone markers.

## Scene contents

| Element | Names in the model |
|---|---|
| Shelving units (static, 4 levels) | `shelf_A`, `shelf_B`, `shelf_C` |
| Slot target sites on every level | `shelf_<S>_L<level>_S<slot>`, e.g. `shelf_A_L1_S0` |
| Grocery items (free bodies) | `item_<name>`, joint `item_<name>_joint` |
| Packing table + 2 delivery totes | body `packing_station`, sites `tote_0`, `tote_1` |
| Primary drop target | site `drop_zone` (inside `tote_0`) |
| Stretch 3 (all names prefixed) | `robot/base_link`, `robot/joint_lift`, camera `robot/d435i_camera_rgb`, ... |
| Robot start pose | site `robot_spawn` on the blue dock pad, keyframe `stow` |

Items sit on levels 1 and 2 (0.55 m and 1.05 m); the bottom deck (level 0) is at 0.10 m. Level 1 holds the flat packs,
because top-down grasps need about 0.36 m of headroom for the arm. Level 2 holds
only tall items, because a top-down grasp there would exceed Stretch's 1.1 m
lift. Levels 0 and 3 hold static cardboard "back-stock". The packing bench is
0.72 m tall so the robot can lift items over the tote rims.

| Shelf | Theme | Items |
|---|---|---|
| A | Sauces & bottles | ketchup, bbq_sauce, mustard (L1); salad_dressing, tomato_sauce (L2) |
| B | Snacks & boxes | cookies, mac_and_cheese, pudding_mix, popcorn (L1); crackers, sugar (L2) |
| C | Cans & dairy | tomato_soup, tuna, butter, cream_cheese (L1); alphabet_soup, milk, orange_juice (L2) |

## Customising

All placement is driven by data, so you don't need to change code:

- `config/layout.yaml`: room size, shelf dimensions and level heights, shelf
  positions, station/tote sizes, dock position.
- `config/inventory.yaml`: which item goes in which shelf/level/slot. You can
  also set the orientation (`euler_deg`, `yaw_deg`) and `scale`. HOPE items
  default to 1.5× because they are toy-sized.

Run `uv run qcw-build` after editing. Configs are validated: unknown shelves,
out-of-range slots and double-booked slots all fail with a clear message.

## Code layout

```
src/qc_warehouse/
  config.py       YAML -> frozen dataclasses + validation
  shelf.py        procedural rack unit, slot sites, shelf cameras
  station.py      packing table, totes, drop_zone, robot dock
  room.py         floor, walls, lights, floor tape, global cameras
  items.py        loads third-party item MJCFs, scales/uprights, attaches as free bodies
  robot.py        attaches Stretch 3 (Menagerie) + model tweaks
  build_scene.py  composes everything with mujoco.MjSpec -> assets/scene.xml
  pick.py         qcw-pick CLI (headless or passive viewer)
  slam_cli.py     qcw-slam: docker compose wrapper for the SLAM service
  map_image.py    SLAM map + trajectories -> PNG
  view.py / render.py
  autonomy/
    orders.py       order file loading/validation
    warehouse.py    WMS view from layout.yaml: slots, shelf faces, totes (+ ground-truth grading)
    stretch.py      robot interface (estimated/true pose, encoders, gyro, joint commands, diff drive)
    frames.py       SE2 poses and frame algebra
    odometry.py     wheel-encoder + gyro dead reckoning
    lidar.py        2-D lidar ray-cast model
    scan_gate.py    predicts which scans slam_toolbox processes (lockstep)
    ros_messages.py ROS 2 message JSON builders/parsers, ROS map resampling
    slam_bridge.py  rosbridge client for slam_toolbox (+ offline fake for tests)
    slam_frontend.py per-tick sensor -> SLAM -> correction pipeline
    localization.py SLAM localizer, estimated kinematics, error monitor
    slam_stack.py   wires the SLAM pipeline onto robot + runner
    mapping.py      planner map providers: live SLAM map or a static grid
    navigation.py   occupancy grid, A*, path smoothing, pure pursuit (pure functions)
    motion.py       base skills: drive/rotate/follow path/navigate, base hold, head look-at
    perception.py   RGB-D capture, back-projection, item identification
    grasping.py     side vs top-down grasp planning (pure)
    poses.py        where to stand to reach a shelf slot / tote (pure)
    manipulation.py pick / place skills
    fulfill.py      order fulfilment loop, recovery, results
    runner.py       fixed-rate control loop, sensor hooks, viewer sync
docker/                       slam_toolbox + rosbridge image, launch file, SLAM parameters, compose file
scripts/fetch_assets.py, scripts/setup_mjpython.sh
```

## Asset credits & licenses

- **LIBERO HOPE objects**: Lifelong-Robot-Learning/LIBERO, MIT license. These
  are MJCF conversions of the HOPE dataset (NVIDIA).
- **YCB objects**: vikashplus/YCB_sim, Apache-2.0. Based on the YCB Object and
  Model Set (Calli et al.).
- **Hello Robot Stretch 3**: google-deepmind/mujoco_menagerie, Apache-2.0.
- Shelves, station and room are built procedurally in this repo.
- **slam_toolbox** (Steve Macenski, LGPL-2.1) and **rosbridge_suite** (BSD) run
  unmodified from the ROS 2 Jazzy apt packages inside the Docker image. Neither
  is vendored or linked into this repo; the simulator talks to them over a
  websocket via **roslibpy** (MIT).
