# data/ — simulation data for building and tuning the real robot

Everything the simulation (`../robot-sim`) learns or measures lands here, so the real hardware can start from
numbers that have already been exercised in simulation instead of from scratch.

```
data/
  calibration/sim_parameters.json   what the simulated robot + scene look like and how it is driven
  runs.jsonl                        one line per recorded run: parameters, success, timings, key measurements
  episodes/<id>/
      meta.json                     parameters, outcome, per-stage durations, tuned constants, log
      trajectory.npz                10 Hz time series (see below)
      frames/wrist/*.jpg            wrist-camera frames  (git-ignored: large)
      frames/chassis/*.jpg          chassis-camera frames (git-ignored: large)
```

## How it is produced

| Command (in `robot-sim/`) | What it does |
|---|---|
| `uv run dsr-panel` and tick **Record data** | records every run you start from the web page |
| `uv run dsr-collect --task restock_misplaced --episodes 20` | 20 headless runs with random item positions |
| `uv run dsr-collect --task shelf_to_tray --episodes 9 --sweep` | every rack once |
| `... --no-assist` | pure friction grasps (no weld) — the honest, less reliable mode |
| `... --no-images` | trajectories only (about 100x smaller, much faster) |

`calibration/sim_parameters.json` is rewritten whenever you run the collector.

## `trajectory.npz` arrays (N samples at ~10 Hz)

| Array | Shape | Meaning |
|---|---|---|
| `t` | N | simulated seconds |
| `stage` / `stage_names` | N / S | index into the run's stage names (`drive_to_item`, `pre_grasp`, `drive_in`, `grasp`, `lift_back_out`, `carry`, `release`, `return_home`, ...) |
| `joints_deg` | N×6 | measured arm joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper |
| `joints_cmd_deg` | N×6 | commanded arm joint targets (what a policy would output as `action`) |
| `base_pose` | N×3 | rover x (m), y (m), yaw (rad) in the table frame |
| `base_cmd` | N×3 | commanded rover twist: vx, vy (m/s, body frame), wz (rad/s) |
| `item_pos` | N×3 | item centre (m) — ground truth, not available on the real robot |
| `jaw_centre` | N×3 | point between the jaws (m) |
| `weld` | N | 1 while the grasp-assist weld is carrying the item |

Joint order and zero point match LeRobot's SO-101 with `use_degrees=True`: zero is the middle of each joint's range
(SO-101 "new calibration"). The gripper is in degrees of jaw angle here; LeRobot's SO-101 reports it on a 0–100 scale,
so a mapping has to be fitted on the real arm (see below).

## Recorded set (as of 2026-10-10, 43 runs)

| Simulation | Grasp assist | Success | Mean sim time |
|---|---|---|---|
| shelf_to_tray (every rack once) | on | 9/9 | 26 s |
| shelf_to_tray (every rack once) | off | 3/9 | 26 s |
| restock_misplaced (random table spots, random dest rack) | on | 14/15 | 23 s |
| restock_misplaced (random table spots, random dest rack) | off | 6/10 | 19 s |

The one assisted failure is an item left right by the table corner next to the rover's start spot (−0.68, −0.38),
where no stand-off pose fits on the table. The assist-off failures are grip failures (item dropped or wedged).

## What carries over to the real robot — and what does not

**Carries over (use directly, then verify):**

- **Geometry and kinematics** — wheel, deck, arm-mount, camera positions and the SO-101 joint limits in
  `sim_parameters.json`. Build the rover to these, or edit the model (`frontend/src/lib/robotModel.ts`) to match
  what you really built and re-export.
- **The approach recipe** — jaws level, wrist roll 90° so the jaws open sideways, stand off at the rack mouth,
  centre sideways using the camera, drive straight in along the jaw axis, close, lift, back out. The numbers
  (`manipulation` block) are starting values: stand-off 0.34–0.37 m, jaw-centre offset 0.036 m, grasp height 1.5 cm
  above the item centre, 12.5 cm drive-in.
- **Joint trajectories** — `joints_cmd_deg` per stage is a reference for what the arm has to do; replay it
  slowly on the real arm to check limits and clearances before running a policy.
- **Reachability limits** — which racks/table spots can be reached and from where (the planner rejects the
  rest). The top rack row and the left-most racks are within reach; spots behind the tray are not.
- **Stage timings** — a sanity range for cycle time, not a promise: real servos and a real base will be slower.

**Does not carry over (treat as unknown until measured on hardware):**

- **Grip quality.** With *grasp assist* on, the sim carries the item with a weld, so a successful run says the
  motion is feasible, **not** that friction would hold the item. Runs with `grasp_assist: false` are the more
  honest signal and succeed noticeably less often (see the run index). Real jaw pads, item weight and
  surface decide this.
- **Mecanum slip and floor friction**, wheel-speed limits, battery sag — modelled as an ideal holonomic base.
- **Camera images** — rendered, not real; lighting, exposure and latency differ. Use them for pipeline
  testing and as a prior, not as a substitute for real demonstrations.
- **Servo behaviour** — the sim uses ideal position actuators with the stock gains.

## Using it for the real build

1. **Check the hardware against `calibration/`**: joint ranges, mount positions, camera poses. Anything that
   differs is a model update, not a policy problem.
2. **Replay a reference trajectory** from `episodes/*/trajectory.npz` (`joints_cmd_deg`) on the real arm at low
   speed; log what it really does and compare.
3. **Fit the gripper mapping**: record the real gripper's 0–100 reading at known jaw openings and map it to the
   sim degrees (`gripper_deg` block: open 52, release 70, closed −5).
4. **Training data**: `dsr-collect` produces image + joint-state + action episodes in bulk. Use them to
   pre-train or to test the ACT/LeRobot pipeline end to end; fine-tune with real teleoperated episodes. A
   converter to the LeRobot dataset format is not included yet.
5. **Keep both columns**: when a real run disagrees with the sim, add it next to the sim run (same
   parameters). The gap is the thing worth modelling next.

## Known limits of the current data

- One item (a soup can scaled to 4.6 cm so the SO-101 jaw can open around it).
- Fixed shelf, tray and dock positions; only the pick rack, the item's table position and its destination rack vary.
- Recorded episodes come from a scripted controller (inverse kinematics + closed-loop sideways correction),
  not from a learned policy or from human demonstrations.
