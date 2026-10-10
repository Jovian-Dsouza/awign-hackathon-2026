# data/ — simulation data for building and tuning the real robot

Everything the simulation (`../robot-sim`) learns or measures lands here, so the real hardware can start from
numbers that have already been exercised in simulation instead of from scratch.

```
data/
  calibration/sim_parameters.json   what the simulated robot + scene look like and how it is driven
  runs.jsonl                        one line per recorded run: parameters, success, timings, key measurements
  lerobot/dark_store_so101/         LeRobotDataset v3 export (parquet + mp4) for `lerobot-train`
  pi_trajectories/                  episodes as Pi-panel playback files
  tools/export_lerobot.py           the converter
  tools/export_pi_trajectories.py   Pi-panel trajectory writer
  episodes/<id>/
      meta.json                     parameters, outcome, per-stage durations, tuned constants, log
      trajectory.npz                10 Hz time series (see below)
      frames/wrist/*.jpg            wrist-camera frames, 640x480  (git-ignored: large)
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

## Recorded set

None right now. The synthetic episodes and the LeRobot dataset made from them were deleted on request: rendered
frames are not real photos of the real wrist camera, so they are not useful for training the real policy. Generate
a fresh set with `dsr-collect` whenever you need trajectories (for checking motion limits, timings or the
Pi-panel playback files), not for fine-tuning.

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

## LeRobot dataset (optional, from simulation)

The team's real ACT pipeline is `firmware/act-pick-infer/` (record real demos on the Pi with `record.py`, fine-tune the
Hugging Face ACT `TakuyaHiraoka/act_so101_pick_diverse_objects` with `finetune_colab.py` on a GPU, run it with
`infer.py`). The simulation does not use ACT. This exporter only exists to produce a same-format dataset (480x640
`observation.images.wrist`, 6-joint `observation.state` / `action`) if you ever want to test the pipeline with it.


`tools/export_lerobot.py` (optional) converts the recorded episodes into a **LeRobotDataset (v3)** with LeRobot's own writer, so
`lerobot-train`, `lerobot-dataset-viz` etc. read it directly. It follows the real robot's conventions, taken from the
Pi firmware (`../firmware/combined-web/web_combined.py`, which drives the arm through LeRobot's `SO101Follower`):

| Feature | Shape | What |
|---|---|---|
| `observation.images.wrist` | 3x480x640 video | the only camera: USB bar camera on the gripper (firmware streams 640x480) |
| `observation.state` | 6 | `<joint>.pos`: body joints in degrees, gripper 0..100 (0 = closed) |
| `action` | 6 (9 with `--with-base`) | commanded `<joint>.pos`, same units; base_vx/vy/wz in the Arduino's -255..255 `V` units |
| `task` | text | e.g. "pick up the soup can from rack level 2 slot 0 and put it in the tray" |

```bash
uv run --with 'lerobot[dataset]' python data/tools/export_lerobot.py                 # arm only, one camera
uv run --with 'lerobot[dataset]' python data/tools/export_lerobot.py --with-base --out data/lerobot/with_base
# options: --scenario, --assist on|off|any, --include-failed, --no-videos
```

Ground-truth item position and the assist-weld flag are left out on purpose (they do not exist on the real robot).
Only successful episodes **with camera frames** are exported by default.

`tools/export_pi_trajectories.py` writes the same episodes as `{t, pos}` JSON in the format of the Pi panel's
record/playback (`~/.so101_arm/trajs/`), so a simulated pick can be replayed slowly on the real arm first.

Train (ACT, wrist camera only; use a GPU or Apple MPS):

```bash
uv run --with 'lerobot[dataset,training]' lerobot-train \
  --dataset.repo_id=local/dark_store_so101 --dataset.root=data/lerobot/dark_store_so101 \
  --policy.type=act --policy.push_to_hub=false --policy.device=mps \
  --batch_size=8 --steps=20000 --output_dir=outputs/act_dark_store
```

Reality check before you expect a useful policy:

- Demonstrations are **scripted and near-identical** per rack, with one item, so a policy trained on them will be
  brittle. Best use: pre-training / pipeline validation, then fine-tune on real teleoperated episodes.
- With grasp assist on, a successful sim pick does not prove friction holds the item.
- Images are rendered; expect a domain gap to the real camera (lighting, exposure, lens; intrinsics unknown).
- Units match the firmware (degrees, gripper 0..100), and the sim picks at wrist roll about -90 like the real arm
  (about -92), but the sim's gripper 0..100 mapping assumes the SO-101's calibrated range matches its joint limits.

## Using it for the real build

1. **Check the hardware against `calibration/`**: joint ranges, mount positions, camera poses. Anything that
   differs is a model update, not a policy problem.
2. **Replay a reference trajectory** from `episodes/*/trajectory.npz` (`joints_cmd_deg`) on the real arm at low
   speed; log what it really does and compare.
3. **Fit the gripper mapping**: record the real gripper's 0–100 reading at known jaw openings and map it to the
   sim degrees (`gripper_deg` block: open 52, release 70, closed −5).
4. **Training data**: `dsr-collect` produces image + joint-state + action episodes in bulk. Use them to
   pre-train or to test the ACT/LeRobot pipeline end to end (`tools/export_lerobot.py` produces the LeRobot
   format); fine-tune with real teleoperated episodes.
5. **Keep both columns**: when a real run disagrees with the sim, add it next to the sim run (same
   parameters). The gap is the thing worth modelling next.

## Known limits of the current data

- One item (a soup can scaled to 4.6 cm so the SO-101 jaw can open around it).
- Fixed shelf, tray and dock positions; only the pick rack, the item's table position and its destination rack vary.
- Recorded episodes come from a scripted controller (inverse kinematics + closed-loop sideways correction),
  not from a learned policy or from human demonstrations.
