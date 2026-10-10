# Dark-store robot simulation (MuJoCo)

Simulation of **our** robot in a miniature quick-commerce dark store:

- **Rover:** 4-wheel mecanum chassis (holonomic planar drive, wheels spin from mecanum IK). The geometry is exported from the website model (`frontend/src/lib/robotModel.ts`), so the sim and the site show the identical robot
- **Arm:** SO-101 (stock MJCF from [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), position actuators)
- **Compute:** Raspberry Pi (visual only)
- **Cameras:** `wrist_cam` on the gripper, `chassis_cam` on the front of the chassis
- **Store:** 3 shelves + packing station + dock, scaled down so the arm can reach every level

Reference: the Stretch 3 simulation in `../simulation/` (full autonomy stack: SLAM, navigation,
grasping). Scene builders (`shelf`, `station`, `room`, `items`, `config`) are reused from it and
scaled to our robot; the robot, control and demo are new.

## Run

```bash
# only when the website model changes: re-export it (needs Node 22+)
(cd ../frontend && node scripts/export-robot.ts ../robot-sim/assets/robot)

uv sync
uv run python scripts/fetch_assets.py   # grocery item models into assets/third_party/ (pinned commits)
uv run dsr-build                        # regenerate assets/scene.xml from config/
uv run dsr-demo                         # drive + arm demo, frames in renders/
uv run dsr-view                         # interactive viewer (static scene)
uv run dsr-pick                         # scripted pick-and-carry, headless, prints result
./scripts/setup_mjpython.sh             # macOS, once: lets mjpython run in this venv
uv run mjpython -m dsr_sim.pick --viewer  # watch the pick-and-carry live (keys: = - zoom, 1-4 camera views)
uv run pytest
```

Config lives in `config/layout.yaml` and `config/inventory.yaml`. Robot dimensions, camera poses
and arm rest/ready poses are constants at the top of `src/dsr_sim/robot.py`.

## Control API (`dsr_sim.control.Robot`)

```python
robot.set_twist(vx, vy, wz)      # body-frame velocity, m/s and rad/s (x forward, y left)
robot.set_arm_deg("ready")       # named pose, or a 6-tuple of joint degrees
robot.arm_deg()                  # current joint angles in degrees
robot.pose                       # x, y, yaw of the rover
```

Joint order and zero convention match LeRobot's SO-101 with `use_degrees=True`
(`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`).

## What is and is not modelled

| Modelled | Not (yet) |
|---|---|
| Holonomic base with mecanum wheel IK and spinning wheels | Mecanum roller slip / wheel-floor contact |
| SO-101 kinematics, joint limits, servo-like position actuators | Servo backlash, torque limits beyond the stock model |
| Two cameras rendered from the right mounts | Real camera intrinsics, exposure, USB latency |
| Items, shelves, tote, dock | Item physical properties for real grocery packaging |
| Scripted pick-and-carry (IK + friction grasp) | SLAM, navigation, a *learned* grasp policy, LeRobot dataset recording |

Dimensions are approximate (from photos). Wrist-camera pose and the Pi position are placeholders.
The robot geometry lives in the frontend (`robotModel.ts`); edit it there and re-export, not in `robot.py`.

## Next steps

1. Replace approximate dimensions with measured ones; add the real printed wrist-camera clip model.
2. Gym-style env (reset/step, camera images + joint state) so LeRobot can record episodes.
3. Scripted pick-and-place demonstrations to generate an ACT training dataset.
4. Navigation/localization for the mecanum base (port ideas from `../simulation/`).
