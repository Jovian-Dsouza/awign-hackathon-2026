"""Our robot: 4-wheel mecanum rover + SO-101 arm + Raspberry Pi + two cameras.

The rover geometry (decks, wheels with rollers, Pi) is NOT hand-written here: it is
loaded from assets/robot/robot_parts.json, exported from the website's model
(frontend/src/lib/robotModel.ts via `node scripts/export-robot.ts`), so the simulation and the site
show the identical robot. Re-export after changing the frontend model.

Drive model: the rover is a holonomic planar body (slide x, slide y, hinge yaw) driven by velocity
actuators; the four wheel hinges are driven by mecanum inverse kinematics so they spin correctly,
but they do not touch the floor. Real mecanum roller slip is NOT modelled.

Names: rover bodies/joints/actuators under `robot/`, the arm under `so101/`.
"""

from __future__ import annotations

import json
import re
import struct
from pathlib import Path

import mujoco
import numpy as np

from dsr_sim.config import DockConfig
from dsr_sim.constants import ASSETS_DIR
from dsr_sim.items import AssetError, absolutize_files

ROBOT_ASSETS = ASSETS_DIR / "robot"
PARTS_JSON = ROBOT_ASSETS / "robot_parts.json"
SO101_XML = ASSETS_DIR / "so101" / "so101_new_calib.xml"
BASE = "robot/base"
ARM_PREFIX = "so101/"


def _load_model() -> dict:
    if not PARTS_JSON.exists():
        raise AssetError(
            f"{PARTS_JSON} not found. Export the robot from the website model:\n"
            "  cd ../frontend && node scripts/export-robot.ts ../robot-sim/assets/robot"
        )
    return json.loads(PARTS_JSON.read_text())


MODEL = _load_model()

WHEEL_RADIUS = MODEL["base"]["wheelRadius"]
HALF_WHEELBASE = MODEL["base"]["halfWheelbase"]
HALF_TRACK = MODEL["base"]["halfTrack"]
DECK_TOP = MODEL["anchors"]["deck_top"]
ARM_MOUNT = tuple(MODEL["anchors"]["arm_mount"])
WHEELS = {tag: (1 if w["position"][0] > 0 else -1, 1 if w["position"][1] > 0 else -1) for tag, w in MODEL["wheels"].items()}

BASE_KV = 400.0
WHEEL_KV = 2.0
MAX_LINEAR = 0.5  # m/s command limit
MAX_ANGULAR = 3.0  # rad/s
MAX_WHEEL = 40.0  # rad/s
JAW_FRICTION = 1.0  # sliding friction on the jaws (items are ~1.0). Too grippy drags released items along.

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Degrees, "new calibration" zero (middle of each joint's range); same convention as LeRobot degrees.
GRIPPER_DEG_RANGE = (-10.0, 100.0)  # MJCF gripper range <-> LeRobot gripper 0..100


def gripper_deg_to_pct(deg: float) -> float:
    lo, hi = GRIPPER_DEG_RANGE
    return (deg - lo) / (hi - lo) * 100.0


def gripper_pct_to_deg(pct: float) -> float:
    lo, hi = GRIPPER_DEG_RANGE
    return lo + pct / 100.0 * (hi - lo)


# "rest" is where the real arm starts every episode of the working ACT pipeline: the 10 recordings in
# data/real/so101_pick_test1_*/ all begin at pan -3, lift -85.8, elbow 97, wrist flex 3.8, roll -81, gripper 1.3
# (observation.state, LeRobot degrees; gripper 0..100). The sim's wrist-roll zero sits 90 deg from the real
# calibration (the real arm picks at roll ~0 where the sim's level-jaw grasp is -90), so the roll is real - 90 = -171,
# which is past the sim joint's limit (-157.2): the nearest reachable value is used. Elbow 97 is clipped to 96.8.
POSES_DEG = {
    "rest": (-2.9, -85.8, 96.8, 3.8, -157.0, -4.2),
    "ready": (0, -40, 55, 60, 0, 40),
    "reach_low": (0, 10, 10, 0, 0, 60),
}

# Same colours as the website's arm styling (printed parts vs servo motors).
ARM_PRINTED_RGBA = (0x2B / 255, 0x2B / 255, 0x30 / 255, 1.0)
ARM_MOTOR_RGBA = (0x0C / 255, 0x0C / 255, 0x0E / 255, 1.0)

# Parts that take part in collisions (the rest are visual only).
COLLIDING = {"deck_lower", "deck_upper", "battery"}
_Q_Z_TO_Y = np.array([np.cos(-np.pi / 4), np.sin(-np.pi / 4), 0.0, 0.0])  # maps MuJoCo's cylinder axis (z) to three.js's (y)


# ---------------------------------------------------------------- small math helpers
def _mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def _xyzw_to_wxyz(q) -> np.ndarray:
    return np.array([q[3], q[0], q[1], q[2]], dtype=float)


def _rgba(hex_color: str, alpha: float = 1.0) -> list[float]:
    h = hex_color.lstrip("#")
    if len(h) == 3:
        h = "".join(c * 2 for c in h)
    return [int(h[i : i + 2], 16) / 255 for i in (0, 2, 4)] + [alpha]


def _rpy_to_mat(roll, pitch, yaw) -> np.ndarray:
    """URDF fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _mat_to_quat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, mat.reshape(9))
    return quat


def _stl_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned bounds of an STL (binary or ASCII)."""
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        pts = np.array([[float(v) for v in m.groups()] for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)])
    else:
        n = struct.unpack("<I", data[80:84])[0]
        tri = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n, offset=84)
        pts = tri["v"].reshape(-1, 3)
    return pts.min(axis=0), pts.max(axis=0)


# ---------------------------------------------------------------- generic part -> MuJoCo geom
class _Materials:
    """One MuJoCo material per distinct (colour, metalness, roughness, emission)."""

    def __init__(self, spec: mujoco.MjSpec) -> None:
        self.spec, self._names = spec, {}

    def get(self, part: dict) -> str:
        key = (part["color"], part.get("metalness", 0.0), part.get("roughness", 0.6), part.get("emissive"))
        if key not in self._names:
            name = f"robot/mat{len(self._names)}"
            mat = self.spec.add_material(name=name, rgba=_rgba(part["color"]))
            mat.specular = 0.1 + 0.5 * key[1]
            mat.shininess = 0.2 + 0.6 * (1.0 - key[2])
            if key[3]:
                mat.emission = 0.8
            self._names[key] = name
        return self._names[key]


def _add_part(body: mujoco.MjsBody, part: dict, mats: _Materials, collide: bool) -> mujoco.MjsGeom:
    a = part["args"]
    quat = _xyzw_to_wxyz(part["quaternion"])
    kw: dict = {"pos": list(part["position"]), "material": mats.get(part)}
    shape = part["shape"]
    if shape == "box":
        kw.update(type=mujoco.mjtGeom.mjGEOM_BOX, size=[a[0] / 2, a[1] / 2, a[2] / 2], quat=list(quat))
    elif shape == "cylinder":
        kw.update(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[a[0], a[2] / 2, 0], quat=list(_mul(quat, _Q_Z_TO_Y)))
    elif shape == "capsule":
        kw.update(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[a[0], a[1] / 2, 0], quat=list(_mul(quat, _Q_Z_TO_Y)))
    elif shape == "sphere":
        kw.update(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[a[0], 0, 0], quat=list(quat))
    elif shape == "deck":
        mesh_name = f"robot/{part['name']}_mesh"
        kw.update(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mesh_name, quat=list(quat))
    else:
        raise AssetError(f"unknown part shape {shape!r} in {PARTS_JSON}")
    geom = body.add_geom(**kw)
    geom.name = f"robot/{part['name']}" if "/" not in part["name"] else part["name"]
    if not collide:
        geom.contype = 0
        geom.conaffinity = 0
        geom.group = 2
    return geom


def _register_meshes(spec: mujoco.MjSpec) -> None:
    for part in MODEL["chassis"]:
        if part.get("mesh"):
            spec.add_mesh(name=f"robot/{part['name']}_mesh", file=str(ROBOT_ASSETS / part["mesh"]))


# ---------------------------------------------------------------- build
def _velocity_actuator(spec, name, joint, kv, limit):
    a = spec.add_actuator(name=name, trntype=mujoco.mjtTrn.mjTRN_JOINT, target=joint)
    a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    a.gainprm[0] = kv
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.biasprm[2] = -kv
    a.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    a.ctrlrange = [-limit, limit]
    return a


def _add_wheels(spec: mujoco.MjSpec, base: mujoco.MjsBody, mats: _Materials) -> None:
    for tag, wheel in MODEL["wheels"].items():
        body = base.add_body(name=f"robot/wheel_{tag}", pos=list(wheel["position"]))
        joint = body.add_joint(name=f"robot/wheel_{tag}_joint", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0], damping=0.002)
        for part in wheel["parts"]:
            geom = _add_part(body, {**part, "name": f"wheel_{tag}_{part['name']}"}, mats, collide=False)
            geom.mass = 0.002  # light visual rollers; the rim carries the wheel mass below
        _velocity_actuator(spec, f"robot/wheel_{tag}", joint.name, WHEEL_KV, MAX_WHEEL)


# Bar webcam clipped on a printed bracket above the gripper (see the photos of the real arm). Gripper frame:
# x = jaw-opening direction, -y = top of the gripper at the sim's grasp roll (+90 deg), -z = forward. Boxes are (centre, size, colour); shared with
# frontend/src/components/Arm.tsx so both models match.
WRIST_CAM_PARTS = (
    ("post", (0.0, 0.043, -0.020), (0.022, 0.034, 0.034), "#2b2b30"),    # printed column glued to the gripper top
    ("bracket", (0.0, 0.0615, -0.030), (0.050, 0.006, 0.040), "#2b2b30"),  # platform the camera sits on
    ("cheek_l", (-0.027, 0.070, -0.030), (0.005, 0.016, 0.040), "#2b2b30"),
    ("cheek_r", (0.027, 0.070, -0.030), (0.005, 0.016, 0.040), "#2b2b30"),
    ("cam_body", (0.0, 0.0755, -0.034), (0.092, 0.024, 0.026), "#0c0c0e"),  # bar camera
    ("cam_face", (0.0, 0.0755, -0.0475), (0.040, 0.016, 0.003), "#17171a"),  # front bezel
)
WRIST_CAM_LENS = (0.0, 0.0755, -0.0495)  # lens centre of the part drawing above (its own frame)
# Thin flat USB lead: leaves the camera's side and runs back along the arm.
WRIST_CAM_CABLE = ((-0.046, 0.0755, -0.034), (-0.056, 0.074, -0.018), (-0.050, 0.052, 0.020), (-0.034, 0.030, 0.062))

# The camera's view direction and field of view are FITTED to the real wrist camera, not guessed:
# data/real/so101_pick_test1_*/ (10 episodes) shows the two jaw blades as dark masks, and the pose below is the one
# whose rendered jaws overlap those masks best (IoU about 0.8 at 5 gripper openings). The lens stays where the photos
# of the real arm put it (printed bracket above the gripper, WRIST_CAM_LENS); what the fit adds is that the optic
# looks 57 deg down toward the jaw tips, so both blades fill the lower half of the frame and converge toward the top,
# and that the lens is a normal ~45 deg (vertical) webcam lens. The same fit gives the real gripper scale:
# jaw angle ~ -5.6 + 1.02 * (gripper 0..100).
WRIST_CAM_POS = (0.0, 0.0755, -0.0485)  # lens centre in the gripper frame, m (1 mm in front of the lens ring)
WRIST_CAM_EULER_DEG = (-57.0, 3.3, -0.7)  # extrinsic x, y, z rotations of the camera frame (x right, y up, looks -z)
WRIST_CAM_FOVY = 44.8  # vertical field of view, deg (640x480)


def wrist_cam_rotation() -> np.ndarray:
    """Camera frame (columns: x right, y up, z backwards) expressed in the gripper frame."""
    rx, ry, rz = np.deg2rad(WRIST_CAM_EULER_DEG)
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    mx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    my = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    mz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return mz @ my @ mx


def _add_wrist_camera(spec: mujoco.MjSpec, mats: _Materials) -> None:
    """Bar webcam on a printed bracket above the gripper, thin USB lead, plus a MuJoCo camera at the lens."""
    grip = spec.body(ARM_PREFIX + "gripper")
    if grip is None:
        raise AssetError("SO-101 gripper body not found; cannot mount wrist camera")
    rot = wrist_cam_rotation()

    def deco(name, geom_type, color, roughness=0.6, **kw):
        g = grip.add_geom(type=geom_type, material=mats.get({"color": color, "roughness": roughness}), **kw)
        g.name = f"robot/wrist_cam_{name}"
        g.contype = g.conaffinity = 0
        g.group = 2

    for name, pos, size, color in WRIST_CAM_PARTS:
        deco(name, mujoco.mjtGeom.mjGEOM_BOX, color, pos=list(pos), size=[s / 2 for s in size])
    lx, ly, lz = WRIST_CAM_LENS
    deco("ring", mujoco.mjtGeom.mjGEOM_CYLINDER, "#b9bec6", 0.3, pos=[lx, ly, lz], size=[0.0085, 0.0012, 0], quat=[1, 0, 0, 0])
    deco("lens", mujoco.mjtGeom.mjGEOM_CYLINDER, "#05060a", 0.2, pos=[lx, ly, lz - 0.0008], size=[0.0058, 0.0012, 0], quat=[1, 0, 0, 0])
    for i, (a, b) in enumerate(zip(WRIST_CAM_CABLE[:-1], WRIST_CAM_CABLE[1:])):
        deco(f"cable{i}", mujoco.mjtGeom.mjGEOM_CAPSULE, "#0a0a0c", 0.7, fromto=[*a, *b], size=[0.0014, 0, 0])
    grip.add_camera(name="wrist_cam", pos=list(WRIST_CAM_POS), xyaxes=[*rot[:, 0], *rot[:, 1]], fovy=WRIST_CAM_FOVY)


def _recolor_arm(spec: mujoco.MjSpec) -> None:
    """Match the website: printed parts dark grey, servo motors near-black."""
    for mat in spec.materials:
        if mat.name.startswith(ARM_PREFIX):
            mat.rgba = list(ARM_MOTOR_RGBA if "sts3215" in mat.name else ARM_PRINTED_RGBA)


COLLISION_PIECES_DIR = ASSETS_DIR / "so101" / "collision"


def _convex_collision(spec: mujoco.MjSpec) -> None:
    """Collide the arm links as the convex pieces of scripts/decompose_collision.py, not as one hull per mesh.

    MuJoCo collides a mesh as a single convex hull. The arm's links are not convex: the hull of the gripper fills
    the gap between its jaws and the folded arm lies inside its own hulls, so nothing could be gripped and the
    arm wedged itself at rest. The pieces follow the real shapes instead."""
    for body in list(spec.bodies):
        if not body.name.startswith(ARM_PREFIX):
            continue
        for geom in list(body.geoms):
            if geom.type != mujoco.mjtGeom.mjGEOM_MESH or not (geom.contype or geom.conaffinity):
                continue
            base = geom.meshname.removeprefix(ARM_PREFIX)
            pieces = sorted(COLLISION_PIECES_DIR.glob(f"{base}_*.stl"), key=lambda f: int(f.stem.rsplit("_", 1)[1]))
            if not pieces:
                continue
            for i, piece in enumerate(pieces):
                mesh_name = f"{ARM_PREFIX}col/{base}_{i}"
                if all(m.name != mesh_name for m in spec.meshes):
                    spec.add_mesh(name=mesh_name, file=str(piece))
                new = body.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_MESH,
                    meshname=mesh_name,
                    pos=list(geom.pos),
                    quat=list(geom.quat),
                    group=geom.group,
                    contype=geom.contype,
                    conaffinity=geom.conaffinity,
                    condim=geom.condim,
                    friction=list(geom.friction),
                )
                new.name = f"{ARM_PREFIX}col/{body.name.removeprefix(ARM_PREFIX)}_{base}_{i}"
            spec.delete(geom)


# Link pairs whose CAD meshes interpenetrate by 1-2 cm in the folded rest pose (the elbow sits at its limit there; this
# list was measured at the old firmware REST_POSE and still holds at the real start pose, which folds the same way).
# They can only touch each other in that folded pose, so their contact is switched off. Everything else (jaws vs items, arm vs shelf/tray/deck) is real contact.
STOWED_OVERLAPS = (
    ("shoulder", "wrist"),
    ("shoulder", "gripper"),
    ("shoulder", "moving_jaw_so101_v1"),
    ("shoulder", "lower_arm"),
    ("upper_arm", "wrist"),
    (None, "gripper"),  # the rover deck
    (None, "moving_jaw_so101_v1"),
)


def _exclude_stowed_overlaps(spec: mujoco.MjSpec) -> None:
    for a, b in STOWED_OVERLAPS:
        ex = spec.add_exclude()
        ex.name = f"stowed_{(a or 'deck')}_{b}"
        ex.bodyname1 = BASE if a is None else ARM_PREFIX + a
        ex.bodyname2 = ARM_PREFIX + b


GRIPPER_MAX_TORQUE = 0.8  # N*m. The STS3215 stalls at ~3 N*m; LeRobot's SO101Follower caps the gripper's
# Max_Torque_Limit / Protection_Current at a fraction of that so it does not crush objects or cook the servo.


def _limit_gripper_torque(spec: mujoco.MjSpec) -> None:
    act = spec.actuator(ARM_PREFIX + "gripper")
    act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
    act.forcerange = [-GRIPPER_MAX_TORQUE, GRIPPER_MAX_TORQUE]


def _grippy_jaws(spec: mujoco.MjSpec) -> None:
    """Rubber-like friction on the gripper's collision geoms so held items do not slip out."""
    for body_name in (ARM_PREFIX + "gripper", ARM_PREFIX + "moving_jaw_so101_v1"):
        body = spec.body(body_name)
        for geom in body.geoms:
            if geom.contype:
                geom.friction = [JAW_FRICTION, 0.02, 0.002]
                geom.condim = 4


def add_robot(spec: mujoco.MjSpec, dock: DockConfig, so101_xml: Path = SO101_XML) -> mujoco.MjsBody:
    """Add the full robot at the dock; returns the base body."""
    if not so101_xml.exists():
        raise AssetError(f"SO-101 model not found at {so101_xml}")
    mats = _Materials(spec)
    _register_meshes(spec)

    base = spec.worldbody.add_body(name=BASE, pos=[dock.pos[0], dock.pos[1], 0.0])
    for axis_name, axis in (("x", [1, 0, 0]), ("y", [0, 1, 0])):
        base.add_joint(name=f"robot/base_{axis_name}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=axis, damping=1.0)
    base.add_joint(name="robot/base_yaw", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], damping=0.05)

    for part in (*MODEL["chassis"], *MODEL["pi"]):
        geom = _add_part(base, part, mats, collide=part["name"] in COLLIDING)
        if part["name"] in COLLIDING:
            geom.mass = 0.4 if part["name"] == "battery" else 0.15
    _add_wheels(spec, base, mats)
    _velocity_actuator(spec, "robot/vx", "robot/base_x", BASE_KV, MAX_LINEAR)
    _velocity_actuator(spec, "robot/vy", "robot/base_y", BASE_KV, MAX_LINEAR)
    _velocity_actuator(spec, "robot/wz", "robot/base_yaw", 30.0, MAX_ANGULAR)

    child = mujoco.MjSpec.from_file(str(so101_xml))
    for key in list(child.keys):
        child.delete(key)
    absolutize_files(child, so101_xml.parent)
    root = child.body("base")
    if root is None:
        raise AssertionError(f"{so101_xml}: body 'base' not found")
    base.add_frame(pos=list(ARM_MOUNT)).attach_body(root, ARM_PREFIX, "")
    _add_wrist_camera(spec, mats)
    _recolor_arm(spec)
    _convex_collision(spec)
    _exclude_stowed_overlaps(spec)
    _limit_gripper_torque(spec)
    _grippy_jaws(spec)
    return base


def add_home_keyframe(spec: mujoco.MjSpec, model: mujoco.MjModel) -> None:
    """'home' keyframe: rover at the dock, arm folded in the rest pose."""
    qpos = list(model.qpos0)
    ctrl = [0.0] * model.nu
    for joint, deg in zip(ARM_JOINTS, POSES_DEG["rest"]):
        rad = float(np.deg2rad(deg))
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, ARM_PREFIX + joint)
        qpos[model.jnt_qposadr[jid]] = rad
        ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_PREFIX + joint)] = rad
    spec.add_key(name="home", qpos=qpos, ctrl=ctrl)
