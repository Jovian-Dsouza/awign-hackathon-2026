"""RGB-D perception from the Stretch head camera (Intel D435i).

Identification uses the simulator's segmentation render as an oracle
instance-segmentation model: each pixel tells which geom is visible, which maps
to an item. Depth is back-projected to 3-D points, which give the item's
position and size in the world. Swapping in a learned detector only means
replacing `item_masks`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
MIN_PIXELS = 40           # fewer visible pixels than this = not identified
LOW_PCT, HIGH_PCT = 3, 97  # robust extents (ignore depth-edge outliers)


@dataclass(frozen=True)
class CameraFrame:
    rgb: np.ndarray        # (H, W, 3) uint8
    depth: np.ndarray      # (H, W) metres along the optical axis
    seg_geom: np.ndarray   # (H, W) geom id per pixel, -1 for none
    cam_pos: np.ndarray    # (3,) world
    cam_mat: np.ndarray    # (3, 3) world-from-camera (MuJoCo: x right, y up, looks along -z)
    fovy_deg: float


@dataclass(frozen=True)
class Detection:
    name: str
    pixels: int
    center: np.ndarray     # (3,) world: centre of the visible bounding box
    min_xyz: np.ndarray
    max_xyz: np.ndarray

    @property
    def size(self) -> np.ndarray:
        return self.max_xyz - self.min_xyz


def backproject(depth: np.ndarray, mask: np.ndarray, cam_pos: np.ndarray, cam_mat: np.ndarray, fovy_deg: float) -> np.ndarray:
    """World-frame 3-D points for masked pixels of a MuJoCo pinhole camera."""
    height, width = depth.shape
    focal = (height / 2) / math.tan(math.radians(fovy_deg) / 2)
    v, u = np.nonzero(mask)
    d = depth[v, u]
    x_cam = (u + 0.5 - width / 2) / focal * d
    y_cam = -(v + 0.5 - height / 2) / focal * d
    points_cam = np.stack([x_cam, y_cam, -d], axis=1)
    return points_cam @ cam_mat.T + cam_pos


class HeadCamera:
    """Renders RGB, depth and segmentation from a named camera."""

    def __init__(self, model: mujoco.MjModel, camera: str, width: int = CAMERA_WIDTH, height: int = CAMERA_HEIGHT):
        self._model = model
        self._cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        if self._cam_id < 0:
            raise ValueError(f"camera {camera!r} not in model")
        self._renderer = mujoco.Renderer(model, height, width)

    def close(self) -> None:
        self._renderer.close()

    def capture(self, data: mujoco.MjData, pose_data: mujoco.MjData | None = None) -> CameraFrame:
        """Render from the physical state `data`; report the camera pose from `pose_data`
        (the robot's estimated kinematics) so detections land in the robot's own frame."""
        pose_data = data if pose_data is None else pose_data
        r = self._renderer
        r.update_scene(data, camera=self._cam_id)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(data, camera=self._cam_id)
        depth = r.render().copy()
        r.disable_depth_rendering()
        r.enable_segmentation_rendering()
        r.update_scene(data, camera=self._cam_id)
        seg = r.render().copy()
        r.disable_segmentation_rendering()
        seg_geom = np.where(seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM, seg[..., 0], -1)
        return CameraFrame(
            rgb=rgb,
            depth=depth,
            seg_geom=seg_geom,
            cam_pos=pose_data.cam_xpos[self._cam_id].copy(),
            cam_mat=pose_data.cam_xmat[self._cam_id].reshape(3, 3).copy(),
            fovy_deg=float(self._model.cam_fovy[self._cam_id]),
        )


def item_masks(model: mujoco.MjModel, frame: CameraFrame, item_bodies: dict[str, int]) -> dict[str, np.ndarray]:
    """Per-item pixel masks (only items with at least one visible pixel)."""
    geom_root = model.body_rootid[model.geom_bodyid]
    root_per_pixel = np.where(frame.seg_geom >= 0, geom_root[np.clip(frame.seg_geom, 0, None)], -1)
    masks = {}
    for name, body_id in item_bodies.items():
        mask = root_per_pixel == body_id
        if mask.any():
            masks[name] = mask
    return masks


def detect(frame: CameraFrame, name: str, mask: np.ndarray) -> Detection | None:
    """Localise one item from its mask; None if too few pixels to trust."""
    pixels = int(mask.sum())
    if pixels < MIN_PIXELS:
        return None
    points = backproject(frame.depth, mask, frame.cam_pos, frame.cam_mat, frame.fovy_deg)
    lo = np.percentile(points, LOW_PCT, axis=0)
    hi = np.percentile(points, HIGH_PCT, axis=0)
    return Detection(name=name, pixels=pixels, center=(lo + hi) / 2, min_xyz=lo, max_xyz=hi)


def identify_items(model: mujoco.MjModel, frame: CameraFrame, item_bodies: dict[str, int]) -> dict[str, Detection]:
    """All confidently visible items in the frame, keyed by name."""
    detections = {}
    for name, mask in item_masks(model, frame, item_bodies).items():
        detection = detect(frame, name, mask)
        if detection is not None:
            detections[name] = detection
    return detections
