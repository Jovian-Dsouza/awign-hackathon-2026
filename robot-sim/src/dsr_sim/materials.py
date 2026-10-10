"""Shared textures/materials and small geometry helpers used by the scene builders."""

from __future__ import annotations

import math
from collections.abc import Sequence

import mujoco
import numpy as np

from dsr_sim.constants import COLORS, DECOR_GROUP


def add_materials(spec: mujoco.MjSpec) -> None:
    """Register one material per palette colour plus a textured concrete floor."""
    for name, rgba in COLORS.items():
        spec.add_material(name=name, rgba=list(rgba))

    spec.add_texture(
        name="floor_tex",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.55, 0.56, 0.58],
        rgb2=[0.50, 0.51, 0.53],
        mark=mujoco.mjtMark.mjMARK_EDGE,
        markrgb=[0.42, 0.43, 0.45],
        width=512,
        height=512,
    )
    floor = spec.add_material(name="floor", texrepeat=[10, 8], reflectance=0.0)
    floor.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor_tex"

    spec.add_texture(
        name="skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.85, 0.88, 0.92],
        rgb2=[0.35, 0.38, 0.42],
        width=512,
        height=512,
    )


def yaw_quat(yaw_deg: float) -> list[float]:
    half = math.radians(yaw_deg) / 2
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def euler_quat(euler_deg: Sequence[float]) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.radians(np.asarray(euler_deg, dtype=float)), "xyz")
    return quat


def quat_mul(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, dtype=float), np.asarray(b, dtype=float))
    return out


def rotate_xy(xy: Sequence[float], yaw_deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return np.array([c * xy[0] - s * xy[1], s * xy[0] + c * xy[1]])


def add_box(
    body: mujoco.MjsBody,
    name: str | None,
    half_size: Sequence[float],
    pos: Sequence[float],
    material: str,
    *,
    collide: bool = True,
    friction: Sequence[float] | None = None,
) -> mujoco.MjsGeom:
    """Add a box geom. Non-colliding boxes go into the decor group."""
    geom = body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=list(half_size),
        pos=list(pos),
        material=material,
    )
    if name:
        geom.name = name
    if friction is not None:
        geom.friction = list(friction)
    if not collide:
        geom.contype = 0
        geom.conaffinity = 0
        geom.group = DECOR_GROUP
    return geom


def look_at_xyaxes(pos: Sequence[float], target: Sequence[float]) -> list[float]:
    """Camera xyaxes for a camera at `pos` looking at `target` with +z up."""
    forward = np.asarray(target, dtype=float) - np.asarray(pos, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return [*right, *up]
