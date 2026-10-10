"""The demo surface: a flat white table top and nothing around it (no walls, so views stay clear).

The table top is the world z = 0 plane. A far-below catch plane stops anything that falls off.
"""

from __future__ import annotations

import mujoco

from dsr_sim.config import Layout
from dsr_sim.materials import add_box, look_at_xyaxes

CATCH_PLANE_Z = -0.75


def configure_visuals(spec: mujoco.MjSpec) -> None:
    spec.visual.headlight.ambient = [0.4, 0.4, 0.4]
    spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    spec.visual.quality.shadowsize = 4096
    spec.visual.global_.offwidth = 1600
    spec.visual.global_.offheight = 1000
    spec.visual.map.znear = 0.01
    spec.stat.center = [0.0, 0.0, 0.1]
    spec.stat.extent = 1.5


def add_room(spec: mujoco.MjSpec, layout: Layout) -> None:
    configure_visuals(spec)
    table = layout.table
    half_x, half_y = table.size[0] / 2, table.size[1] / 2
    half_t = table.thickness / 2
    add_box(spec.worldbody, "table_top", (half_x, half_y, half_t), (0, 0, -half_t), "table_top")
    spec.worldbody.add_geom(name="catch_plane", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.05], pos=[0, 0, CATCH_PLANE_Z], material="floor")

    world = spec.worldbody
    world.add_light(name="key", pos=[0.4, -0.8, 2.0], dir=[-0.15, 0.3, -1], diffuse=[0.6, 0.6, 0.6], castshadow=True)
    world.add_light(name="fill", pos=[-0.6, 0.8, 1.6], dir=[0.2, -0.3, -1], diffuse=[0.3, 0.3, 0.3], castshadow=False)

    overview_pos = [0.0, -half_y * 3.0, 1.15]
    world.add_camera(name="overview", pos=overview_pos, xyaxes=look_at_xyaxes(overview_pos, [0.0, 0.05, 0.05]), fovy=58)
    world.add_camera(name="top_down", pos=[0, 0, 1.9], xyaxes=[1, 0, 0, 0, 1, 0], fovy=45)
