"""Warehouse room: floor, perimeter walls, lighting, floor tape and global cameras."""

from __future__ import annotations

import mujoco

from qc_warehouse.config import Layout
from qc_warehouse.materials import add_box, look_at_xyaxes

TAPE_WIDTH = 0.06
TAPE_HEIGHT = 0.001
LIGHT_HEIGHT = 4.0
AISLE_TAPE_OFFSET = 0.35  # distance of the safety line in front of the shelf faces


def configure_visuals(spec: mujoco.MjSpec) -> None:
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.45, 0.45, 0.45]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    spec.visual.quality.shadowsize = 4096
    spec.visual.global_.offwidth = 1600
    spec.visual.global_.offheight = 1000
    spec.visual.map.znear = 0.01
    spec.stat.center = [0.0, 0.5, 0.8]
    spec.stat.extent = 6.0


def _add_walls(spec: mujoco.MjSpec, layout: Layout) -> None:
    room = layout.room
    half_x, half_y = room.size[0] / 2, room.size[1] / 2
    half_t, half_h = room.wall_thickness / 2, room.wall_height / 2
    world = spec.worldbody
    add_box(world, "wall_north", (half_x + room.wall_thickness, half_t, half_h), (0, half_y + half_t, half_h), "wall")
    add_box(world, "wall_south", (half_x + room.wall_thickness, half_t, half_h), (0, -half_y - half_t, half_h), "wall")
    add_box(world, "wall_east", (half_t, half_y, half_h), (half_x + half_t, 0, half_h), "wall")
    add_box(world, "wall_west", (half_t, half_y, half_h), (-half_x - half_t, 0, half_h), "wall")


def _add_floor_tape(spec: mujoco.MjSpec, layout: Layout) -> None:
    """Yellow safety line along the shelf faces plus a green walkway to the station."""
    world = spec.worldbody
    shelf_front_y = min(s.pos[1] for s in layout.shelves) - layout.shelf.depth / 2 - AISLE_TAPE_OFFSET
    span_x = max(abs(s.pos[0]) for s in layout.shelves) + layout.shelf.width / 2 + 0.3
    add_box(world, "tape_shelf_line", (span_x, TAPE_WIDTH / 2, TAPE_HEIGHT), (0, shelf_front_y, TAPE_HEIGHT), "tape_yellow", collide=False)

    station = layout.station
    walk_top = shelf_front_y - 0.3
    walk_bottom = station.pos[1] + station.table_size[1] / 2 + 0.6
    half_len = (walk_top - walk_bottom) / 2
    if half_len > 0:
        for side in (-1, 1):
            x = station.pos[0] + side * 0.6
            add_box(world, f"tape_walkway_{'w' if side < 0 else 'e'}", (TAPE_WIDTH / 2, half_len, TAPE_HEIGHT), (x, walk_bottom + half_len, TAPE_HEIGHT), "tape_green", collide=False)


def _add_lights(spec: mujoco.MjSpec, layout: Layout) -> None:
    world = spec.worldbody
    world.add_light(name="sun", pos=[0, 0, LIGHT_HEIGHT + 2], dir=[0.2, 0.3, -1], diffuse=[0.5, 0.5, 0.5], castshadow=True)
    world.add_light(name="ceiling_station", pos=[layout.station.pos[0], layout.station.pos[1], LIGHT_HEIGHT], dir=[0, 0, -1], diffuse=[0.3, 0.3, 0.3], castshadow=False)
    for shelf in layout.shelves:
        world.add_light(
            name=f"ceiling_{shelf.name}",
            pos=[shelf.pos[0], shelf.pos[1] - 1.5, LIGHT_HEIGHT],
            dir=[0, 0.5, -1],
            diffuse=[0.25, 0.25, 0.25],
            castshadow=False,
        )


def add_room(spec: mujoco.MjSpec, layout: Layout) -> None:
    configure_visuals(spec)
    half_x, half_y = layout.room.size[0] / 2, layout.room.size[1] / 2
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[half_x, half_y, 0.05], material="floor")
    _add_walls(spec, layout)
    _add_floor_tape(spec, layout)
    _add_lights(spec, layout)

    overview_pos = [half_x * 0.75, -half_y * 0.95, 4.2]
    spec.worldbody.add_camera(name="overview", pos=overview_pos, xyaxes=look_at_xyaxes(overview_pos, [0, 0.8, 0.4]), fovy=60)
    top_pos = [0, 0, 11.0]
    spec.worldbody.add_camera(name="top_down", pos=top_pos, xyaxes=[1, 0, 0, 0, 1, 0], fovy=45)
