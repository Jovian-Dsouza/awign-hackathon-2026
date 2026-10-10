import mujoco
import pytest

from dsr_sim.constants import DEFAULT_SCENE
from dsr_sim.pick import Task
from dsr_sim.run import reset_scene
from dsr_sim.scenarios import SCENARIOS, resolve
from dsr_sim.world import ParamError, Params, item_position_problem, validate


def run(name: str, **kw) -> bool:
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    params = Params.from_dict(kw)
    validate(name, params)
    scenario = SCENARIOS[name]
    reset_scene(model, data, scenario, params)
    task = Task(model, data)
    task.run(0.5)
    start = data.time
    ok = scenario.run(task, params)
    assert data.time - start < 60.0  # no stage should be stuck waiting out a timeout
    return ok


def test_resolve_by_name_and_number():
    names = list(SCENARIOS)
    assert resolve("1").name == names[0]
    assert resolve(names[1]).name == names[1]
    with pytest.raises(KeyError):
        resolve("nope")


@pytest.mark.parametrize("rack", [[0, 2], [1, 0], [0, 0]])
def test_shelf_to_tray_picks_what_is_in_the_chosen_rack(rack):
    """The chosen rack's own item is the one that ends up in the tray; nothing is moved to the rack first."""
    from dsr_sim.world import item_at

    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    p = Params(rack=tuple(rack))
    sc = SCENARIOS["shelf_to_tray"]
    reset_scene(model, data, sc, p)
    others_before = {i.name: data.xpos[model.body(f"item_{i.name}").id].copy() for i in __import__("dsr_sim.world", fromlist=["INVENTORY"]).INVENTORY}
    task = Task(model, data)
    task.run(0.5)
    name = item_at(tuple(rack))
    start = task.item_pos(name)
    assert abs(data.xpos[model.body(f"item_{name}").id][2] - others_before[name][2]) < 0.01  # still in its own rack
    assert sc.run(task, p)
    tray = data.site_xpos[model.site("drop_zone").id]
    assert abs(task.item_pos(name)[0] - tray[0]) < 0.13 and start[2] != task.item_pos(name)[2]


@pytest.mark.parametrize("rack", [[1, 1], [2, 0]])
def test_flat_item_the_jaws_cannot_hold_is_reported_not_faked(rack):
    """The tape and the box cutter lie flat: nothing but friction holds items, so the run must fail honestly."""
    from dsr_sim.world import item_at

    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    p = Params(rack=tuple(rack))
    sc = SCENARIOS["shelf_to_tray"]
    reset_scene(model, data, sc, p)
    task = Task(model, data)
    task.run(0.5)
    name = item_at(tuple(rack))
    assert sc.run(task, p) is False
    assert task.item_pos(name)[1] > 0.15  # still on the shelf, not carried anywhere


def test_no_weld_or_ghost_jaws_exist():
    """Items are held by contact only: no weld equality in the model and every jaw geom collides with items."""
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    assert model.neq == 0
    jaw = [g for g in range(model.ngeom) if model.body(model.geom_bodyid[g]).name in ("so101/gripper", "so101/moving_jaw_so101_v1") and model.geom_contype[g]]
    assert jaw and all(model.geom_contype[g] & 1 and model.geom_conaffinity[g] & 1 for g in jaw)


def test_empty_rack_is_refused():
    with pytest.raises(ParamError, match="empty"):
        validate("shelf_to_tray", Params(rack=(2, 2)))


@pytest.mark.parametrize("xy", [[0.30, -0.02], [-0.6, 0.0], [0.5, -0.3]])
def test_restock_from_anywhere(xy):
    assert run("restock_misplaced", item_xy=xy)


def test_restock_into_other_empty_rack():
    assert run("restock_misplaced", item_xy=[0.2, -0.1], dest_rack=[1, 2])


def test_validation_messages():
    with pytest.raises(ParamError, match="tray"):
        validate("restock_misplaced", Params(item_xy=(0.8, 0.0)))
    with pytest.raises(ParamError, match="already holds"):
        validate("restock_misplaced", Params(item_xy=(0.0, 0.0), dest_rack=(1, 1)))
    with pytest.raises(ParamError, match="no rack"):
        validate("shelf_to_tray", Params(rack=(3, 0)))
    assert item_position_problem(0.3, -0.02) is None


def test_unreachable_spot_is_reported_not_attempted():
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    p = Params(item_xy=(1.0, 0.0))  # behind the tray: no stand-off exists
    sc = SCENARIOS["restock_misplaced"]
    reset_scene(model, data, sc, p)
    task = Task(model, data)
    assert sc.run(task, p) is False


def test_wrist_camera_never_rotates_underneath():
    """The bar camera must stay on top of the gripper (never swing below it or dip under the table) in every stage."""
    import mujoco
    import numpy as np

    from dsr_sim import run as RUN
    from dsr_sim.build_scene import build_spec
    from dsr_sim.config import load_inventory, load_layout
    from dsr_sim.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT
    from dsr_sim.pick import Task
    from dsr_sim.scenarios import SCENARIOS
    from dsr_sim.world import Params

    layout = load_layout(DEFAULT_LAYOUT)
    model = build_spec(layout, load_inventory(DEFAULT_INVENTORY, layout)).compile()
    grip = model.body("so101/gripper").id
    parts = [g for g in range(model.ngeom) if (model.geom(g).name or "").startswith("robot/wrist_cam_")]
    assert parts
    for name, params in (("shelf_to_tray", Params(rack=(1, 0))), ("restock_misplaced", Params(item_xy=(0.3, 0.25), dest_rack=(1, 1)))):
        data = mujoco.MjData(model)
        scenario = SCENARIOS[name]
        RUN.reset_scene(model, data, scenario, params)
        task = Task(model, data, None, 1.0, None)
        task.run(0.5)
        worst = {"up": 9.0, "z": 9.0}

        def watch(_task, data=data, worst=worst):
            rot = data.xmat[grip].reshape(3, 3)
            worst["up"] = min(worst["up"], float((rot @ np.array([0.0, 1.0, 0.0]))[2]))  # camera side, world-up part
            worst["z"] = min(worst["z"], float(min(data.geom_xpos[g][2] for g in parts)))

        task.observers.append(watch)
        assert scenario.run(task, params)
        assert worst["up"] > -0.15, f"{name}: camera swung underneath the gripper ({worst['up']:.2f})"
        assert worst["z"] > 0.0, f"{name}: camera went below the table surface ({worst['z']:.3f} m)"


def test_robot_does_not_pass_through_shelf_or_tray():
    """No robot part (arm, gripper, wrist camera) may sink more than a few mm into the shelf boards or the tray."""
    import mujoco
    import numpy as np

    from dsr_sim import run as RUN
    from dsr_sim.build_scene import build_spec
    from dsr_sim.config import load_inventory, load_layout
    from dsr_sim.constants import DEFAULT_INVENTORY, DEFAULT_LAYOUT
    from dsr_sim.pick import Task
    from dsr_sim.scenarios import SCENARIOS
    from dsr_sim.world import Params

    layout = load_layout(DEFAULT_LAYOUT)
    model = build_spec(layout, load_inventory(DEFAULT_INVENTORY, layout)).compile()

    def top(b):
        while model.body_parentid[b] != 0:
            b = model.body_parentid[b]
        return model.body(b).name

    robot = [g for g in range(model.ngeom) if top(model.geom_bodyid[g]).startswith(("robot", "so101")) and model.geom_size[g].max() > 0]
    env = [g for g in range(model.ngeom) if top(model.geom_bodyid[g]).startswith(("drop_tray", "shelf"))]
    for name, params in (("shelf_to_tray", Params(rack=(1, 0))), ("restock_misplaced", Params(item_xy=(-0.7, -0.3), dest_rack=(2, 0)))):
        data = mujoco.MjData(model)
        scenario = SCENARIOS[name]
        RUN.reset_scene(model, data, scenario, params)
        task = Task(model, data, None, 1.0, None)
        task.run(0.5)
        worst = [0.0]
        last = [-1.0]
        ft = np.zeros(6)

        def watch(_t, data=data, worst=worst, last=last, ft=ft):
            if data.time - last[0] < 0.1:
                return
            last[0] = data.time
            for rg in robot:
                for eg in env:
                    worst[0] = min(worst[0], mujoco.mj_geomDistance(model, data, rg, eg, 0.0, ft))

        task.observers.append(watch)
        assert scenario.run(task, params)
        assert worst[0] > -0.005, f"{name}: the robot sank {-worst[0] * 1000:.1f} mm into the shelf or tray"
