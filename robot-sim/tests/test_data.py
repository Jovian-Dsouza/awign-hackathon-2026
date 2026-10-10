import json

import numpy as np

from dsr_sim import datalog
from dsr_sim.constants import DEFAULT_SCENE
from dsr_sim.pick import Task
from dsr_sim.run import reset_scene
from dsr_sim.scenarios import SCENARIOS
from dsr_sim.world import Params

import mujoco


def test_recorded_run_writes_episode(tmp_path, monkeypatch):
    monkeypatch.setattr(datalog, "EPISODES_DIR", tmp_path / "episodes")
    monkeypatch.setattr(datalog, "RUNS_FILE", tmp_path / "runs.jsonl")
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    sc, p = SCENARIOS["shelf_to_tray"], Params(rack=(1, 0))
    reset_scene(model, data, sc, p)
    task = Task(model, data)
    task.run(0.5)
    assert datalog.run_recorded(task, sc, p, record_images=False)

    line = json.loads((tmp_path / "runs.jsonl").read_text().splitlines()[-1])
    assert line["success"] and line["params"]["rack"] == [1, 0] and line["item"] == "water_bottle"
    ep = next((tmp_path / "episodes").iterdir())
    meta = json.loads((ep / "meta.json").read_text())
    assert meta["stages"] and meta["constants"]["pinch_z_m"] > 0
    z = np.load(ep / "trajectory.npz")
    assert z["joints_deg"].shape[1] == 6 and len(z["t"]) > 100


def test_calibration_export(tmp_path):
    path = datalog.export_calibration(path=tmp_path / "sim.json")
    doc = json.loads(path.read_text())
    assert [j["name"] for j in doc["robot"]["arm"]["joints"]][0] == "shoulder_pan"
    assert len(doc["scene"]["shelf"]["rack_slot_xyz_m"]) == 3
