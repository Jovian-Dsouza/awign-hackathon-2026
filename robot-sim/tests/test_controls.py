import threading
import time

import mujoco
import pytest

from dsr_sim.constants import DEFAULT_SCENE
from dsr_sim.pick import Closed, Restart, RunControl, Task


class FakeViewer:
    def __init__(self):
        self.running = True
        self.syncs = 0

    def is_running(self):
        return self.running

    def sync(self):
        self.syncs += 1


@pytest.fixture
def setup():
    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    viewer, control = FakeViewer(), RunControl(speed=8.0)
    return Task(model, data, viewer, 8.0, None, control), viewer, control


def test_space_pauses_then_resumes(setup):
    task, viewer, control = setup
    control.on_key(32)  # space
    assert control.paused
    threading.Timer(0.4, lambda: control.on_key(32)).start()
    t0, sim0 = time.time(), task.data.time
    task.run(0.05)
    assert time.time() - t0 >= 0.35  # blocked while paused
    assert task.data.time - sim0 >= 0.05  # then ran


def test_pause_does_not_advance_time(setup):
    task, viewer, control = setup
    control.paused = True
    stop = threading.Timer(0.3, lambda: setattr(control, "paused", False))
    stop.start()
    sim0 = task.data.time
    th = threading.Thread(target=lambda: task.run(0.02))
    th.start()
    time.sleep(0.15)
    assert task.data.time == sim0  # frozen while paused
    th.join()


def test_reset_key_raises_restart(setup):
    task, viewer, control = setup
    control.on_key(82)  # R
    with pytest.raises(Restart):
        task.run(1.0)


def test_backspace_and_rewound_clock_both_reset(setup):
    task, viewer, control = setup
    control.on_key(259)
    assert control.reset_requested
    control.reset_requested = False
    task.run(0.1)
    task.data.time = 0.0  # the viewer's Reset button rewinds time
    with pytest.raises(Restart):
        task.run(0.1)


def test_closed_window_stops_the_run(setup):
    task, viewer, control = setup
    viewer.running = False
    with pytest.raises(Closed):
        task.run(1.0)


def test_speed_keys(setup):
    _, _, control = setup
    control.speed = 1.0
    control.on_key(93)
    assert control.speed == pytest.approx(1.5)
    control.on_key(91)
    assert control.speed == pytest.approx(1.0)
