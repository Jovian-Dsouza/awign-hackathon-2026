import pytest

from overwatch.arming import RobotActivityPoller, WatchWindow


def test_idle_window_is_closed():
    w = WatchWindow(grace_s=10, manual_s=60)
    s = w.state(now=100.0)
    assert s["watching"] is False and "not watching" in s["why"]


def test_manual_window_opens_then_closes_after_its_length():
    w = WatchWindow(grace_s=10, manual_s=60)
    w.arm_manual(30, now=100.0)
    assert w.state(now=101.0)["watching"] and w.state(now=129.0)["watching"]
    assert w.state(now=131.0)["watching"] is False


def test_each_opening_is_a_new_epoch_so_the_pipeline_rebaselines():
    w = WatchWindow(grace_s=0, manual_s=10)
    w.arm_manual(5, now=0.0)
    e1 = w.state(now=1.0)["epoch"]
    assert w.state(now=2.0)["epoch"] == e1                    # same window
    assert w.state(now=20.0)["watching"] is False
    w.arm_manual(5, now=30.0)
    assert w.state(now=31.0)["epoch"] == e1 + 1               # a new pickup


def test_robot_signal_keeps_it_open_and_grace_covers_a_late_drop():
    w = WatchWindow(grace_s=20, manual_s=60)
    w.set_signal("act", True, "driving", now=0.0)
    assert w.state(now=50.0)["watching"]
    w.set_signal("act", False, "", now=60.0)                  # the pickup ended
    assert w.state(now=75.0)["watching"]                      # 15 s later: a drop can still land
    assert w.state(now=81.0)["watching"] is False


def test_disarm_ends_manual_window_but_keeps_grace():
    w = WatchWindow(grace_s=5, manual_s=60)
    w.arm_manual(now=0.0)
    w.disarm(now=10.0)
    assert w.state(now=12.0)["watching"] and w.state(now=16.0)["watching"] is False


def test_poller_interprets_each_robot_source():
    pi = RobotActivityPoller._pi_active
    assert pi({"arm": {"mode": "play"}, "cmd": {}}) == (True, "arm trajectory playing")
    assert pi({"arm": {"mode": "idle"}, "cmd": {"vx": 0, "vy": 0, "wz": 0, "scale": 160}}) == (False, "")
    assert pi({"arm": {"mode": "idle"}, "cmd": {"vx": 120}}) == (True, "base driving")
    p = RobotActivityPoller(WatchWindow(), uthale_url="http://x:8000", pi_url="http://pi:8003", act_url="http://pi:8004")
    assert set(p.targets) == {"uthale", "pi", "act"}
    assert p.targets["uthale"][1]({"mode": "executing", "current_task_title": "Pick milk"}) == (True, "Pick milk")
    assert p.targets["uthale"][1]({"mode": "idle"})[0] is False
    assert p.targets["act"][1]({"mode": "driving"})[0] is True
    assert p.targets["act"][1]({"mode": "preview"})[0] is False
