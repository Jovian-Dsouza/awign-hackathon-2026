"""End to end through the real pipeline and HTTP API, with a fake camera."""

import dataclasses
import json
import time
import urllib.request

import cv2
import pytest
from fastapi.testclient import TestClient

from overwatch.app import create_app
from overwatch.config import Settings
from overwatch.events import Decision
from overwatch.responders import NullResponder

from conftest import FakeSource, textured_background, with_box


def wait_for(fn, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.05)
    raise AssertionError("timed out")


def make(tmp_path, source, **over):
    s = dataclasses.replace(Settings(), data_dir=tmp_path, analyze_fps=30.0, persist_s=0.4, grace_s=0.3, **over)
    return create_app(s, source=source, responder=NullResponder())


def start_watch(client, seconds=30):
    """The two things a person does: 'Start monitoring from here', and the robot starts working."""
    assert client.post("/api/setup").json()["monitoring"]["on"] is True
    assert client.post("/api/watch", json={"seconds": seconds}).json()["watching"] is True
    wait_for(lambda: client.get("/api/status").json()["alignment"])


def test_nothing_is_reported_when_the_robot_is_not_picking(tmp_path):
    bg = textured_background()
    src = FakeSource(bg)
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        src.set(with_box(bg, 300, 200, 40, 40))          # something appears and sits there...
        time.sleep(1.5)                                  # ...for ~45 frames, but nobody is picking
        st = client.get("/api/status").json()
        assert st["watch"]["watching"] is False
        assert client.get("/api/incidents").json() == []


def test_drop_during_a_pickup_to_admin_verdict_to_training_row(tmp_path):
    bg = textured_background()
    src = FakeSource(bg)
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        start_watch(client)
        src.set(with_box(bg, 300, 200, 40, 40))          # the robot drops something
        inc = wait_for(lambda: next(iter(client.get("/api/incidents?status=open").json()), None))
        assert inc["kind"] == "dropped_item" and inc["final"]["action"] == "propose_recovery"
        assert client.get(f"/api/incidents/{inc['id']}/snapshot").headers["content-type"] == "image/jpeg"

        body = client.post(f"/api/incidents/{inc['id']}/verdict", json={"verdict": "approve_recovery"}).json()
        assert body["status"] == "actioned" and body["responder"]["dry_run"] is True
        assert client.post(f"/api/incidents/{inc['id']}/verdict", json={"verdict": "dismiss"}).status_code == 409

        src.set(bg)                                      # item put back
        wait_for(lambda: client.get(f"/api/incidents/{inc['id']}").json()["cleared_ts"])
        rows = [json.loads(l) for l in client.get("/api/feedback.jsonl").text.splitlines() if l]
        assert len(rows) == 1 and rows[0]["expected"] == {"real_drop": True, "recoverable": True}


def test_one_incident_per_item_not_one_per_frame(tmp_path):
    bg = textured_background()
    src = FakeSource(bg)
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        start_watch(client)
        src.set(with_box(bg, 300, 200, 40, 40))
        wait_for(lambda: client.get("/api/incidents").json())
        time.sleep(1.5)
        assert len(client.get("/api/incidents").json()) == 1


def test_a_big_thing_like_the_robot_is_not_an_incident(tmp_path):
    bg = textured_background()
    src = FakeSource(bg)
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        start_watch(client)
        src.set(with_box(bg, 150, 120, 220, 220))        # ~16 % of the picture: robot-sized
        time.sleep(1.5)
        assert client.get("/api/incidents").json() == []
        assert client.get("/api/status").json()["ignored_changes"]["too_big"] >= 1


def test_each_new_pickup_takes_a_fresh_baseline(tmp_path):
    """Something already on the table before the pickup is not a drop."""
    bg = textured_background()
    src = FakeSource(with_box(bg, 300, 200, 40, 40))     # item already there
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        start_watch(client)
        time.sleep(1.5)
        assert client.get("/api/incidents").json() == []


def test_window_closes_after_the_pickup_plus_grace(tmp_path):
    src = FakeSource(textured_background())
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        client.post("/api/watch", json={"seconds": 0.5})
        assert client.get("/api/status").json()["watch"]["watching"]
        wait_for(lambda: client.get("/api/status").json()["watch"]["watching"] is False, timeout=5)


class _SureItIsFalse:
    available, error, load_s = True, None, 0.1

    def load(self):
        return True

    def decide(self, obs, ctx=None):
        return Decision("ignore", 0, False, ["laya: false alarm"], "laya", p_recoverable=0.1,
                        extra={"p_real_drop": 0.05})


def test_laya_filter_mode_hides_a_false_alarm_but_keeps_the_record(tmp_path):
    bg = textured_background()
    src = FakeSource(bg)
    s = dataclasses.replace(Settings(), data_dir=tmp_path, analyze_fps=30.0, persist_s=0.4, laya_mode="filter")
    with TestClient(create_app(s, source=src, responder=NullResponder(), laya=_SureItIsFalse())) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        start_watch(client)
        src.set(with_box(bg, 300, 200, 40, 40))
        filtered = wait_for(lambda: client.get("/api/incidents?status=filtered").json())
        assert filtered[0]["final"]["action"] == "ignore" and filtered[0]["rules"]["action"] == "propose_recovery"
        assert client.get("/api/incidents").json() == []                    # nothing alerts
        assert client.get("/api/status").json()["open_incidents"] == 0


def test_nothing_is_watched_until_someone_sets_the_position(tmp_path):
    """The robot and camera are not at fixed positions: what it sees at startup is not assumed right."""
    bg = textured_background()
    src = FakeSource(bg)
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        assert client.get("/api/status").json()["monitoring"]["on"] is False
        client.post("/api/watch", json={"seconds": 30})            # the robot is "working"...
        src.set(with_box(bg, 300, 200, 40, 40))                  # ...and something appears
        time.sleep(1.5)
        assert client.get("/api/incidents").json() == []         # ...but nobody said "start from here"


def test_setting_up_with_no_picture_is_refused(tmp_path):
    class Dead(FakeSource):
        def latest(self):
            return None
    with TestClient(make(tmp_path, Dead(textured_background()))) as client:
        assert client.post("/api/setup").status_code == 503


def test_moving_the_camera_asks_for_the_position_again(tmp_path):
    import numpy as np
    bg = textured_background()
    src = FakeSource(bg)
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        client.post("/api/setup")
        assert client.get("/api/status").json()["monitoring"]["view_moved"] is False
        src.set(np.roll(bg, 40, axis=1))                         # somebody nudges the camera (robot idle!)
        st = wait_for(lambda: (x := client.get("/api/status").json())["monitoring"]["view_moved"] and x)
        assert any("Set the position again" in w for w in st["camera_warnings"])
        assert client.get("/api/incidents").json() == []        # a moved camera is never an incident
        client.post("/api/setup")                                # press the button at the new position
        assert wait_for(lambda: client.get("/api/status").json()["monitoring"]["view_moved"] is False)


def test_something_taken_away_is_not_a_drop(tmp_path):
    """Picking an item off the table/rack leaves a gap: a change, but not a drop."""
    bg = textured_background()
    src = FakeSource(with_box(bg, 300, 200, 40, 40))             # the item is there when monitoring starts
    with TestClient(make(tmp_path, src)) as client:
        wait_for(lambda: client.get("/api/frame.jpg").status_code == 200)
        start_watch(client)
        src.set(bg)                                              # the robot takes it away
        time.sleep(1.8)
        assert client.get("/api/incidents").json() == []
        assert client.get("/api/status").json()["ignored_changes"]["left"] >= 1


def test_zone_validation_and_persistence(tmp_path):
    bg = textured_background()
    with TestClient(make(tmp_path, FakeSource(bg))) as client:
        assert client.put("/api/zones", json=[{"name": "x", "points": [[0, 0], [1, 1]]}]).status_code == 422
        good = [{"name": "table", "mode": "watch", "points": [[0, 0.4], [1, 0.4], [1, 1], [0, 1]]}]
        assert client.put("/api/zones", json=good).status_code == 200
    with TestClient(make(tmp_path, FakeSource(bg))) as client2:      # survives a restart
        assert [z["name"] for z in client2.get("/api/zones").json()] == ["table"]


def test_hold_is_a_dry_run_without_a_pi(tmp_path):
    with TestClient(make(tmp_path, FakeSource(textured_background()))) as client:
        r = client.post("/api/robot/hold").json()
        assert r["ok"] and r["dry_run"]


# ---- remote camera agent ingest ------------------------------------------------

def _jpeg(img):
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def _remote_app(tmp_path, token="s3cret"):
    s = dataclasses.replace(Settings(), data_dir=tmp_path, source="remote", ingest_token=token,
                            analyze_fps=30.0, persist_s=0.4)
    return create_app(s, responder=NullResponder())


def test_remote_agent_frames_reach_the_pipeline(tmp_path):
    bg = textured_background()
    with TestClient(_remote_app(tmp_path)) as client:
        st = client.get("/api/status").json()["camera"]
        assert st["state"] == "waiting" and st["agent_connected"] is False
        with client.websocket_connect("/api/ingest", headers={"Authorization": "Bearer s3cret"}) as ws:
            for _ in range(8):
                ws.send_bytes(_jpeg(bg))
            wait_for(lambda: client.get("/api/status").json()["camera"]["state"] == "streaming")
            cam = client.get("/api/status").json()["camera"]
            assert (cam["width"], cam["height"]) == (640, 480) and cam["agent_connected"] is True
            ws.send_bytes(b"not a jpeg")                       # garbage is dropped, connection survives
            for _ in range(5):
                ws.send_bytes(_jpeg(bg))
            time.sleep(0.2)
            assert client.post("/api/setup").json()["monitoring"]["on"] is True
            client.post("/api/watch", json={"seconds": 30})
            for _ in range(10):
                ws.send_bytes(_jpeg(bg))
                time.sleep(0.02)
            for _ in range(80):
                ws.send_bytes(_jpeg(with_box(bg, 300, 200, 40, 40)))
                time.sleep(0.02)
            inc = wait_for(lambda: next(iter(client.get("/api/incidents").json()), None))
            assert inc["kind"] == "dropped_item"
        wait_for(lambda: client.get("/api/status").json()["camera"]["agent_connected"] is False)


def test_ingest_refuses_wrong_or_missing_token(tmp_path):
    from starlette.websockets import WebSocketDisconnect
    with TestClient(_remote_app(tmp_path)) as client:
        for headers in ({}, {"Authorization": "Bearer nope"}):
            with pytest.raises(WebSocketDisconnect):
                with client.websocket_connect("/api/ingest", headers=headers):
                    pass


def test_ingest_refused_when_not_in_remote_mode_or_token_empty(tmp_path):
    from starlette.websockets import WebSocketDisconnect
    with TestClient(make(tmp_path, FakeSource(textured_background()))) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/api/ingest", headers={"Authorization": "Bearer x"}):
                pass
    with TestClient(_remote_app(tmp_path, token="")) as client:
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("/api/ingest", headers={"Authorization": "Bearer "}):
                pass


def test_second_agent_is_refused_while_one_is_connected(tmp_path):
    from starlette.websockets import WebSocketDisconnect
    hdr = {"Authorization": "Bearer s3cret"}
    with TestClient(_remote_app(tmp_path)) as client:
        with client.websocket_connect("/api/ingest", headers=hdr) as first:
            first.send_bytes(_jpeg(textured_background()))
            wait_for(lambda: client.get("/api/status").json()["camera"]["agent_connected"])
            with client.websocket_connect("/api/ingest", headers=hdr) as second:
                with pytest.raises(WebSocketDisconnect):
                    second.receive_bytes()


def test_ingest_works_over_a_real_socket(tmp_path):
    """TestClient talks to the ASGI app in-process, so it cannot notice a server that has no
    WebSocket library (plain `uvicorn` answers 404). This goes through a real uvicorn + socket."""
    import socket
    import threading

    import uvicorn
    from websockets.sync.client import connect

    with socket.socket() as sk:
        sk.bind(("127.0.0.1", 0))
        port = sk.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(_remote_app(tmp_path), host="127.0.0.1", port=port, log_level="warning"))
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    try:
        wait_for(lambda: server.started)
        with connect(f"ws://127.0.0.1:{port}/api/ingest", additional_headers={"Authorization": "Bearer s3cret"},
                     open_timeout=5) as ws:
            for _ in range(5):
                ws.send(_jpeg(textured_background()))

            def streaming():
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=3) as r:
                    return json.load(r)["camera"]["state"] == "streaming"
            wait_for(streaming)
    finally:
        server.should_exit = True
        t.join(5)


def test_no_camera_configured_refuses_to_guess(tmp_path):
    s = dataclasses.replace(Settings(), data_dir=tmp_path, source="")
    with pytest.raises(SystemExit, match="No camera configured"):
        create_app(s, responder=NullResponder())
