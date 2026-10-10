import time

import pytest
from fastapi.testclient import TestClient

from app.api.deps import Services
from app.config import settings
from app.database import Database
from app.main import create_app
from app.repository import Repository
from app.services import qc
from app.services.camera import CameraService
from app.services.catalog import load_catalog
from app.services.robot import SimulatedRobotAdapter, SimulationScenario
from app.services.sensors import SimulatedSensorHub
from app.services.workflow import WorkflowEngine

EXP = qc.Expected(sku="milk", quantity=1, destination="BAY-1", nominal_weight_g=1000)
GOOD = {"nfc": {"tag_read": True, "tag_sku": "milk"}, "load_cell": {"delta_g": 1010}}


# ---------------- QC rules ----------------
def test_qc_pass_requires_agreeing_evidence():
    assert qc.evaluate(EXP, "placed", "BAY-1", GOOD) == ("PASS", [])


def test_qc_self_report_alone_is_review():
    status, reasons = qc.evaluate(EXP, "placed", "BAY-1", {"nfc": None, "load_cell": None})
    assert status == "REVIEW" and "self-report" in reasons[0]


@pytest.mark.parametrize("sensors,expected", [
    ({"nfc": {"tag_read": True, "tag_sku": "tuna"}, "load_cell": {"delta_g": 1000}}, "FAIL"),
    ({"nfc": {"tag_read": False, "tag_sku": None}, "load_cell": {"delta_g": 1000}}, "REVIEW"),
    ({"nfc": {"tag_read": True, "tag_sku": "milk"}, "load_cell": {"delta_g": 1300}}, "REVIEW"),
    ({"nfc": {"tag_read": True, "tag_sku": "milk"}, "load_cell": {"delta_g": 3}}, "FAIL"),
])
def test_qc_flags(sensors, expected):
    assert qc.evaluate(EXP, "placed", "BAY-1", sensors)[0] == expected


def test_qc_robot_failure_and_wrong_bay_fail():
    assert qc.evaluate(EXP, "grasp_failed", None, GOOD)[0] == "FAIL"
    assert qc.evaluate(EXP, "placed", "BAY-2", GOOD)[0] == "FAIL"


# ---------------- end-to-end workflow ----------------
def make_client(db_path, anomaly_rate=0.0) -> tuple[TestClient, Services]:
    repo = Repository(Database(db_path))
    catalog = load_catalog(settings.inventory_path)
    camera = CameraService(index=99)  # never started: tests must not grab the real camera
    scenario = SimulationScenario(anomaly_rate, seed=1)
    robot = SimulatedRobotAdapter(scenario, speed=200)
    engine = WorkflowEngine(repo, catalog, robot, SimulatedSensorHub(scenario), camera, db_path.parent / "ev")
    svc = Services(repo=repo, camera=camera, engine=engine, catalog=catalog)
    return TestClient(create_app(svc)), svc


def wait_for(client, order_id, statuses, timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        order = client.get(f"/api/orders/{order_id}").json()
        if order["status"] in statuses:
            return order
        time.sleep(0.05)
    raise AssertionError(f"{order_id} stuck in {order['status']}")


def test_order_flows_to_dispatch_and_survives_restart(tmp_path):
    db = tmp_path / "t.db"
    client, svc = make_client(db)
    with client:
        assert len(client.get("/api/catalog").json()) >= 5
        r = client.post("/api/orders", json={"items": [{"sku": "milk", "quantity": 1},
                                                       {"sku": "cookies", "quantity": 2}],
                                             "destination": "BAY-2", "priority": "high"})
        assert r.status_code == 201, r.text
        oid = r.json()["id"]
        order = wait_for(client, oid, {"dispatched"})
        assert order["progress"] == 1.0
        assert [t["type"] for t in order["tasks"]][:6] == ["navigate", "identify", "pick", "transport", "place", "verify"]
        assert all(q["status"] == "PASS" and q["evidence_source"] == "simulated" for q in order["qc"])
        assert client.get("/api/overview").json()["orders_fulfilled"] == 1
        types = {a["type"] for a in client.get("/api/activity?limit=200").json()}
        assert {"order_created", "mission_started", "item_picked", "item_delivered", "qc_pass", "dispatched"} <= types
        assert client.get("/api/analytics").json()["totals"]["dispatched"] == 1

    client2, _ = make_client(db)
    with client2:
        assert client2.get(f"/api/orders/{oid}").json()["status"] == "dispatched"


def test_injected_anomaly_requires_human_review(tmp_path):
    client, _ = make_client(tmp_path / "t.db")
    with client:
        r = client.post("/api/robot/commands", json={"command": "inject_anomaly", "anomaly": "sku_mismatch"})
        assert r.json()["accepted"] and r.json()["simulated"]
        oid = client.post("/api/orders", json={"items": [{"sku": "tuna"}]}).json()["id"]
        order = wait_for(client, oid, {"verification"})
        time.sleep(0.2)
        order = client.get(f"/api/orders/{oid}").json()
        assert order["status"] == "verification" and order["flags"] == 1
        rec = order["qc"][0]
        assert rec["status"] == "FAIL"
        assert client.get("/api/qc?status=flagged").json()[0]["id"] == rec["id"]

        r = client.post(f"/api/qc/{rec['task_id']}/review", json={"decision": "approved", "reviewer": "Asha"})
        assert r.status_code == 200 and r.json()["review_decision"] == "approved"
        assert client.get(f"/api/orders/{oid}").json()["status"] == "dispatched"
        again = client.post(f"/api/qc/{rec['task_id']}/review", json={"decision": "rejected", "reviewer": "x"})
        assert again.status_code == 409


def test_grasp_failure_rejected_goes_to_exception(tmp_path):
    client, _ = make_client(tmp_path / "t.db")
    with client:
        client.post("/api/robot/commands", json={"command": "inject_anomaly", "anomaly": "grasp_failure"})
        oid = client.post("/api/orders", json={"items": [{"sku": "butter"}]}).json()["id"]
        wait_for(client, oid, {"verification"})
        order = client.get(f"/api/orders/{oid}").json()
        statuses = {t["type"]: t["status"] for t in order["tasks"]}
        assert statuses["pick"] == "failed" and statuses["place"] == "skipped"
        pick = next(t for t in order["tasks"] if t["type"] == "pick")
        assert pick["attempts"] == 2
        rec = order["qc"][0]
        client.post(f"/api/qc/{rec['task_id']}/review", json={"decision": "rejected", "reviewer": "Ravi"})
        assert client.get(f"/api/orders/{oid}").json()["status"] == "exception"


def test_stop_pause_and_validation(tmp_path):
    client, svc = make_client(tmp_path / "t.db")
    with client:
        assert client.post("/api/robot/commands", json={"command": "stop"}).json()["status"]["mode"] == "stopped"
        oid = client.post("/api/orders", json={"items": [{"sku": "milk"}]}).json()["id"]
        time.sleep(0.3)
        assert client.get(f"/api/orders/{oid}").json()["status"] == "queued"
        assert not client.post("/api/robot/commands", json={"command": "resume"}).json()["accepted"]
        client.post("/api/robot/commands", json={"command": "reset"})
        wait_for(client, oid, {"dispatched"})
        assert client.post("/api/orders", json={"items": [{"sku": "nope"}]}).status_code == 422
        assert client.get("/api/orders/ORD-1").status_code == 404
        assert client.get("/api/camera/stream").status_code == 503
