"""UthaLe backend: FastAPI app wiring."""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api import camera as camera_api
from app.api import operations
from app.api.deps import Services
from app.config import settings
from app.database import Database
from app.repository import Repository, now_iso
from app.services.camera import CameraService
from app.services.catalog import load_catalog
from app.services.robot import SimulatedRobotAdapter, SimulationScenario
from app.services.sensors import NullSensorHub, SimulatedSensorHub
from app.services.workflow import WorkflowEngine

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
log = logging.getLogger("uthale")


def build_services(db_path=None) -> Services:
    repo = Repository(Database(db_path or settings.db_path))
    catalog = load_catalog(settings.inventory_path)
    camera = CameraService(
        settings.camera_index, settings.resolved_camera_backend, settings.camera_width,
        settings.camera_height, settings.camera_fps, settings.camera_jpeg_quality, settings.camera_overlay,
    )
    if settings.robot_adapter != "simulated":
        raise RuntimeError(f"ROBOT_ADAPTER={settings.robot_adapter!r} is not implemented; only 'simulated' exists")
    scenario = SimulationScenario(settings.sim_anomaly_rate, settings.sim_seed)
    robot = SimulatedRobotAdapter(scenario, speed=settings.sim_speed)
    sensors = SimulatedSensorHub(scenario) if settings.sim_sensors else NullSensorHub()
    engine = WorkflowEngine(repo, catalog, robot, sensors, camera, settings.evidence_dir)
    return Services(repo=repo, camera=camera, engine=engine, catalog=catalog)


@asynccontextmanager
async def lifespan(app: FastAPI):
    s: Services = app.state.services
    if settings.camera_autostart:
        s.camera.start()
    await s.engine.start()
    s.repo.log("system", "UthaLe backend started (robot: simulated)")
    try:
        yield
    finally:
        await s.engine.shutdown()
        s.camera.shutdown()
        s.repo.db.close()
        log.info("Shutdown complete; camera released")


def create_app(svc: Services | None = None) -> FastAPI:
    app = FastAPI(title="UthaLe API", version="0.1.0", lifespan=lifespan)
    app.state.services = svc or build_services()
    app.add_middleware(
        CORSMiddleware, allow_origins=settings.cors_origins, allow_credentials=False,
        allow_methods=["GET", "POST", "PUT"], allow_headers=["*"],
    )

    @app.get("/api/health", tags=["health"])
    def health():
        s: Services = app.state.services
        return {"status": "ok", "time": now_iso(), "camera": s.camera.status()["state"],
                "robot_adapter": s.engine.robot.adapter_name, "simulated": s.engine.robot.simulated}

    app.include_router(camera_api.router)
    app.include_router(operations.router)
    return app


app = create_app()
