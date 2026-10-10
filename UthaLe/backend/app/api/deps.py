from __future__ import annotations

from dataclasses import dataclass

from fastapi import Request

from app.models.schemas import CatalogItem
from app.repository import Repository
from app.services.camera import CameraService
from app.services.workflow import WorkflowEngine


@dataclass
class Services:
    repo: Repository
    camera: CameraService
    engine: WorkflowEngine
    catalog: dict[str, CatalogItem]


def services(request: Request) -> Services:
    return request.app.state.services
