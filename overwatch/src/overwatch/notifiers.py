"""Outbound alert: one JSON POST per new incident to a configured webhook (for example
a UthaLe backend endpoint). Fire-and-forget on a thread so a slow receiver never
stalls the camera loop; the result is logged, and the incident is stored regardless."""

from __future__ import annotations

import logging
import threading

import httpx

log = logging.getLogger(__name__)


class WebhookNotifier:
    def __init__(self, url: str, token: str = ""):
        self.url, self.token = url, token

    @property
    def enabled(self) -> bool:
        return bool(self.url)

    def notify(self, incident: dict) -> None:
        if not self.enabled:
            return
        threading.Thread(target=self._post, args=(incident,), daemon=True).start()

    def _post(self, incident: dict) -> None:
        headers = {"Authorization": f"Bearer {self.token}"} if self.token else {}
        try:
            r = httpx.post(self.url, json={"source": "overwatch", "incident": incident},
                           headers=headers, timeout=5.0)
            if r.status_code >= 300:
                log.warning("Webhook %s answered HTTP %s", self.url, r.status_code)
        except httpx.HTTPError as e:
            log.warning("Webhook %s unreachable: %s", self.url, e)
