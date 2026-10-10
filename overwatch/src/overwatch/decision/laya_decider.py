"""Laya (https://laya.convaiinnovations.com/) as the fast typed-decision brain.

Laya reads text/JSON and answers typed questions (choice / score / noul) in one forward
pass with calibrated probabilities. It cannot see images, so it is given the camera
side's structured Observation and answers the three questions in `questions.py`.

Heavy imports (torch, transformers) happen lazily on first use, so the rest of the
service runs without them. If Laya cannot load, `available` is False and `error` says
why; the service keeps running on the rules floor.

Laya's job here: given a candidate drop, decide whether it is a real drop or a false alarm,
and whether the arm could recover it. It reads the detector's measurements, not pixels.
Its docs warn that zero-shot accuracy on typed decisions is close to random, so measure it
(`GET /api/feedback/summary` compares it with the admin's verdicts) before trusting it.
"""

from __future__ import annotations

import logging
import threading
import time

from overwatch.decision.questions import QUESTIONS, action_from_answers
from overwatch.events import Decision, Observation

log = logging.getLogger(__name__)


class LayaDecider:
    name = "laya"

    def __init__(self, model: str = "typed-decisions", path: str = ""):
        self.model, self.path = model, path
        self.available = False
        self.error: str | None = None
        self.load_s: float | None = None
        self._router = None
        self._lock = threading.Lock()

    def load(self) -> bool:
        """Load the checkpoint. First run downloads it from Hugging Face (~0.8 GB)."""
        with self._lock:
            if self._router is not None:
                return True
            t0 = time.time()
            try:
                if self.path:
                    import laya
                    self._router = laya.load(self.path)
                else:
                    from laya import Router
                    self._router = Router()
            except Exception as e:  # missing package, no network, bad path: stay on rules
                self.available, self.error = False, f"{type(e).__name__}: {e}"
                log.warning("Laya unavailable: %s", self.error)
                return False
            try:  # Router/Agent load weights lazily: do it now so load_s is real and incident #1 is fast
                self._predict({"event": "warmup"})
            except Exception as e:
                self._router, self.available, self.error = None, False, f"{type(e).__name__}: {e}"
                log.warning("Laya warm-up failed: %s", self.error)
                return False
            self.load_s = round(time.time() - t0, 1)
            self.available, self.error = True, None
            log.info("Laya ready (%s) in %.1fs", self.path or self.model, self.load_s)
            return True

    def decide(self, obs: Observation, ctx: dict | None = None) -> Decision | None:
        if not self.load():
            return None
        # Exactly Observation.to_state(): the same shape store.training_rows() exports,
        # so what Laya is fine-tuned on is what it is later asked about.
        state = obs.to_state()
        t0 = time.time()
        try:
            res = self._predict(state)
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            log.warning("Laya predict failed: %s", self.error)
            return None
        ms = round((time.time() - t0) * 1000, 1)
        return self._to_decision(res.get("answers", {}), ms)

    def _predict(self, state: dict) -> dict:
        if self.path:
            return self._router.predict(state, QUESTIONS)
        return self._router.predict(state, QUESTIONS, model=self.model)

    @staticmethod
    def _to_decision(ans: dict, latency_ms: float) -> Decision | None:
        try:
            p_real = float(ans["real_drop"]["noul"])
            p_rec = float(ans["recoverable"]["noul"])
        except (KeyError, TypeError, ValueError):
            return None
        action = action_from_answers(p_real >= 0.5, p_rec >= 0.5)
        conf = round(min(ans["real_drop"].get("confidence", 0.5), ans["recoverable"].get("confidence", 0.5)), 3)
        return Decision(
            action=action, severity=1 if action != "ignore" else 0, recoverable=p_rec >= 0.5,
            p_recoverable=round(p_rec, 3), confidence=conf, latency_ms=latency_ms,
            reasons=[f"Laya: P(real drop)={p_real:.0%}, P(recoverable)={p_rec:.0%} -> {action}"],
            decider="laya", extra={"p_real_drop": round(p_real, 3)},
        )
