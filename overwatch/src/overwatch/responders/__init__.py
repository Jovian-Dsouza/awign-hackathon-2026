from overwatch.responders.base import NullResponder, Responder
from overwatch.responders.pi import PiResponder

__all__ = ["NullResponder", "PiResponder", "Responder", "build_responder"]


def build_responder(kind: str, pi_url: str = "", recover_traj: str = "") -> Responder:
    if kind == "pi":
        if not pi_url:
            raise ValueError("OVERWATCH_RESPONDER=pi needs OVERWATCH_PI_URL (e.g. http://raspberrypi.local:8003)")
        return PiResponder(pi_url, recover_traj)
    return NullResponder()
