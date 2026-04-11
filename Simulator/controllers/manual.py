from __future__ import annotations

from .base import BaseController


class ManualController(BaseController):
    """M0: Manual control: u = u_h."""

    controller_id = "M0"

    def get_action(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        v, omega = self._clip_action(user_cmd[0], user_cmd[1])
        return {
            "v": v,
            "omega": omega,
            "u_h": (float(user_cmd[0]), float(user_cmd[1])),
            "u_a": None,
            "alpha": 0.0,
            "goal_probs": None,
            "risk_terms": None,
            "dominant_risk": None,
        }

