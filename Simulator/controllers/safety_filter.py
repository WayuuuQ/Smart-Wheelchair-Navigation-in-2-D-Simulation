from __future__ import annotations

from controller import SharedController

from .base import BaseController


class SafetyFilterController(BaseController):
    """M2: Safety filter only: u = SafetyFilter(u_h)."""

    controller_id = "M2"

    def __init__(self, limits):
        super().__init__(limits)
        self.shared = SharedController(max_v=limits.max_v, max_omega=limits.max_omega)

    def get_action(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        u_h = (float(user_cmd[0]), float(user_cmd[1]))
        lidar = obs["lidar"]
        ctx = obs.get("control_context")
        out = self.shared.apply_safety_filter(u_h, lidar, ctx)
        v, omega = self._clip_action(out["v"], out["omega"])

        # Expose a minimal risk_terms dict so logging can pick it up.
        risk_terms = {
            "d_min": float(out.get("d_min", 0.0)),
            "front_min": float(out.get("front_min", 0.0)),
            "left_min": float(out.get("left_min", 0.0)),
            "right_min": float(out.get("right_min", 0.0)),
        }

        return {
            "v": v,
            "omega": omega,
            "u_h": u_h,
            "u_a": None,
            "alpha": 0.0,
            "goal_probs": None,
            "risk_terms": risk_terms,
            "dominant_risk": "d_min",
        }

