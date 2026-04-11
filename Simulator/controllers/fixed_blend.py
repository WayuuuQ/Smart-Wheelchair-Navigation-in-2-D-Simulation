from __future__ import annotations

from controller import SharedController

from .base import BaseController
from .local_planner import SimpleLocalPlanner


class FixedBlendingController(BaseController):
    """M1: Fixed blending: u = a0*u_h + (1-a0)*u_a.

    Here u_a comes from a simple local planner.
    """

    controller_id = "M1"

    def __init__(self, limits, a0: float = 0.6):
        super().__init__(limits)
        self.a0 = float(a0)
        self.shared = SharedController(max_v=limits.max_v, max_omega=limits.max_omega)
        self.planner = SimpleLocalPlanner(self.shared)

    def get_action(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        u_h = (float(user_cmd[0]), float(user_cmd[1]))
        v_a, omega_a, _planner_out = self.planner.plan(obs)
        u_a = (v_a, omega_a)

        v = self.a0 * u_h[0] + (1.0 - self.a0) * u_a[0]
        omega = self.a0 * u_h[1] + (1.0 - self.a0) * u_a[1]
        v, omega = self._clip_action(v, omega)

        return {
            "v": v,
            "omega": omega,
            "u_h": u_h,
            "u_a": u_a,
            # Log alpha as the autonomous authority weight.
            "alpha": 1.0 - self.a0,
            "goal_probs": None,
            "risk_terms": None,
            "dominant_risk": None,
        }

