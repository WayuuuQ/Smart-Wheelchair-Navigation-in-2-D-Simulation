from __future__ import annotations

from controller import SharedController

from .base import BaseController
from .local_planner import SimpleLocalPlanner


class M3SkeletonController(BaseController):
    """M3 skeleton.

    Placeholder only: reserves fields for adaptive alpha, intent inference,
    and explainable outputs. For now it returns u = u_h (manual), while also
    computing u_a to make wiring and logging trivial.
    """

    controller_id = "M3"

    def __init__(
        self,
        limits,
        safety_distance: float = 75.0,
        stop_distance: float = 35.0,
    ):
        super().__init__(limits)
        self.shared = SharedController(
            max_v=limits.max_v,
            max_omega=limits.max_omega,
            safety_distance=safety_distance,
            stop_distance=stop_distance,
        )
        self.planner = SimpleLocalPlanner(self.shared)

    def get_action(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        u_h = (float(user_cmd[0]), float(user_cmd[1]))
        v_a, omega_a, _planner_out = self.planner.plan(obs)
        u_a = (v_a, omega_a)

        v, omega = self._clip_action(u_h[0], u_h[1])
        return {
            "v": v,
            "omega": omega,
            "u_h": u_h,
            "u_a": u_a,
            # Reserved fields for Part C.
            "alpha": 0.0,
            "goal_probs": None,
            "risk_terms": {},
            "dominant_risk": None,
            "implemented": False,
        }
