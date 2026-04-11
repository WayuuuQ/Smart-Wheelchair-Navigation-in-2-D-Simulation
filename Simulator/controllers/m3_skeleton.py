from __future__ import annotations

from controller import SharedController

from .base import BaseController
from .local_planner import SimpleLocalPlanner


class M3SkeletonController(BaseController):
    """M3: adaptive blending based on front obstacle risk."""

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
        lidar = obs["lidar"]
        ctx = obs.get("control_context")
        analysis = self.shared._analyze_lidar(lidar)
        alpha = self.shared.get_adaptive_alpha(analysis["front_min"])
        v, omega = self.shared.blend_commands(u_h, u_a, alpha, lidar, ctx)
        v, omega = self._clip_action(v, omega)

        return {
            "v": v,
            "omega": omega,
            "u_h": u_h,
            "u_a": u_a,
            "alpha": float(alpha),
            "goal_probs": None,
            "risk_terms": {
                "d_min": float(analysis["d_min"]),
                "front_min": float(analysis["front_min"]),
                "left_min": float(analysis["left_min"]),
                "right_min": float(analysis["right_min"]),
            },
            "dominant_risk": "front_min",
            "implemented": True,
        }
