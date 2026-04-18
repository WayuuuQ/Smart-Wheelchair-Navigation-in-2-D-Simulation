from __future__ import annotations

from controller import SharedController
from adaptive_alpha.risk_alpha import AdaptiveAlphaModel
from explain.fields import build_explain_output

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
        self.alpha_model = AdaptiveAlphaModel(
            safety_distance=safety_distance,
            stop_distance=stop_distance,
        )

    def get_action(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        u_h = (float(user_cmd[0]), float(user_cmd[1]))
        v_a, omega_a, _planner_out = self.planner.plan(obs)
        u_a = (v_a, omega_a)
        lidar = obs["lidar"]
        ctx = obs.get("control_context")
        analysis = self.shared._analyze_lidar(lidar)
        alpha, risk_terms, dominant_risk = self.alpha_model.compute(
            u_h,
            u_a,
            analysis,
            control_context=ctx,
        )
        v, omega = self.shared.blend_commands(u_h, u_a, alpha, lidar, ctx)
        v, omega = self._clip_action(v, omega)
        explain_out = build_explain_output(
            alpha=alpha,
            risk_terms=risk_terms,
            dominant_risk=dominant_risk,
            goal_probs=None,
        )

        return {
            "v": v,
            "omega": omega,
            "u_h": u_h,
            "u_a": u_a,
            **explain_out,
            "implemented": True,
        }
