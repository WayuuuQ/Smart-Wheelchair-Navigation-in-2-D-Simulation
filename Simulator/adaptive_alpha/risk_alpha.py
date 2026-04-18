from __future__ import annotations

import math
from collections import deque


class AdaptiveAlphaModel:
    def __init__(
        self,
        safety_distance: float,
        stop_distance: float,
        alpha_min: float = 0.03,
        alpha_max: float = 0.8,
        sigmoid_gain: float = 6.0,
        sigmoid_center: float = 0.45,
        history_size: int = 8,
    ):
        self.safety_distance = float(safety_distance)
        self.stop_distance = float(stop_distance)
        self.alpha_min = float(alpha_min)
        self.alpha_max = float(alpha_max)
        self.sigmoid_gain = float(sigmoid_gain)
        self.sigmoid_center = float(sigmoid_center)
        self.user_omega_history = deque(maxlen=int(history_size))

    def _clamp01(self, value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def _sigmoid(self, x: float) -> float:
        return 1.0 / (1.0 + math.exp(-self.sigmoid_gain * (x - self.sigmoid_center)))

    def compute(self, user_cmd, assist_cmd, analysis: dict, control_context=None):
        user_v, user_omega = float(user_cmd[0]), float(user_cmd[1])
        assist_omega = float(assist_cmd[1])
        self.user_omega_history.append(user_omega)

        front_min = float(analysis["front_min"])
        d_min = float(analysis["d_min"])

        distance_risk = self._clamp01(
            (self.safety_distance - front_min) / max(1e-6, self.safety_distance - self.stop_distance)
        )
        if user_v > 1e-6:
            ttc = front_min / max(user_v, 1e-6)
            ttc_risk = self._clamp01((1.2 - ttc) / 1.2)
        else:
            ttc = None
            ttc_risk = 0.0

        corridor_risk = 0.0
        path_heading_error = 0.0
        if control_context is not None:
            corridor_risk = 1.0 if control_context.get("corridor_mode", False) else 0.0
            path_heading_error = abs(float(control_context.get("path_heading_error", 0.0)))

        mean_omega = sum(self.user_omega_history) / max(1, len(self.user_omega_history))
        instability = sum(abs(value - mean_omega) for value in self.user_omega_history) / max(1, len(self.user_omega_history))
        user_instability_risk = self._clamp01(instability / 1.2)

        steering_conflict = (
            abs(user_omega) > 0.1
            and abs(assist_omega) > 0.1
            and user_omega * assist_omega < 0.0
        )
        steering_conflict_risk = 1.0 if steering_conflict else 0.0
        heading_risk = self._clamp01(path_heading_error / 0.8)
        global_distance_risk = self._clamp01((self.safety_distance - d_min) / self.safety_distance)

        total_risk = (
            0.34 * distance_risk
            + 0.24 * ttc_risk
            + 0.14 * corridor_risk
            + 0.12 * user_instability_risk
            + 0.10 * steering_conflict_risk
            + 0.06 * heading_risk
        )
        total_risk = self._clamp01(total_risk)
        alpha = self.alpha_min + (self.alpha_max - self.alpha_min) * self._sigmoid(total_risk)

        risk_terms = {
            "d_min": d_min,
            "front_min": front_min,
            "ttc": ttc,
            "distance_risk": distance_risk,
            "ttc_risk": ttc_risk,
            "corridor_risk": corridor_risk,
            "user_instability_risk": user_instability_risk,
            "steering_conflict_risk": steering_conflict_risk,
            "heading_risk": heading_risk,
            "global_distance_risk": global_distance_risk,
            "total_risk": total_risk,
        }
        dominant_risk = max(
            (
                "distance_risk",
                "ttc_risk",
                "corridor_risk",
                "user_instability_risk",
                "steering_conflict_risk",
                "heading_risk",
            ),
            key=lambda key: risk_terms[key],
        )
        return alpha, risk_terms, dominant_risk
