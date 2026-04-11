from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ControllerLimits:
    max_v: float
    max_omega: float


class BaseController:
    """Base class for all controllers.

    Controllers should be pure with respect to inputs, except for internal state
    (e.g., intent filters) used for temporal consistency.
    """

    controller_id: str = "BASE"

    def __init__(self, limits: ControllerLimits):
        self.limits = limits

    def _clip(self, value: float, limit: float) -> float:
        return max(-float(limit), min(float(limit), float(value)))

    def _clip_action(self, v: float, omega: float) -> tuple[float, float]:
        return (
            self._clip(v, self.limits.max_v),
            self._clip(omega, self.limits.max_omega),
        )

    def get_action(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        raise NotImplementedError

