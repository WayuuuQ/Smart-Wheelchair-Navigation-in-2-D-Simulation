"""Controller baselines (M0-M3).

Unified interface:
    get_action(obs, user_cmd) -> action_dict

Where action_dict MUST contain:
    - v: float
    - omega: float

And MAY contain (for logging / future work):
    - u_h: (v, omega) user command
    - u_a: (v, omega) autonomous/local-planner command
    - alpha: float (assist authority, 0..1)
    - goal_probs: list[float] | dict | None
    - risk_terms: dict | None
    - dominant_risk: str | None
"""

from .factory import SUPPORTED_CONTROLLERS, make_controller

__all__ = ["SUPPORTED_CONTROLLERS", "make_controller"]

