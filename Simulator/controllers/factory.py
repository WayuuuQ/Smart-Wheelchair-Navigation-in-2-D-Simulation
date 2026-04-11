from __future__ import annotations

from .base import ControllerLimits
from .fixed_blend import FixedBlendingController
from .m3_skeleton import M3SkeletonController
from .manual import ManualController
from .safety_filter import SafetyFilterController


SUPPORTED_CONTROLLERS = ("M0", "M1", "M2", "M3")


def normalize_controller_id(value: str) -> str:
    value = (value or "").strip()
    if not value:
        return "M0"
    upper = value.upper()
    if upper in SUPPORTED_CONTROLLERS:
        return upper
    # Allow legacy names.
    legacy = {
        "MANUAL": "M0",
        "SAFETY_ONLY": "M2",
        "FIXED_ALPHA": "M1",
        "ADAPTIVE_ALPHA": "M3",
    }
    return legacy.get(upper, upper)


def make_controller(controller_id: str, *, max_v: float, max_omega: float, **kwargs):
    cid = normalize_controller_id(controller_id)
    limits = ControllerLimits(max_v=float(max_v), max_omega=float(max_omega))

    if cid == "M0":
        return ManualController(limits)
    if cid == "M1":
        a0 = float(kwargs.get("a0", 0.6))
        return FixedBlendingController(limits, a0=a0)
    if cid == "M2":
        return SafetyFilterController(limits)
    if cid == "M3":
        return M3SkeletonController(limits)

    raise ValueError(
        f"Unknown controller '{controller_id}'. Supported: {', '.join(SUPPORTED_CONTROLLERS)}"
    )

