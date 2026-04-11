from __future__ import annotations

from controller import SharedController


class SimpleLocalPlanner:
    """Thin wrapper around the existing SharedController goal-seeking assist."""

    def __init__(self, shared: SharedController):
        self.shared = shared

    def plan(self, obs: dict) -> tuple[float, float, dict]:
        pose = obs["pose"]
        goal = obs["goal"]
        lidar = obs["lidar"]
        ctx = obs.get("control_context")
        out = self.shared.compute_assist_control(pose, goal, lidar, ctx)
        return float(out["v"]), float(out["omega"]), out

