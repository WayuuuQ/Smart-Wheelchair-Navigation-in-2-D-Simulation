"""Smoke tests for controllers (no pygame window).

Run from repo root:
    python Simulator/controllers/smoke_test_controllers.py

These tests validate:
  - Unified interface: get_action(obs, user_cmd) -> dict
  - Required fields exist: v, omega, u_h, u_a, alpha
  - Basic behavioral expectations:
      M0: exec ~= user_cmd (within clipping)
      M1: exec is fixed blend between user_cmd and u_a (a0=0.6)
      M2: with near obstacle, should reduce/stop forward speed and steer
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np


def _ensure_import_path():
    # When executed as a script, sys.path[0] is this folder; add Simulator/ to import `controllers`.
    this_dir = os.path.dirname(os.path.abspath(__file__))
    simulator_dir = os.path.dirname(this_dir)
    if simulator_dir not in sys.path:
        sys.path.insert(0, simulator_dir)


def _make_lidar(num_beams=31, fov_deg=180.0, ranges=None):
    angles = np.linspace(
        -math.radians(fov_deg) / 2.0,
        math.radians(fov_deg) / 2.0,
        num_beams,
        dtype=float,
    )
    if ranges is None:
        ranges = np.full((num_beams,), 200.0, dtype=float)
    else:
        ranges = np.asarray(ranges, dtype=float)
        assert ranges.shape == (num_beams,)
    return {"ranges": ranges, "angles": angles}


def _assert_action_fields(action: dict):
    for k in ("v", "omega", "u_h", "u_a", "alpha"):
        assert k in action, f"missing field: {k}"
    assert isinstance(action["v"], (int, float))
    assert isinstance(action["omega"], (int, float))
    assert isinstance(action["alpha"], (int, float))


def test_m0_manual(make_controller):
    c = make_controller("M0", max_v=120.0, max_omega=2.5)
    obs = {
        "pose": (0.0, 0.0, 0.0),
        "goal": (100.0, 0.0),
        "lidar": _make_lidar(),
        "control_context": None,
    }
    user_cmd = (50.0, 0.5)
    action = c.get_action(obs, user_cmd)
    _assert_action_fields(action)
    assert abs(action["v"] - user_cmd[0]) < 1e-9
    assert abs(action["omega"] - user_cmd[1]) < 1e-9
    assert action["u_a"] is None
    assert abs(action["alpha"] - 0.0) < 1e-9


def test_m1_fixed_blend(make_controller):
    a0 = 0.6
    c = make_controller("M1", max_v=120.0, max_omega=2.5, a0=a0)
    obs = {
        "pose": (0.0, 0.0, 0.0),
        "goal": (150.0, 0.0),
        "lidar": _make_lidar(),
        "control_context": None,
    }
    user_cmd = (40.0, 0.2)
    action = c.get_action(obs, user_cmd)
    _assert_action_fields(action)
    assert action["u_a"] is not None, "M1 should provide u_a"
    u_a = action["u_a"]
    v_expected = a0 * user_cmd[0] + (1.0 - a0) * u_a[0]
    omega_expected = a0 * user_cmd[1] + (1.0 - a0) * u_a[1]
    assert abs(action["v"] - v_expected) < 1e-9
    assert abs(action["omega"] - omega_expected) < 1e-9
    assert abs(action["alpha"] - (1.0 - a0)) < 1e-9


def test_m2_safety_filter(make_controller):
    c = make_controller("M2", max_v=120.0, max_omega=2.5)

    # Create a near obstacle in front, but slightly closer on the left-front rays than the
    # right-front rays, so left_min != right_min and avoidance omega is non-zero.
    num_beams = 31
    ranges = np.full((num_beams,), 200.0, dtype=float)
    center = num_beams // 2
    ranges[center - 2 : center + 3] = 25.0  # front blocked (< stop_distance=35)
    ranges[center + 1 : center + 3] = 15.0  # left-front even closer => turn away

    obs = {
        "pose": (0.0, 0.0, 0.0),
        "goal": (150.0, 0.0),
        "lidar": _make_lidar(num_beams=num_beams, ranges=ranges),
        "control_context": None,
    }
    user_cmd = (60.0, 0.0)  # try to drive forward into blockage
    action = c.get_action(obs, user_cmd)
    _assert_action_fields(action)

    # Expect strong speed reduction / stop when front is critically blocked.
    assert action["v"] <= 1e-6, f"expected stop, got v={action['v']}"

    # Expect some steering correction (avoidance omega).
    assert abs(action["omega"]) > 1e-3, f"expected turn, got omega={action['omega']}"


def test_m3_skeleton(make_controller):
    c = make_controller("M3", max_v=120.0, max_omega=2.5)
    obs = {
        "pose": (0.0, 0.0, 0.0),
        "goal": (150.0, 0.0),
        "lidar": _make_lidar(),
        "control_context": None,
    }
    user_cmd = (30.0, -0.1)
    action = c.get_action(obs, user_cmd)
    _assert_action_fields(action)
    assert action["u_a"] is not None, "M3 skeleton should compute u_a for wiring/logging"
    assert "goal_probs" in action
    assert "risk_terms" in action


def main():
    _ensure_import_path()
    from controllers.factory import make_controller

    tests = [
        test_m0_manual,
        test_m1_fixed_blend,
        test_m2_safety_filter,
        test_m3_skeleton,
    ]
    for t in tests:
        t(make_controller)
    print(f"OK: {len(tests)} controller smoke tests passed.")


if __name__ == "__main__":
    main()
