import math

import numpy as np


class SharedController:
    def __init__(
        self,
        max_v=120.0,
        max_omega=2.5,
        safety_distance=75.0,
        stop_distance=35.0,
        front_sector_deg=35.0,
        side_sector_deg=90.0,
        goal_tolerance=20.0,
        fixed_alpha=0.18,
    ):
        self.max_v = max_v
        self.max_omega = max_omega
        self.safety_distance = float(safety_distance)
        self.stop_distance = float(stop_distance)
        self.front_sector_rad = math.radians(front_sector_deg)
        self.side_sector_rad = math.radians(side_sector_deg)
        self.goal_tolerance = float(goal_tolerance)
        self.fixed_alpha = float(fixed_alpha)
        self.avoid_commit_steps = 0
        self.avoid_commit_sign = 0.0
        self.recovery_commit_steps = 0
        self.recovery_commit_sign = 0.0

    def _clip(self, value, limit):
        return max(-limit, min(limit, value))

    def _is_recovery_command(self, user_cmd, control_context=None):
        if control_context is not None and control_context.get("recovery_active"):
            return True
        v_cmd, omega_cmd = user_cmd
        return v_cmd <= 0.0 and abs(omega_cmd) >= 2.0

    def _is_corridor_mode(self, control_context=None):
        return control_context is not None and control_context.get("corridor_mode", False)

    def _compute_forward_scale(self, front_min, safety_distance=None, stop_distance=None):
        effective_safety = self.safety_distance if safety_distance is None else safety_distance
        effective_stop = self.stop_distance if stop_distance is None else stop_distance

        if front_min <= effective_stop:
            return 0.0
        if front_min >= effective_safety:
            return 1.0

        span = max(1e-6, effective_safety - effective_stop)
        return (front_min - effective_stop) / span

    def _analyze_lidar(self, sensor_data):
        ranges = np.asarray(sensor_data["ranges"], dtype=float)
        angles = np.asarray(sensor_data["angles"], dtype=float)

        front_mask = np.abs(angles) <= self.front_sector_rad
        left_mask = (angles > 0.0) & (angles <= self.side_sector_rad)
        right_mask = (angles < 0.0) & (angles >= -self.side_sector_rad)

        front_min = (
            float(np.min(ranges[front_mask]))
            if np.any(front_mask)
            else float(np.min(ranges))
        )
        left_min = float(np.min(ranges[left_mask])) if np.any(left_mask) else front_min
        right_min = (
            float(np.min(ranges[right_mask])) if np.any(right_mask) else front_min
        )
        d_min = float(np.min(ranges))

        return {
            "ranges": ranges,
            "angles": angles,
            "front_min": front_min,
            "left_min": left_min,
            "right_min": right_min,
            "d_min": d_min,
        }

    def _compute_avoidance_omega(self, left_min, right_min):
        danger_left = max(0.0, self.safety_distance - left_min) / self.safety_distance
        danger_right = (
            max(0.0, self.safety_distance - right_min) / self.safety_distance
        )
        return (danger_left - danger_right) * self.max_omega

    def _get_open_turn_sign(self, analysis, fallback_omega=0.0):
        lateral_gap = analysis["left_min"] - analysis["right_min"]
        if abs(lateral_gap) > 2.0:
            return 1.0 if lateral_gap > 0.0 else -1.0
        if abs(fallback_omega) > 0.05:
            return float(np.sign(fallback_omega))
        return 1.0

    def _apply_turn_commitment(self, analysis, omega_cmd, avoidance_omega):
        if analysis["front_min"] > self.safety_distance + 8.0:
            self.avoid_commit_steps = 0
            self.avoid_commit_sign = 0.0
            return avoidance_omega

        suggested_sign = 0.0
        if abs(avoidance_omega) > 0.05:
            suggested_sign = float(np.sign(avoidance_omega))
        elif abs(omega_cmd) > 0.1:
            suggested_sign = float(np.sign(omega_cmd))

        if self.avoid_commit_steps > 0 and self.avoid_commit_sign != 0.0:
            self.avoid_commit_steps -= 1
            commit_sign = self.avoid_commit_sign
        elif suggested_sign != 0.0:
            commit_sign = suggested_sign
            self.avoid_commit_sign = suggested_sign
            self.avoid_commit_steps = 14
        else:
            return avoidance_omega

        min_turn = 0.35 * self.max_omega
        if analysis["front_min"] <= self.stop_distance + 10.0:
            min_turn = 0.55 * self.max_omega

        return commit_sign * max(abs(avoidance_omega), min_turn)

    def _apply_recovery_commitment(self, analysis, user_cmd):
        v_cmd, omega_cmd = user_cmd

        if analysis["front_min"] > self.safety_distance + 5.0:
            self.recovery_commit_steps = 0
            self.recovery_commit_sign = 0.0
            return v_cmd, omega_cmd

        if self.recovery_commit_steps > 0 and self.recovery_commit_sign != 0.0:
            self.recovery_commit_steps -= 1
            commit_sign = self.recovery_commit_sign
        else:
            commit_sign = self._get_open_turn_sign(analysis, fallback_omega=omega_cmd)
            self.recovery_commit_sign = commit_sign
            self.recovery_commit_steps = 18

        commit_omega = commit_sign * self.max_omega
        commit_v = max(v_cmd, -0.15 * self.max_v)

        # If the front sector is no longer critically blocked, allow a small crawl
        # to break pure spin loops in narrow corridors.
        if analysis["front_min"] > self.stop_distance + 6.0 and v_cmd >= 0.0:
            commit_v = 0.15 * self.max_v

        return commit_v, commit_omega

    def apply_safety_filter(self, user_cmd, sensor_data, control_context=None):
        analysis = self._analyze_lidar(sensor_data)
        v_cmd, omega_cmd = user_cmd
        recovery_cmd = self._is_recovery_command(user_cmd, control_context)
        corridor_mode = self._is_corridor_mode(control_context)
        preferred_turn_sign = 0.0
        if control_context is not None:
            preferred_turn_sign = float(control_context.get("preferred_turn_sign", 0.0))

        if v_cmd > 0.0:
            forward_scale = self._compute_forward_scale(analysis["front_min"])
            v_exec = v_cmd * forward_scale

            # Preserve user progress when only the side wall is close.
            if analysis["d_min"] < self.stop_distance:
                v_exec = min(v_exec, 0.2 * self.max_v)
        else:
            forward_scale = 1.0
            v_exec = max(v_cmd, -0.2 * self.max_v)

        avoidance_omega = self._compute_avoidance_omega(
            analysis["left_min"], analysis["right_min"]
        )
        if corridor_mode and preferred_turn_sign != 0.0:
            if abs(avoidance_omega) > 0.05 and np.sign(avoidance_omega) != preferred_turn_sign:
                avoidance_omega *= 0.35
            elif abs(omega_cmd) > 0.05 and np.sign(omega_cmd) == preferred_turn_sign:
                avoidance_omega *= 0.5

        if recovery_cmd:
            self.avoid_commit_steps = 0
            self.avoid_commit_sign = 0.0
        else:
            avoidance_omega = self._apply_turn_commitment(
                analysis, omega_cmd, avoidance_omega
            )

        blend = 1.0 - forward_scale if v_cmd > 0.0 else 0.0
        if recovery_cmd:
            v_exec, omega_exec = self._apply_recovery_commitment(analysis, user_cmd)
        else:
            omega_exec = (1.0 - blend) * omega_cmd + blend * (
                omega_cmd + avoidance_omega
            )

        if v_cmd > 0.0 and analysis["front_min"] <= self.stop_distance:
            v_exec = 0.0
            omega_exec = omega_cmd + avoidance_omega

        v_exec = self._clip(v_exec, self.max_v)
        omega_exec = self._clip(omega_exec, self.max_omega)

        return {
            "v": v_exec,
            "omega": omega_exec,
            "d_min": analysis["d_min"],
            "front_min": analysis["front_min"],
            "left_min": analysis["left_min"],
            "right_min": analysis["right_min"],
        }

    def compute_assist_control(self, pose, goal, lidar_data, control_context=None):
        x, y, theta = pose
        goal_x, goal_y = goal

        goal_dx = goal_x - x
        goal_dy = goal_y - y
        goal_distance = math.hypot(goal_dx, goal_dy)
        goal_heading = math.atan2(goal_dy, goal_dx)
        heading_error = math.atan2(
            math.sin(goal_heading - theta), math.cos(goal_heading - theta)
        )

        analysis = self._analyze_lidar(lidar_data)
        forward_scale = self._compute_forward_scale(analysis["front_min"])
        avoidance_omega = self._compute_avoidance_omega(
            analysis["left_min"], analysis["right_min"]
        )

        if control_context is not None and control_context.get("recovery_active"):
            return {
                "v": 0.0,
                "omega": 0.0,
                "d_min": analysis["d_min"],
                "front_min": analysis["front_min"],
            }

        assist_v = self.max_v * forward_scale
        if goal_distance <= self.goal_tolerance:
            assist_v = 0.0

        heading_omega = self._clip(1.2 * heading_error, self.max_omega)
        assist_omega = self._clip(heading_omega + avoidance_omega, self.max_omega)

        return {
            "v": self._clip(assist_v, self.max_v),
            "omega": assist_omega,
            "d_min": analysis["d_min"],
            "front_min": analysis["front_min"],
        }

    def get_fixed_alpha(self):
        return self.fixed_alpha

    def get_adaptive_alpha(self, d_min):
        if d_min <= self.stop_distance:
            return 0.45
        if d_min >= self.safety_distance:
            return 0.05

        ratio = (self.safety_distance - d_min) / (
            self.safety_distance - self.stop_distance
        )
        return 0.05 + 0.40 * ratio

    def blend_commands(self, user_cmd, assist_cmd, alpha, sensor_data, control_context=None):
        user_v, user_omega = user_cmd
        assist_v, assist_omega = assist_cmd
        safety_out = self.apply_safety_filter(user_cmd, sensor_data, control_context)
        effective_alpha = alpha
        recovery_cmd = self._is_recovery_command(user_cmd, control_context)

        # When the user is rotating in place, shared control should not fight that motion.
        if abs(user_v) < 1e-6 or recovery_cmd:
            return (
                self._clip(safety_out["v"], self.max_v),
                self._clip(safety_out["omega"], self.max_omega),
            )

        # If user steering and assist steering disagree, reduce assist authority unless risk is high.
        steering_conflict = (
            abs(user_omega) > 0.1
            and abs(assist_omega) > 0.1
            and np.sign(user_omega) != np.sign(assist_omega)
        )
        if steering_conflict and safety_out["front_min"] > self.stop_distance + 10.0:
            effective_alpha *= 0.25
        if self._is_corridor_mode(control_context):
            effective_alpha *= 0.5

        # Keep translational authority almost entirely with the user.
        # Shared control mainly adjusts steering and still respects the safety filter.
        if user_v > 0.0:
            exec_v = min(user_v, safety_out["v"])
        else:
            exec_v = safety_out["v"]

        # Let the assistant mainly help by steering toward the goal and away from nearby obstacles.
        exec_omega = (
            effective_alpha * assist_omega
            + (1.0 - effective_alpha) * safety_out["omega"]
        )

        return (
            self._clip(exec_v, self.max_v),
            self._clip(exec_omega, self.max_omega),
        )
