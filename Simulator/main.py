import argparse
import csv
import heapq
import json
import math
from pathlib import Path

import numpy as np
from controllers import SUPPORTED_CONTROLLERS, make_controller


FPS = 60
NEAR_COLLISION_DISTANCE = 40.0
INTERVENTION_V_THRESHOLD = 1.0
INTERVENTION_OMEGA_THRESHOLD = 0.1
OSCILLATION_OMEGA_THRESHOLD = 1.0
DEFAULT_BATCH_SCENES = ("scenes/s0.json", "scenes/s1.json", "scenes/s2.json", "scenes/s3.json", "scenes/s4.json")
CONTROLLER_TO_MODE = {
    "M0": "manual",
    "M1": "fixed_alpha",
    "M2": "safety_only",
    "M3": "adaptive_alpha",
}


def _import_pygame():
    """Import pygame lazily so `--help` works without pygame installed."""
    try:
        import pygame  # type: ignore
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Missing dependency: pygame. Install it (see environment.yml) to run the simulator."
        ) from exc
    return pygame


class BatchPathUser:
    def __init__(self, env, robot_radius, forward_speed, turn_speed, grid_size=20):
        self.env = env
        self.robot_radius = robot_radius
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.grid_size = grid_size
        self.clearance = robot_radius + 4.0
        self.waypoints = self._build_waypoints()
        self.current_waypoint_idx = 0
        self.last_pose = None
        self.stuck_counter = 0
        self.last_waypoint_idx = 0
        self.waypoint_hold_steps = 0
        self.best_goal_distance = float("inf")
        self.no_progress_steps = 0
        self.recovery_steps = 0
        self.recovery_turn = 1.0
        self.recovery_active = False
        self.control_context = {
            "recovery_active": False,
            "corridor_mode": False,
            "preferred_turn_sign": 0.0,
            "path_heading_error": 0.0,
            "waypoint_distance": 0.0,
        }

    def _grid_from_world(self, x, y):
        return (
            int(round((x - self.env.margin) / self.grid_size)),
            int(round((y - self.env.margin) / self.grid_size)),
        )

    def _world_from_grid(self, gx, gy):
        return (
            self.env.margin + gx * self.grid_size,
            self.env.margin + gy * self.grid_size,
        )

    def _is_free_cell(self, gx, gy):
        x, y = self._world_from_grid(gx, gy)
        return not self.env.check_collision(x, y, self.clearance)

    def _heuristic(self, node, goal):
        return math.hypot(node[0] - goal[0], node[1] - goal[1])

    def _distance_to_nearest_obstacle(self, x, y):
        boundary_clearance = min(
            x - self.env.margin,
            self.env.width - self.env.margin - x,
            y - self.env.margin,
            self.env.height - self.env.margin - y,
        )
        nearest = boundary_clearance

        for rect in self.env.obstacles:
            dx = max(rect.left - x, 0.0, x - rect.right)
            dy = max(rect.top - y, 0.0, y - rect.bottom)
            distance = math.hypot(dx, dy)
            if rect.left <= x <= rect.right and rect.top <= y <= rect.bottom:
                distance = 0.0
            nearest = min(nearest, distance)

        return nearest

    def _cell_penalty(self, gx, gy):
        x, y = self._world_from_grid(gx, gy)
        clearance = self._distance_to_nearest_obstacle(x, y) - self.robot_radius
        if clearance >= 50.0:
            return 0.0
        return ((50.0 - max(clearance, 0.0)) / 10.0) ** 2

    def _neighbors(self, node, max_gx, max_gy):
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ):
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx <= max_gx and 0 <= ny <= max_gy:
                yield nx, ny, math.hypot(dx, dy)

    def _segment_is_free(self, start_xy, end_xy):
        distance = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
        samples = max(2, int(distance / 8.0))
        for idx in range(samples + 1):
            ratio = idx / samples
            x = start_xy[0] + ratio * (end_xy[0] - start_xy[0])
            y = start_xy[1] + ratio * (end_xy[1] - start_xy[1])
            if self.env.check_collision(x, y, self.clearance):
                return False
        return True

    def _find_nearest_free_cell(self, cell, max_gx, max_gy):
        if self._is_free_cell(*cell):
            return cell

        max_radius = max(max_gx, max_gy)
        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx = max(0, min(max_gx, cell[0] + dx))
                    ny = max(0, min(max_gy, cell[1] + dy))
                    if self._is_free_cell(nx, ny):
                        return nx, ny
        return cell

    def _plan_grid_path(self, start_xy, goal_xy):
        max_gx = int((self.env.width - 2 * self.env.margin) / self.grid_size)
        max_gy = int((self.env.height - 2 * self.env.margin) / self.grid_size)

        start = self._grid_from_world(*start_xy)
        goal = self._grid_from_world(*goal_xy)
        start = (
            max(0, min(max_gx, start[0])),
            max(0, min(max_gy, start[1])),
        )
        goal = (
            max(0, min(max_gx, goal[0])),
            max(0, min(max_gy, goal[1])),
        )
        start = self._find_nearest_free_cell(start, max_gx, max_gy)
        goal = self._find_nearest_free_cell(goal, max_gx, max_gy)

        open_heap = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for nx, ny, move_cost in self._neighbors(current, max_gx, max_gy):
                neighbor = (nx, ny)
                if not self._is_free_cell(nx, ny):
                    continue
                if move_cost > 1.0:
                    if not self._is_free_cell(current[0], ny):
                        continue
                    if not self._is_free_cell(nx, current[1]):
                        continue

                tentative_g = g_score[current] + move_cost + 0.25 * self._cell_penalty(nx, ny)
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    priority = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (priority, neighbor))

        return []

    def _simplify_path(self, grid_path):
        if not grid_path:
            return []

        simplified_grid = [grid_path[0]]
        last_direction = None
        for idx in range(1, len(grid_path)):
            dx = grid_path[idx][0] - grid_path[idx - 1][0]
            dy = grid_path[idx][1] - grid_path[idx - 1][1]
            direction = (dx, dy)
            keep_for_clearance = False
            if idx > 1:
                keep_for_clearance = self._cell_penalty(*grid_path[idx - 1]) >= 1.5
            if (direction != last_direction and idx > 1) or keep_for_clearance:
                simplified_grid.append(grid_path[idx - 1])
            last_direction = direction
        simplified_grid.append(grid_path[-1])

        world_points = []
        for gx, gy in simplified_grid:
            wx, wy = self._world_from_grid(gx, gy)
            if world_points and math.hypot(wx - world_points[-1][0], wy - world_points[-1][1]) < 1e-6:
                continue
            world_points.append((wx, wy))
        return world_points

    def _make_escape_subgoal(self, pose, target_heading, turn_sign):
        lateral_distance = 55.0
        forward_distance = 35.0
        subgoal_x = (
            pose[0]
            + forward_distance * math.cos(target_heading)
            + lateral_distance * math.cos(target_heading + turn_sign * math.pi / 2.0)
        )
        subgoal_y = (
            pose[1]
            + forward_distance * math.sin(target_heading)
            + lateral_distance * math.sin(target_heading + turn_sign * math.pi / 2.0)
        )
        subgoal_x = max(self.env.margin + self.clearance, min(self.env.width - self.env.margin - self.clearance, subgoal_x))
        subgoal_y = max(self.env.margin + self.clearance, min(self.env.height - self.env.margin - self.clearance, subgoal_y))
        if self.env.check_collision(subgoal_x, subgoal_y, self.clearance):
            return None
        return (subgoal_x, subgoal_y)

    def _replan_from_pose(self, pose, escape_subgoal=None):
        start_xy = (pose[0], pose[1])
        if escape_subgoal is None:
            grid_path = self._plan_grid_path(start_xy, self.env.goal)
        else:
            first_leg = self._plan_grid_path(start_xy, escape_subgoal)
            second_leg = self._plan_grid_path(escape_subgoal, self.env.goal)
            if first_leg and second_leg:
                grid_path = first_leg[:-1] + second_leg
            else:
                grid_path = self._plan_grid_path(start_xy, self.env.goal)
        if not grid_path:
            return
        self.waypoints = self._simplify_path(grid_path)
        if not self.waypoints or math.hypot(self.waypoints[-1][0] - self.env.goal[0], self.waypoints[-1][1] - self.env.goal[1]) > self.env.goal_radius:
            self.waypoints.append(self.env.goal)
        self.current_waypoint_idx = 0
        self.last_waypoint_idx = 0
        self.waypoint_hold_steps = 0

    def _build_waypoints(self):
        start_x, start_y, _ = self.env.get_start_pose()
        grid_path = self._plan_grid_path((start_x, start_y), self.env.goal)
        if not grid_path:
            return [self.env.goal]

        waypoints = self._simplify_path(grid_path)
        if waypoints and math.hypot(waypoints[-1][0] - self.env.goal[0], waypoints[-1][1] - self.env.goal[1]) > self.env.goal_radius:
            waypoints.append(self.env.goal)
        return waypoints

    def get_command(self, pose, lidar_data=None):
        x, y, theta = pose
        self.recovery_active = self.recovery_steps > 0
        self.control_context = {
            "recovery_active": self.recovery_active,
            "corridor_mode": False,
            "preferred_turn_sign": 0.0,
            "path_heading_error": 0.0,
            "waypoint_distance": 0.0,
        }

        if self.last_pose is not None:
            moved = math.hypot(x - self.last_pose[0], y - self.last_pose[1])
            if moved < 1.0:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
        self.last_pose = (x, y)
        goal_distance = math.hypot(self.env.goal[0] - x, self.env.goal[1] - y)
        if goal_distance < self.best_goal_distance - 12.0:
            self.best_goal_distance = goal_distance
            self.no_progress_steps = 0
        else:
            self.no_progress_steps += 1

        if self.stuck_counter >= 90:
            self._replan_from_pose(pose)
            self.recovery_steps = 25
            self.recovery_turn *= -1.0
            self.stuck_counter = 0
            self.recovery_active = True
            self.no_progress_steps = 0

        if self.recovery_steps > 0:
            self.recovery_steps -= 1
            self.recovery_active = True
            self.control_context["recovery_active"] = True
            self.control_context["preferred_turn_sign"] = self.recovery_turn
            if self.recovery_steps >= 12:
                return 0.0, self.recovery_turn * self.turn_speed
            return -0.2 * self.forward_speed, self.recovery_turn * self.turn_speed

        while self.current_waypoint_idx < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint_idx]
            reached_waypoint = math.hypot(target_x - x, target_y - y) < max(
                self.grid_size * 0.65, 12.0
            )
            if reached_waypoint:
                self.current_waypoint_idx += 1
            else:
                break

        if self.current_waypoint_idx >= len(self.waypoints):
            return 0.0, 0.0

        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        if self.current_waypoint_idx == self.last_waypoint_idx:
            self.waypoint_hold_steps += 1
        else:
            self.last_waypoint_idx = self.current_waypoint_idx
            self.waypoint_hold_steps = 0
        target_heading = math.atan2(target_y - y, target_x - x)
        heading_error = math.atan2(
            math.sin(target_heading - theta), math.cos(target_heading - theta)
        )

        distance_to_target = math.hypot(target_x - x, target_y - y)
        omega_cmd = max(-self.turn_speed, min(self.turn_speed, 2.0 * heading_error))
        preferred_turn_sign = 0.0
        if abs(heading_error) > 0.08:
            preferred_turn_sign = 1.0 if heading_error > 0.0 else -1.0
        elif abs(omega_cmd) > 0.05:
            preferred_turn_sign = 1.0 if omega_cmd > 0.0 else -1.0

        front_min = None
        left_min = None
        right_min = None
        d_min = None
        if lidar_data is not None and len(lidar_data["ranges"]) > 0:
            ranges = np.asarray(lidar_data["ranges"], dtype=float)
            angles = np.asarray(lidar_data["angles"], dtype=float)
            front_mask = np.abs(angles) <= math.radians(22.0)
            left_mask = (angles > 0.0) & (angles <= math.radians(90.0))
            right_mask = (angles < 0.0) & (angles >= -math.radians(90.0))
            front_min = (
                float(np.min(ranges[front_mask]))
                if np.any(front_mask)
                else float(np.min(ranges))
            )
            left_min = (
                float(np.min(ranges[left_mask])) if np.any(left_mask) else front_min
            )
            right_min = (
                float(np.min(ranges[right_mask])) if np.any(right_mask) else front_min
            )
            d_min = float(np.min(ranges))

        speed_scale = 1.0
        if abs(heading_error) > 0.7:
            speed_scale = 0.0
        elif abs(heading_error) > 0.35:
            speed_scale = 0.35
        elif distance_to_target < 40.0:
            speed_scale = 0.55

        if front_min is not None and front_min < 70.0:
            speed_scale = min(speed_scale, max(0.0, (front_min - 26.0) / 44.0))

        v_cmd = self.forward_speed * speed_scale
        corridor_mode = False

        if front_min is not None and left_min is not None and right_min is not None:
            open_turn = self.turn_speed if left_min > right_min else -self.turn_speed
            if preferred_turn_sign == 0.0:
                preferred_turn_sign = 1.0 if open_turn > 0.0 else -1.0

            corridor_mode = (
                front_min < 95.0
                and d_min is not None
                and d_min > 22.0
                and distance_to_target > 18.0
                and abs(heading_error) < 0.35
            )
            if front_min < 52.0:
                if corridor_mode and preferred_turn_sign != 0.0:
                    omega_cmd = preferred_turn_sign * self.turn_speed
                else:
                    omega_cmd = open_turn
                v_cmd = min(v_cmd, 0.3 * self.forward_speed)
            if front_min < 36.0:
                if corridor_mode and preferred_turn_sign != 0.0:
                    omega_cmd = preferred_turn_sign * self.turn_speed
                else:
                    omega_cmd = open_turn
                v_cmd = 0.0
            if d_min is not None and d_min < 24.0:
                self.recovery_steps = 16
                if abs(heading_error) > 0.12:
                    self.recovery_turn = 1.0 if heading_error > 0.0 else -1.0
                else:
                    self.recovery_turn = 1.0 if left_min > right_min else -1.0
                self.recovery_active = True
                self.control_context = {
                    "recovery_active": True,
                    "corridor_mode": False,
                    "preferred_turn_sign": self.recovery_turn,
                    "path_heading_error": float(heading_error),
                    "waypoint_distance": float(distance_to_target),
                }
                return 0.0, self.recovery_turn * self.turn_speed

        if self.no_progress_steps >= 220:
            self.recovery_turn = -self.recovery_turn
            escape_subgoal = self._make_escape_subgoal(
                pose, target_heading, self.recovery_turn
            )
            self._replan_from_pose(pose, escape_subgoal=escape_subgoal)
            self.recovery_steps = 14
            self.recovery_active = True
            self.no_progress_steps = 0
            self.waypoint_hold_steps = 0
            self.control_context = {
                "recovery_active": True,
                "corridor_mode": False,
                "preferred_turn_sign": self.recovery_turn,
                "path_heading_error": float(heading_error),
                "waypoint_distance": float(distance_to_target),
            }
            return 0.0, self.recovery_turn * self.turn_speed

        self.control_context = {
            "recovery_active": self.recovery_active,
            "corridor_mode": corridor_mode,
            "preferred_turn_sign": preferred_turn_sign,
            "path_heading_error": float(heading_error),
            "waypoint_distance": float(distance_to_target),
        }

        return v_cmd, omega_cmd

    def get_control_context(self):
        return dict(self.control_context)


def reset_episode(wheelchair, env):
    x, y, theta = env.get_start_pose()
    wheelchair.reset(x, y, theta)
    env.reset()

    path_points = [(int(x), int(y))]
    episode_status = "running"
    elapsed_time = 0.0
    step_count = 0

    return path_points, episode_status, elapsed_time, step_count


def parse_args():
    parser = argparse.ArgumentParser(description="Smart Wheelchair Simulator")
    parser.add_argument(
        "--scene",
        type=str,
        required=False,
        help="Path to the scene JSON file, e.g. scenes/s1.json",
        default="scenes/s1.json",
    )
    parser.add_argument(
        "--controller",
        type=lambda s: str(s).upper(),
        choices=SUPPORTED_CONTROLLERS,
        default="M0",
        help="Controller choice: M0 (manual), M1 (fixed blending), M2 (safety filter), M3 (adaptive blending)",
    )
    parser.add_argument(
        "--a0",
        type=float,
        default=0.6,
        help="M1 fixed blending weight: u = a0*u_h + (1-a0)*u_a (default: 0.6)",
    )
    parser.add_argument(
        "--safety-distance",
        type=float,
        default=75.0,
        help="Safety filter / planner distance threshold (default: 75.0)",
    )
    parser.add_argument(
        "--stop-distance",
        type=float,
        default=35.0,
        help="Safety filter hard stop distance threshold (default: 35.0)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable periodic status prints during simulation (default: off).",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=30,
        help="When --verbose is set, print one status line every N steps (default: 30).",
    )
    # Backward-compatible alias (deprecated).
    parser.add_argument(
        "--mode",
        type=str,
        choices=("manual", "safety_only", "fixed_alpha", "adaptive_alpha"),
        default=None,
        help="(Deprecated) Use --controller. Kept for backward compatibility.",
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Run batch experiments without manual interaction",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Number of episodes per scene and mode in batch mode",
    )
    parser.add_argument(
        "--batch-scenes",
        nargs="+",
        default=list(DEFAULT_BATCH_SCENES),
        help="Scene list for batch mode",
    )
    parser.add_argument(
        "--batch-controllers",
        nargs="+",
        type=lambda s: str(s).upper(),
        choices=SUPPORTED_CONTROLLERS,
        default=list(SUPPORTED_CONTROLLERS),
        help="Controller list for batch mode",
    )
    parser.add_argument(
        "--results-dir",
        type=str,
        default="results",
        help="Output directory for batch experiment results",
    )
    args = parser.parse_args()

    # If user supplied legacy --mode but not --controller, map it.
    if args.mode is not None and args.controller == "M0":
        legacy_map = {
            "manual": "M0",
            "fixed_alpha": "M1",
            "safety_only": "M2",
            "adaptive_alpha": "M3",
        }
        args.controller = legacy_map.get(args.mode, args.controller)

    return args


def controller_id_to_mode(controller_id):
    return CONTROLLER_TO_MODE.get(str(controller_id).upper(), str(controller_id))


def compute_path_length(path_points):
    if len(path_points) < 2:
        return 0.0

    total = 0.0
    for idx in range(1, len(path_points)):
        x0, y0 = path_points[idx - 1]
        x1, y1 = path_points[idx]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def build_episode_metrics():
    return {
        "min_obstacle_distance": float("inf"),
        "obstacle_distance_sum": 0.0,
        "obstacle_samples": 0,
        "near_collision_count": 0,
        "reverse_count": 0,
        "reverse_time": 0.0,
        "stuck_count": 0,
        "recovery_count": 0,
        "oscillation_count": 0,
        "abs_v_diff_sum": 0.0,
        "abs_omega_diff_sum": 0.0,
        "intervention_steps": 0,
        "control_steps": 0,
        "alpha_sum": 0.0,
        "alpha_count": 0,
        "max_alpha": 0.0,
    }


def finalize_episode_metrics(metrics):
    obstacle_samples = max(1, metrics["obstacle_samples"])
    control_steps = max(1, metrics["control_steps"])
    alpha_count = max(1, metrics["alpha_count"])

    min_obstacle_distance = metrics["min_obstacle_distance"]
    if min_obstacle_distance == float("inf"):
        min_obstacle_distance = None

    return {
        "min_obstacle_distance": (
            None if min_obstacle_distance is None else round(min_obstacle_distance, 4)
        ),
        "avg_obstacle_distance": round(
            metrics["obstacle_distance_sum"] / obstacle_samples, 4
        ),
        "near_collision_count": metrics["near_collision_count"],
        "reverse_count": metrics["reverse_count"],
        "reverse_time": round(metrics["reverse_time"], 4),
        "stuck_count": metrics["stuck_count"],
        "recovery_count": metrics["recovery_count"],
        "oscillation_count": metrics["oscillation_count"],
        "avg_abs_v_diff": round(metrics["abs_v_diff_sum"] / control_steps, 4),
        "avg_abs_omega_diff": round(
            metrics["abs_omega_diff_sum"] / control_steps, 4
        ),
        "intervention_rate": round(
            metrics["intervention_steps"] / control_steps, 4
        ),
        "avg_alpha": round(metrics["alpha_sum"] / alpha_count, 4)
        if metrics["alpha_count"] > 0
        else 0.0,
        "max_alpha": round(metrics["max_alpha"], 4),
    }


def build_group_stats(results):
    grouped_rows = []
    group_keys = sorted(
        {
            (row["scene"], row["mode"], row.get("controller_id", row["mode"]))
            for row in results
        }
    )

    for scene, mode, controller_id in group_keys:
        group_rows = [
            row
            for row in results
            if row["scene"] == scene
            and row["mode"] == mode
            and row.get("controller_id", row["mode"]) == controller_id
        ]
        num_runs = len(group_rows)
        if num_runs == 0:
            continue

        grouped_rows.append(
            {
                "scene": scene,
                "mode": mode,
                "controller_id": controller_id,
                "episodes": num_runs,
                "success_rate": round(
                    sum(1 for row in group_rows if row["success"]) / num_runs, 4
                ),
                "collision_rate": round(
                    sum(1 for row in group_rows if row["collision"]) / num_runs, 4
                ),
                "timeout_rate": round(
                    sum(1 for row in group_rows if row["timeout"]) / num_runs, 4
                ),
                "avg_completion_time": round(
                    sum(row["completion_time"] for row in group_rows) / num_runs,
                    4,
                ),
                "avg_path_length": round(
                    sum(row["path_length"] for row in group_rows) / num_runs, 4
                ),
                "avg_min_obstacle_distance": round(
                    sum(row["min_obstacle_distance"] for row in group_rows) / num_runs,
                    4,
                ),
                "avg_obstacle_distance": round(
                    sum(row["avg_obstacle_distance"] for row in group_rows) / num_runs, 4
                ),
                "avg_near_collision_count": round(
                    sum(row["near_collision_count"] for row in group_rows) / num_runs, 4
                ),
                "avg_reverse_count": round(
                    sum(row["reverse_count"] for row in group_rows) / num_runs, 4
                ),
                "avg_reverse_time": round(
                    sum(row["reverse_time"] for row in group_rows) / num_runs, 4
                ),
                "avg_stuck_count": round(
                    sum(row["stuck_count"] for row in group_rows) / num_runs, 4
                ),
                "avg_recovery_count": round(
                    sum(row["recovery_count"] for row in group_rows) / num_runs, 4
                ),
                "avg_oscillation_count": round(
                    sum(row["oscillation_count"] for row in group_rows) / num_runs, 4
                ),
                "avg_abs_v_diff": round(
                    sum(row["avg_abs_v_diff"] for row in group_rows) / num_runs, 4
                ),
                "avg_abs_omega_diff": round(
                    sum(row["avg_abs_omega_diff"] for row in group_rows) / num_runs, 4
                ),
                "avg_intervention_rate": round(
                    sum(row["intervention_rate"] for row in group_rows) / num_runs, 4
                ),
                "avg_alpha": round(
                    sum(row["avg_alpha"] for row in group_rows) / num_runs, 4
                ),
                "max_alpha": round(
                    max(row["max_alpha"] for row in group_rows), 4
                ),
            }
        )

    return grouped_rows


def build_simulation(scene_data):
    # Imports live here so `python Simulator/main.py --help` works even if pygame isn't installed.
    from environment import Environment
    from input_handler import KeyboardTeleop
    from sensor import LidarSensor
    from wheelchair import Wheelchair

    env = Environment(scene_data)
    sx, sy, stheta = env.get_start_pose()
    robot_radius = scene_data.get("robot", {}).get("radius", 20.0)

    wheelchair = Wheelchair(x=sx, y=sy, theta=stheta, radius=robot_radius)
    teleop = KeyboardTeleop()
    lidar = LidarSensor(
        num_beams=31,
        fov_deg=180.0,
        max_range=220.0,
        step_size=4.0,
    )
    max_episode_time = scene_data.get("episode", {}).get("max_time", 60.0)

    return env, wheelchair, teleop, lidar, max_episode_time


def build_obs(pose, goal, lidar_data, control_context=None):
    return {
        "pose": pose,
        "goal": goal,
        "lidar": lidar_data,
        "control_context": control_context,
    }


def run_episode(scene_path, controller_id, controller_kwargs=None, verbose=False, log_every=30, render=True, interactive=True):
    from environment import load_scene

    pygame = _import_pygame()

    scene_data = load_scene(scene_path)
    env, wheelchair, teleop, lidar, max_episode_time = build_simulation(
        scene_data
    )
    renderer = None
    screen = None
    clock = pygame.time.Clock()
    current_controller_id = controller_id
    controller_kwargs = {} if controller_kwargs is None else dict(controller_kwargs)
    controller = make_controller(
        current_controller_id,
        max_v=wheelchair.max_v,
        max_omega=wheelchair.max_omega,
        **controller_kwargs,
    )
    controller_key_map = {
        pygame.K_1: "M0",
        pygame.K_2: "M1",
        pygame.K_3: "M2",
        pygame.K_4: "M3",
    }

    if render:
        from renderer import Renderer

        screen = pygame.display.set_mode((scene_data["map"]["width"], scene_data["map"]["height"]))
        pygame.display.set_caption(scene_data.get("name", "Smart Wheelchair Simulator"))
        renderer = Renderer(screen)

    path_points, episode_status, elapsed_time, step_count = reset_episode(
        wheelchair, env
    )
    episode_metrics = build_episode_metrics()
    final_alpha = None
    final_assist_cmd = None
    user_cmd = (0.0, 0.0)
    exec_v, exec_omega = 0.0, 0.0
    batch_user = None
    control_context = None
    prev_reverse_active = False
    prev_recovery_active = False
    prev_omega_sign = 0
    low_progress_steps = 0
    stuck_active = False
    if not interactive:
        batch_user = BatchPathUser(
            env=env,
            robot_radius=wheelchair.radius,
            forward_speed=teleop.forward_speed,
            turn_speed=teleop.turn_speed,
        )

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 if render else 1.0 / FPS

        if render:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    episode_status = "quit"

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        path_points, episode_status, elapsed_time, step_count = reset_episode(
                            wheelchair, env
                        )

                    if renderer is not None and event.key == pygame.K_l:
                        renderer.show_lidar = not renderer.show_lidar

                    if interactive and event.key in controller_key_map:
                        current_controller_id = controller_key_map[event.key]
                        controller = make_controller(
                            current_controller_id,
                            max_v=wheelchair.max_v,
                            max_omega=wheelchair.max_omega,
                            **controller_kwargs,
                        )
                        print("Switched controller to: {0}".format(current_controller_id))

        if episode_status != "running":
            break

        pose = wheelchair.get_pose()
        lidar_data = lidar.sense(pose, env)

        if interactive:
            user_cmd = teleop.get_command()
            control_context = None
        else:
            user_cmd = batch_user.get_command(pose, lidar_data)
            control_context = batch_user.get_control_context()

        obs = build_obs(pose, env.goal, lidar_data, control_context)
        action = controller.get_action(obs, user_cmd)
        exec_v = float(action["v"])
        exec_omega = float(action["omega"])
        final_alpha = action.get("alpha", None)
        final_assist_cmd = action.get("u_a", None)

        # Low-frequency console logging for acceptance checks (no spam unless --verbose).
        if verbose and log_every > 0 and step_count % log_every == 0:
            d_min = float(np.min(lidar_data["ranges"]))
            u_a = action.get("u_a", None)
            alpha = action.get("alpha", None)
            print(
                f"[{current_controller_id}] "
                f"d_min={d_min:.2f} "
                f"user_cmd={user_cmd} "
                f"exec=({exec_v:.2f},{exec_omega:.2f}) "
                f"u_a={u_a} "
                f"alpha={alpha}"
            )
        old_x, old_y = pose[0], pose[1]
        d_min = float(np.min(lidar_data["ranges"]))

        episode_metrics["min_obstacle_distance"] = min(
            episode_metrics["min_obstacle_distance"], d_min
        )
        episode_metrics["obstacle_distance_sum"] += d_min
        episode_metrics["obstacle_samples"] += 1
        if d_min <= NEAR_COLLISION_DISTANCE:
            episode_metrics["near_collision_count"] += 1

        reverse_active = exec_v < -1e-6
        if reverse_active and not prev_reverse_active:
            episode_metrics["reverse_count"] += 1
        if reverse_active:
            episode_metrics["reverse_time"] += dt
        prev_reverse_active = reverse_active

        recovery_active = bool(
            control_context is not None and control_context.get("recovery_active", False)
        )
        if recovery_active and not prev_recovery_active:
            episode_metrics["recovery_count"] += 1
        prev_recovery_active = recovery_active

        current_omega_sign = 0
        if abs(exec_omega) >= OSCILLATION_OMEGA_THRESHOLD:
            current_omega_sign = 1 if exec_omega > 0.0 else -1
        if (
            prev_omega_sign != 0
            and current_omega_sign != 0
            and current_omega_sign != prev_omega_sign
        ):
            episode_metrics["oscillation_count"] += 1
        prev_omega_sign = current_omega_sign

        abs_v_diff = abs(exec_v - user_cmd[0])
        abs_omega_diff = abs(exec_omega - user_cmd[1])
        episode_metrics["abs_v_diff_sum"] += abs_v_diff
        episode_metrics["abs_omega_diff_sum"] += abs_omega_diff
        episode_metrics["control_steps"] += 1
        if (
            abs_v_diff > INTERVENTION_V_THRESHOLD
            or abs_omega_diff > INTERVENTION_OMEGA_THRESHOLD
        ):
            episode_metrics["intervention_steps"] += 1

        alpha_value = 0.0 if final_alpha is None else float(final_alpha)
        episode_metrics["alpha_sum"] += alpha_value
        episode_metrics["alpha_count"] += 1
        episode_metrics["max_alpha"] = max(episode_metrics["max_alpha"], alpha_value)

        # (Old periodic print removed: replaced by the --verbose/--log-every logger above.)

        wheelchair.step(exec_v, exec_omega, dt)
        env.update(dt)

        x, y, _ = wheelchair.get_pose()
        path_points.append((int(x), int(y)))
        movement = math.hypot(x - old_x, y - old_y)
        if abs(exec_v) > 15.0 and movement < 0.5:
            low_progress_steps += 1
            if low_progress_steps >= 45 and not stuck_active:
                episode_metrics["stuck_count"] += 1
                stuck_active = True
        else:
            low_progress_steps = 0
            stuck_active = False

        elapsed_time += dt
        step_count += 1

        if env.check_collision(x, y, wheelchair.radius):
            episode_status = "collision"
        elif env.goal_reached(x, y):
            episode_status = "success"
        elif elapsed_time >= max_episode_time:
            episode_status = "timeout"

        if render and renderer is not None:
            renderer.render(
                wheelchair=wheelchair,
                env=env,
                lidar_data=lidar_data,
                dt=dt,
                elapsed_time=elapsed_time,
                step_count=step_count,
                episode_status=episode_status,
                path_points=path_points,
            )

    final_pose = wheelchair.get_pose()
    goal_x, goal_y = env.goal
    goal_distance = math.hypot(final_pose[0] - goal_x, final_pose[1] - goal_y)
    finalized_metrics = finalize_episode_metrics(episode_metrics)
    mode_name = controller_id_to_mode(current_controller_id)

    return {
        "scene": scene_path,
        "mode": mode_name,
        "controller_id": current_controller_id,
        "episode_index": 0,
        "success": episode_status == "success",
        "collision": episode_status == "collision",
        "timeout": episode_status == "timeout",
        "completion_time": round(elapsed_time, 4),
        "status": episode_status,
        "elapsed_time": round(elapsed_time, 4),
        "step_count": step_count,
        "path_length": round(compute_path_length(path_points), 4),
        "goal_distance": round(goal_distance, 4),
        **finalized_metrics,
        "final_pose": [round(value, 4) for value in final_pose],
        "final_user_cmd": [round(user_cmd[0], 4), round(user_cmd[1], 4)],
        "final_assist_cmd": (
            [round(final_assist_cmd[0], 4), round(final_assist_cmd[1], 4)]
            if final_assist_cmd is not None
            else None
        ),
        "final_alpha": 0.0 if final_alpha is None else round(final_alpha, 4),
    }


def save_batch_results(results, output_dir):
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    episode_json_path = output_path / "episode_results.json"
    episode_csv_path = output_path / "episode_results.csv"
    summary_json_path = output_path / "batch_summary.json"
    summary_csv_path = output_path / "batch_summary.csv"
    summary_rows = build_group_stats(results)

    with open(episode_json_path, "w", encoding="utf-8") as json_file:
        json.dump(results, json_file, indent=2)

    fieldnames = [
        "scene",
        "mode",
        "controller_id",
        "episode_index",
        "success",
        "collision",
        "timeout",
        "completion_time",
        "status",
        "elapsed_time",
        "step_count",
        "path_length",
        "goal_distance",
        "min_obstacle_distance",
        "avg_obstacle_distance",
        "near_collision_count",
        "reverse_count",
        "reverse_time",
        "stuck_count",
        "recovery_count",
        "oscillation_count",
        "avg_abs_v_diff",
        "avg_abs_omega_diff",
        "intervention_rate",
        "avg_alpha",
        "max_alpha",
        "final_alpha",
        "final_pose",
        "final_user_cmd",
        "final_assist_cmd",
    ]
    with open(episode_csv_path, "w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in results:
            writer.writerow(row)

    with open(summary_json_path, "w", encoding="utf-8") as json_file:
        json.dump(summary_rows, json_file, indent=2)

    summary_fieldnames = [
        "scene",
        "mode",
        "controller_id",
        "episodes",
        "success_rate",
        "collision_rate",
        "timeout_rate",
        "avg_completion_time",
        "avg_path_length",
        "avg_min_obstacle_distance",
        "avg_obstacle_distance",
        "avg_near_collision_count",
        "avg_reverse_count",
        "avg_reverse_time",
        "avg_stuck_count",
        "avg_recovery_count",
        "avg_oscillation_count",
        "avg_abs_v_diff",
        "avg_abs_omega_diff",
        "avg_intervention_rate",
        "avg_alpha",
        "max_alpha",
    ]
    with open(summary_csv_path, "w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=summary_fieldnames)
        writer.writeheader()
        for row in summary_rows:
            writer.writerow(row)


def run_batch_experiments(args):
    results = []
    controller_kwargs = {
        "a0": float(args.a0),
        "safety_distance": float(args.safety_distance),
        "stop_distance": float(args.stop_distance),
    }
    log_every = int(args.log_every)
    verbose = bool(args.verbose)

    for scene_path in args.batch_scenes:
        for controller_id in args.batch_controllers:
            for episode_index in range(args.episodes):
                print(
                    "Batch run: scene={0}, controller={1}, episode={2}".format(
                        scene_path, controller_id, episode_index
                    )
                )
                result = run_episode(
                    scene_path=scene_path,
                    controller_id=controller_id,
                    controller_kwargs=controller_kwargs,
                    verbose=verbose,
                    log_every=log_every,
                    render=False,
                    interactive=False,
                )
                result["episode_index"] = episode_index
                results.append(result)

    save_batch_results(results, args.results_dir)
    print("Batch results saved to: {0}".format(args.results_dir))


def main():
    args = parse_args()

    pygame = _import_pygame()

    controller_kwargs = {
        "a0": float(args.a0),
        "safety_distance": float(args.safety_distance),
        "stop_distance": float(args.stop_distance),
    }
    log_every = int(args.log_every)
    verbose = bool(args.verbose)

    if args.batch:
        pygame.init()
        try:
            run_batch_experiments(args)
        finally:
            pygame.quit()
        return

    pygame.init()
    try:
        print("Running controller: {0}".format(args.controller))
        run_episode(
            scene_path=args.scene,
            controller_id=args.controller,
            controller_kwargs=controller_kwargs,
            verbose=verbose,
            log_every=log_every,
            render=True,
            interactive=True,
        )
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
