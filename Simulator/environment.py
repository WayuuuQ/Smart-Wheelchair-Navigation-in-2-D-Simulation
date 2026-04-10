import math
import pygame
import json
from pathlib import Path


def load_scene(scene_path):
    scene_path = Path(scene_path)

    if not scene_path.exists():
        raise FileNotFoundError(f"Scene file not found: {scene_path}")

    with open(scene_path, "r", encoding="utf-8") as f:
        scene_data = json.load(f)

    return scene_data


class Environment:
    def __init__(self, scene_data):
        self.scene_data = scene_data

        self.width = scene_data["map"]["width"]
        self.height = scene_data["map"]["height"]
        self.margin = scene_data["map"].get("margin", 40)

        start = scene_data["start_pose"]
        self.start_pose = (
            float(start["x"]),
            float(start["y"]),
            float(start["theta"]),
        )

        goal = scene_data["goal"]
        self.goal = (float(goal["x"]), float(goal["y"]))
        self.goal_radius = float(goal.get("radius", 30.0))

        self.obstacles = []
        self.dynamic_obstacles = []

        self._load_static_obstacles()
        self._load_dynamic_obstacles()

    def _load_static_obstacles(self):
        self.obstacles = []
        for obs in self.scene_data.get("static_obstacles", []):
            rect = pygame.Rect(obs["x"], obs["y"], obs["w"], obs["h"])
            self.obstacles.append(rect)

    def _load_dynamic_obstacles(self):
        self.dynamic_obstacles = self.scene_data.get("dynamic_obstacles", [])

    def reset(self):
        self._load_static_obstacles()
        self._load_dynamic_obstacles()

    def update(self, dt):
        # Placeholder for dynamic obstacle updates later
        pass

    def check_boundary_collision(self, x, y, radius):
        if x - radius < self.margin:
            return True
        if x + radius > self.width - self.margin:
            return True
        if y - radius < self.margin:
            return True
        if y + radius > self.height - self.margin:
            return True
        return False

    def circle_rect_collision(self, cx, cy, radius, rect):
        closest_x = max(rect.left, min(cx, rect.right))
        closest_y = max(rect.top, min(cy, rect.bottom))

        dx = cx - closest_x
        dy = cy - closest_y

        return (dx * dx + dy * dy) <= (radius * radius)

    def check_obstacle_collision(self, x, y, radius):
        for rect in self.obstacles:
            if self.circle_rect_collision(x, y, radius, rect):
                return True
        return False

    def check_collision(self, x, y, radius):
        return (
            self.check_boundary_collision(x, y, radius)
            or self.check_obstacle_collision(x, y, radius)
        )

    def goal_reached(self, x, y):
        gx, gy = self.goal
        return math.hypot(x - gx, y - gy) <= self.goal_radius

    def get_start_pose(self):
        return self.start_pose