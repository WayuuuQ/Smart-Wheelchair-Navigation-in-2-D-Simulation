import math
import numpy as np


class LidarSensor:
    def __init__(
        self,
        num_beams=31,
        fov_deg=180.0,
        max_range=180.0,
        step_size=4.0,
    ):
        self.num_beams = num_beams
        self.fov_deg = fov_deg
        self.max_range = max_range
        self.step_size = step_size

        self.relative_angles = np.deg2rad(
            np.linspace(-fov_deg / 2.0, fov_deg / 2.0, num_beams)
        )

    def sense(self, robot_pose, env):
        x, y, theta = robot_pose

        ranges = []
        hit_points = []

        for rel_angle in self.relative_angles:
            beam_angle = theta + rel_angle
            distance, hit_point = self.cast_single_ray(x, y, beam_angle, env)
            ranges.append(distance)
            hit_points.append(hit_point)

        return {
            "ranges": np.array(ranges, dtype=float),
            "hit_points": hit_points,
            "angles": self.relative_angles.copy(),
        }

    def cast_single_ray(self, x, y, angle, env):
        r = 0.0

        while r <= self.max_range:
            px = x + r * math.cos(angle)
            py = y + r * math.sin(angle)

            # hit inner boundary region
            if (
                px < env.margin
                or px > env.width - env.margin
                or py < env.margin
                or py > env.height - env.margin
            ):
                return r, (int(px), int(py))

            # hit obstacle
            for rect in env.obstacles:
                if rect.collidepoint(px, py):
                    return r, (int(px), int(py))

            r += self.step_size

        px = x + self.max_range * math.cos(angle)
        py = y + self.max_range * math.sin(angle)
        return self.max_range, (int(px), int(py))