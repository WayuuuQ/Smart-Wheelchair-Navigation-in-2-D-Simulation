# import math


# class Wheelchair:
#     def __init__(self, x=400.0, y=300.0, theta=0.0):
#         self.x = x
#         self.y = y
#         self.theta = theta

#         self.v = 0.0
#         self.omega = 0.0

#         self.max_v = 120.0          # pixels per second
#         self.max_omega = 2.5        # rad/s

#     def reset(self, x, y, theta=0.0):
#         self.x = x
#         self.y = y
#         self.theta = theta
#         self.v = 0.0
#         self.omega = 0.0

#     def step(self, v_cmd, omega_cmd, dt):
#         # Clamp commands
#         self.v = max(-self.max_v, min(self.max_v, v_cmd))
#         self.omega = max(-self.max_omega, min(self.max_omega, omega_cmd))

#         # Unicycle model
#         self.x += self.v * math.cos(self.theta) * dt
#         self.y += self.v * math.sin(self.theta) * dt
#         self.theta += self.omega * dt

#         # Keep theta within [-pi, pi]
#         self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

#     def get_pose(self):
#         return self.x, self.y, self.theta

import math


class Wheelchair:
    def __init__(self, x=400.0, y=300.0, theta=0.0, radius=20.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.radius = radius

        self.v = 0.0
        self.omega = 0.0

        self.max_v = 120.0
        self.max_omega = 2.5

    def reset(self, x, y, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0

    def step(self, v_cmd, omega_cmd, dt):
        self.v = max(-self.max_v, min(self.max_v, v_cmd))
        self.omega = max(-self.max_omega, min(self.max_omega, omega_cmd))

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def get_pose(self):
        return self.x, self.y, self.theta