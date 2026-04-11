import math
import pygame


class Renderer:
    def __init__(self, screen):
        self.screen = screen

        self.bg_color = (245, 245, 245)
        self.border_color = (60, 60, 60)
        self.robot_color = (40, 90, 180)
        self.heading_color = (220, 50, 50)
        self.obstacle_color = (90, 90, 90)
        self.goal_color = (60, 180, 75)
        self.goal_outline = (20, 120, 40)
        self.path_color = (120, 170, 255)
        self.text_color = (20, 20, 20)
        self.success_color = (20, 140, 40)
        self.fail_color = (180, 40, 40)

        self.show_lidar = False
        self.lidar_beam_color = (255, 165, 0)
        self.lidar_hit_color = (255, 80, 0)

        pygame.font.init()
        self.font = pygame.font.Font(None, 20)
        self.big_font = pygame.font.Font(None, 28)

    def draw_robot(self, x, y, theta, radius):
        center = (int(x), int(y))
        pygame.draw.circle(self.screen, self.robot_color, center, int(radius))

        hx = x + radius * 1.5 * math.cos(theta)
        hy = y + radius * 1.5 * math.sin(theta)
        pygame.draw.line(
            self.screen,
            self.heading_color,
            center,
            (int(hx), int(hy)),
            4
        )

        pygame.draw.circle(self.screen, (255, 255, 255), center, 4)

    def draw_environment(self, env):
        border_rect = pygame.Rect(
            env.margin,
            env.margin,
            env.width - 2 * env.margin,
            env.height - 2 * env.margin,
        )
        pygame.draw.rect(self.screen, self.border_color, border_rect, 3)

        for rect in env.obstacles:
            pygame.draw.rect(self.screen, self.obstacle_color, rect)

        gx, gy = env.goal
        pygame.draw.circle(
            self.screen, self.goal_color, (int(gx), int(gy)), int(env.goal_radius)
        )
        pygame.draw.circle(
            self.screen, self.goal_outline, (int(gx), int(gy)), int(env.goal_radius), 3
        )

    def draw_path(self, path_points):
        if len(path_points) >= 2:
            pygame.draw.lines(self.screen, self.path_color, False, path_points, 2)

    def draw_lidar(self, robot_pose, lidar_data):
        x, y, _ = robot_pose
        origin = (int(x), int(y))

        for hit_point in lidar_data["hit_points"]:
            pygame.draw.line(
                self.screen,
                self.lidar_beam_color,
                origin,
                hit_point,
                1
            )
            pygame.draw.circle(
                self.screen,
                self.lidar_hit_color,
                hit_point,
                2
            )

    def draw_text(self, text, x, y, big=False, color=None):
        font = self.big_font if big else self.font
        surface = font.render(text, True, color or self.text_color)
        self.screen.blit(surface, (x, y))

    def render(
        self,
        wheelchair,
        env,
        lidar_data,
        dt,
        elapsed_time,
        step_count,
        episode_status,
        path_points,
    ):
        self.screen.fill(self.bg_color)

        self.draw_environment(env)
        self.draw_path(path_points)

        robot_pose = wheelchair.get_pose()
        if lidar_data is not None:
            if self.show_lidar:
                self.draw_lidar(robot_pose, lidar_data)

        x, y, theta = robot_pose
        self.draw_robot(x, y, theta, wheelchair.radius)

        min_range = float(lidar_data["ranges"].min()) if lidar_data is not None else 0.0

        self.draw_text("Arrow keys: move   |   R: reset", 10, 10)
        self.draw_text(f"x = {x:.1f}", 10, 40)
        self.draw_text(f"y = {y:.1f}", 10, 65)
        self.draw_text(f"theta = {theta:.2f} rad", 10, 90)
        self.draw_text(f"time = {elapsed_time:.2f} s", 10, 115)
        self.draw_text(f"steps = {step_count}", 10, 140)
        self.draw_text(f"dt = {dt:.3f} s", 10, 165)
        self.draw_text(f"min lidar range = {min_range:.1f} px", 10, 190)

        if episode_status == "running":
            self.draw_text("Status: RUNNING", 10, 225, big=True, color=(30, 30, 30))
        elif episode_status == "success":
            self.draw_text("Status: GOAL REACHED", 10, 225, big=True, color=self.success_color)
        elif episode_status == "collision":
            self.draw_text("Status: COLLISION", 10, 225, big=True, color=self.fail_color)
        elif episode_status == "timeout":
            self.draw_text("Status: TIMEOUT", 10, 225, big=True, color=self.fail_color)

        pygame.display.flip()