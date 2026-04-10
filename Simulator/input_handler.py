import pygame


class KeyboardTeleop:
    def __init__(self):
        self.forward_speed = 120.0   # pixels/s
        self.reverse_speed = 70.0
        self.turn_speed = 2.5        # rad/s

    def get_command(self):
        keys = pygame.key.get_pressed()

        v_cmd = 0.0
        omega_cmd = 0.0

        if keys[pygame.K_UP]:
            v_cmd = self.forward_speed
        elif keys[pygame.K_DOWN]:
            v_cmd = -self.reverse_speed

        if keys[pygame.K_LEFT]:
            omega_cmd = -self.turn_speed
        elif keys[pygame.K_RIGHT]:
            omega_cmd = self.turn_speed

        return v_cmd, omega_cmd