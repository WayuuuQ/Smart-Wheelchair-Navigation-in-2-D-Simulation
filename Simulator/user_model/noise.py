from __future__ import annotations

import numpy as np


class CommandNoiseModel:
    """Human-like control distortion for scripted batch users."""

    def __init__(self, forward_speed: float, turn_speed: float):
        self.forward_speed = float(forward_speed)
        self.turn_speed = float(turn_speed)
        self.command_delay_steps = 3
        self.command_buffer = []
        self.turn_bias = float(np.random.normal(0.0, 0.08 * self.turn_speed))
        self.wrong_turn_steps = 0
        self.wrong_turn_sign = 0.0

    def _clip(self, value, limit):
        return max(-float(limit), min(float(limit), float(value)))

    def apply(
        self,
        v_cmd,
        omega_cmd,
        heading_error,
        front_min,
        d_min,
        preferred_turn_sign,
    ):
        delayed_cmd = (v_cmd, omega_cmd)
        self.command_buffer.append((float(v_cmd), float(omega_cmd)))
        if len(self.command_buffer) > self.command_delay_steps:
            delayed_cmd = self.command_buffer.pop(0)
        elif self.command_buffer:
            delayed_cmd = self.command_buffer[0]

        distorted_v = float(delayed_cmd[0])
        distorted_omega = 0.62 * float(delayed_cmd[1]) + self.turn_bias

        distorted_v += float(np.random.normal(0.0, 0.02 * self.forward_speed))
        distorted_omega += float(np.random.normal(0.0, 0.06 * self.turn_speed))

        if self.wrong_turn_steps > 0:
            self.wrong_turn_steps -= 1
            distorted_omega += self.wrong_turn_sign * 0.95 * self.turn_speed
            distorted_v *= 0.82
        else:
            risk_trigger = front_min is not None and front_min < 110.0
            turn_trigger = abs(heading_error) > 0.28 and preferred_turn_sign != 0.0
            if risk_trigger and turn_trigger and np.random.rand() < 0.08:
                self.wrong_turn_sign = -preferred_turn_sign
                self.wrong_turn_steps = int(np.random.randint(4, 9))

        if d_min is not None and d_min < 35.0 and np.random.rand() < 0.2:
            distorted_v *= 0.55

        if abs(distorted_omega) > 0.45 * self.turn_speed and distorted_v > 0.0:
            distorted_v *= 0.8

        return (
            self._clip(distorted_v, self.forward_speed),
            self._clip(distorted_omega, self.turn_speed),
        )
