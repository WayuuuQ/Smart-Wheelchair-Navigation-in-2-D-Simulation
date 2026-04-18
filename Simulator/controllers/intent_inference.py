from __future__ import annotations

import math
from collections import deque


class SimpleIntentInference:
    """Minimal intent inference with temporal smoothing.

    This module estimates:
    - goal alignment with the navigation target
    - left / right turning preference
    - forward intent
    - an overall confidence score
    """

    def __init__(self, max_v: float, max_omega: float, history_size: int = 8, beta: float = 0.8):
        self.max_v = float(max_v)
        self.max_omega = float(max_omega)
        self.history = deque(maxlen=int(history_size))
        self.beta = float(beta)
        self.state = {
            "goal_alignment": 0.5,
            "turn_left_prob": 0.5,
            "turn_right_prob": 0.5,
            "forward_intent": 0.0,
            "intent_confidence": 0.0,
            "dominant_intent": "forward",
        }

    def _blend(self, key: str, value: float) -> float:
        previous = float(self.state.get(key, value))
        blended = self.beta * previous + (1.0 - self.beta) * float(value)
        self.state[key] = blended
        return blended

    def update(self, obs: dict, user_cmd: tuple[float, float]) -> dict:
        pose = obs["pose"]
        goal = obs["goal"]
        v_cmd = float(user_cmd[0])
        omega_cmd = float(user_cmd[1])

        x, y, theta = pose
        goal_heading = math.atan2(goal[1] - y, goal[0] - x)
        heading_error = math.atan2(
            math.sin(goal_heading - theta), math.cos(goal_heading - theta)
        )

        self.history.append(omega_cmd)
        avg_recent_omega = sum(self.history) / max(1, len(self.history))

        goal_alignment_now = max(0.0, math.cos(heading_error))
        forward_intent_now = max(0.0, min(1.0, v_cmd / max(1e-6, self.max_v)))

        left_score = max(0.0, avg_recent_omega) / max(1e-6, self.max_omega)
        right_score = max(0.0, -avg_recent_omega) / max(1e-6, self.max_omega)
        total_turn = left_score + right_score
        if total_turn < 1e-6:
            left_prob_now = 0.5
            right_prob_now = 0.5
        else:
            left_prob_now = left_score / total_turn
            right_prob_now = right_score / total_turn

        goal_alignment = self._blend("goal_alignment", goal_alignment_now)
        forward_intent = self._blend("forward_intent", forward_intent_now)
        turn_left_prob = self._blend("turn_left_prob", left_prob_now)
        turn_right_prob = self._blend("turn_right_prob", right_prob_now)

        turn_margin = abs(turn_left_prob - turn_right_prob)
        confidence_now = 0.45 * goal_alignment + 0.35 * forward_intent + 0.20 * turn_margin
        intent_confidence = self._blend("intent_confidence", confidence_now)

        if forward_intent >= max(turn_left_prob, turn_right_prob):
            dominant_intent = "forward"
        else:
            dominant_intent = "left" if turn_left_prob >= turn_right_prob else "right"
        self.state["dominant_intent"] = dominant_intent

        return {
            "goal_alignment": float(goal_alignment),
            "turn_left_prob": float(turn_left_prob),
            "turn_right_prob": float(turn_right_prob),
            "forward_intent": float(forward_intent),
            "intent_confidence": float(intent_confidence),
            "dominant_intent": dominant_intent,
        }
