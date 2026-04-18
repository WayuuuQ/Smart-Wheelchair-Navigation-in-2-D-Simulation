from __future__ import annotations


STEP_EXPLAIN_FIELDS = [
    "alpha",
    "dominant_risk",
    "total_risk",
    "front_min",
    "d_min",
    "ttc",
]


def build_explain_output(alpha, risk_terms, dominant_risk, goal_probs=None):
    return {
        "alpha": float(alpha),
        "risk_terms": risk_terms,
        "dominant_risk": dominant_risk,
        "goal_probs": goal_probs,
    }
