from __future__ import annotations

DEFAULT_SCENES = (
    "scenes/s0.json",
    "scenes/s1.json",
    "scenes/s2.json",
    "scenes/s3.json",
    "scenes/s4.json",
)

DEFAULT_METHODS = ("M0", "M1", "M2", "M3")
DEFAULT_TRIALS = 20
DEFAULT_SEED_BASE = 0
DEFAULT_RESULTS_DIR = "results"

METHOD_NAME_MAP = {
    "M0": "manual",
    "M1": "fixed_alpha",
    "M2": "safety_only",
    "M3": "adaptive_alpha",
}
