from __future__ import annotations

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import main as simulator_main

from runner.config import (
    DEFAULT_METHODS,
    DEFAULT_RESULTS_DIR,
    DEFAULT_SCENES,
    DEFAULT_SEED_BASE,
    DEFAULT_TRIALS,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Batch runner wrapper for Smart Wheelchair experiments"
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        default=list(DEFAULT_SCENES),
        help="Scene list, e.g. scenes/s0.json scenes/s1.json",
    )
    parser.add_argument(
        "--methods",
        nargs="+",
        default=list(DEFAULT_METHODS),
        choices=list(DEFAULT_METHODS),
        help="Controller list: M0 M1 M2 M3",
    )
    parser.add_argument(
        "--trials",
        type=int,
        default=DEFAULT_TRIALS,
        help="Number of trials per (scene, method)",
    )
    parser.add_argument(
        "--seed-base",
        type=int,
        default=DEFAULT_SEED_BASE,
        help="Reserved seed base for future deterministic trial control",
    )
    parser.add_argument(
        "--results-dir",
        type=str,
        default=DEFAULT_RESULTS_DIR,
        help="Output directory for episode and batch summary files",
    )
    parser.add_argument(
        "--a0",
        type=float,
        default=0.6,
        help="Fixed blending parameter for M1",
    )
    parser.add_argument(
        "--safety-distance",
        type=float,
        default=75.0,
        help="Safety threshold shared by planner/filter",
    )
    parser.add_argument(
        "--stop-distance",
        type=float,
        default=35.0,
        help="Hard stop threshold shared by planner/filter",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable periodic console logging during trials",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=30,
        help="Print one status line every N steps when verbose is enabled",
    )
    return parser.parse_args()


def build_main_args(args):
    return SimpleNamespace(
        batch=True,
        batch_scenes=list(args.scenes),
        batch_controllers=list(args.methods),
        episodes=int(args.trials),
        results_dir=args.results_dir,
        a0=float(args.a0),
        safety_distance=float(args.safety_distance),
        stop_distance=float(args.stop_distance),
        verbose=bool(args.verbose),
        log_every=int(args.log_every),
        seed_base=int(args.seed_base),
    )


def main():
    args = parse_args()
    batch_args = build_main_args(args)

    pygame = simulator_main._import_pygame()
    pygame.init()
    try:
        simulator_main.run_batch_experiments(batch_args)
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
