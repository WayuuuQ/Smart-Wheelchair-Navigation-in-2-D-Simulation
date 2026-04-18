from __future__ import annotations

import argparse
import csv
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate first-pass experiment figures from summary CSV files"
    )
    parser.add_argument(
        "--mode-summary",
        type=str,
        default="results/mode_summary.csv",
        help="Path to mode_summary.csv",
    )
    parser.add_argument(
        "--scene-mode-summary",
        type=str,
        default="results/scene_mode_summary.csv",
        help="Path to scene_mode_summary.csv",
    )
    parser.add_argument(
        "--figures-dir",
        type=str,
        default="analysis/figures",
        help="Directory for generated figures",
    )
    parser.add_argument(
        "--metrics",
        nargs="+",
        default=["success_rate", "avg_workload", "avg_min_ttc"],
        help="Metrics to plot from summary files",
    )
    return parser.parse_args()


def resolve_path(path_str):
    path = Path(path_str)
    if not path.is_absolute():
        path = ROOT_DIR / path
    return path


def load_rows(path_str):
    path = resolve_path(path_str)
    with open(path, "r", encoding="utf-8", newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def maybe_import_matplotlib():
    try:
        import matplotlib.pyplot as plt  # type: ignore
    except ModuleNotFoundError:
        return None
    return plt


def safe_float(value):
    if value in ("", None):
        return None
    return float(value)


def plot_mode_bars(rows, metric, figures_dir, plt):
    if rows and metric not in rows[0]:
        print(f"Skip mode plot for missing metric: {metric}")
        return
    values = []
    labels = []
    for row in rows:
        value = safe_float(row.get(metric))
        if value is None:
            continue
        labels.append(row["mode"])
        values.append(value)

    if not values:
        return

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(labels, values, color=["#4c78a8", "#f58518", "#54a24b", "#e45756"][: len(values)])
    ax.set_title(f"Mode Comparison: {metric}")
    ax.set_ylabel(metric)
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figures_dir / f"{metric}_by_mode.png", dpi=150)
    plt.close(fig)


def plot_scene_mode_bars(rows, metric, figures_dir, plt):
    if rows and metric not in rows[0]:
        print(f"Skip scene/mode plot for missing metric: {metric}")
        return
    scene_names = sorted({row["scene"] for row in rows})
    mode_names = sorted({row["mode"] for row in rows})

    if not scene_names or not mode_names:
        return

    x = list(range(len(scene_names)))
    width = 0.8 / max(1, len(mode_names))
    fig, ax = plt.subplots(figsize=(10, 5))

    for mode_index, mode in enumerate(mode_names):
        offsets = [idx + (mode_index - (len(mode_names) - 1) / 2.0) * width for idx in x]
        values = []
        for scene in scene_names:
            row = next(
                (
                    item
                    for item in rows
                    if item["scene"] == scene and item["mode"] == mode
                ),
                None,
            )
            values.append(0.0 if row is None else (safe_float(row.get(metric)) or 0.0))
        ax.bar(offsets, values, width=width, label=mode)

    ax.set_title(f"Scene/Mode Comparison: {metric}")
    ax.set_ylabel(metric)
    ax.set_xticks(x)
    ax.set_xticklabels([Path(scene).stem for scene in scene_names])
    ax.legend()
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig(figures_dir / f"{metric}_by_scene_mode.png", dpi=150)
    plt.close(fig)


def main():
    args = parse_args()
    mode_rows = load_rows(args.mode_summary)
    scene_mode_rows = load_rows(args.scene_mode_summary)
    figures_dir = resolve_path(args.figures_dir)
    figures_dir.mkdir(parents=True, exist_ok=True)

    plt = maybe_import_matplotlib()
    if plt is None:
        print("matplotlib is not installed. Summary CSV files are ready, but figures were not generated.")
        print(f"Expected mode summary: {resolve_path(args.mode_summary)}")
        print(f"Expected scene/mode summary: {resolve_path(args.scene_mode_summary)}")
        print(f"Planned figures directory: {figures_dir}")
        return

    for metric in args.metrics:
        plot_mode_bars(mode_rows, metric, figures_dir, plt)
        plot_scene_mode_bars(scene_mode_rows, metric, figures_dir, plt)

    print(f"Saved figures to: {figures_dir}")


if __name__ == "__main__":
    main()
