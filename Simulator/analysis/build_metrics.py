from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
MODE_SUMMARY_FIELDS = [
    "mode",
    "episodes",
    "success_rate",
    "collision_rate",
    "avg_min_ttc",
    "avg_workload",
    "avg_completion_time",
]
SCENE_MODE_SUMMARY_FIELDS = [
    "scene",
    "mode",
    "controller_id",
    "episodes",
    "success_rate",
    "avg_min_ttc",
    "avg_workload",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build mode-level metrics from episode and step logs"
    )
    parser.add_argument(
        "--episode-input",
        type=str,
        default="results/episode_results.csv",
        help="Path to episode_results.csv",
    )
    parser.add_argument(
        "--step-input",
        type=str,
        default="results/step_results.csv",
        help="Path to step_results.csv",
    )
    parser.add_argument(
        "--mode-output",
        type=str,
        default="results/mode_summary.csv",
        help="Path to mode-level output CSV",
    )
    parser.add_argument(
        "--scene-mode-output",
        type=str,
        default="results/scene_mode_summary.csv",
        help="Path to scene/mode-level output CSV",
    )
    return parser.parse_args()


def load_rows(path):
    path = Path(path)
    if not path.is_absolute():
        path = ROOT_DIR / path
    with open(path, "r", encoding="utf-8", newline="") as csv_file:
        return list(csv.DictReader(csv_file))


def safe_float(value):
    if value in ("", None):
        return None
    return float(value)


def safe_bool(value):
    return str(value).strip().lower() == "true"


def average(values):
    values = [value for value in values if value is not None]
    if not values:
        return None
    return sum(values) / len(values)


def round_or_blank(value, digits=4):
    if value is None:
        return ""
    return round(value, digits)


def build_trial_step_metrics(step_rows):
    grouped = defaultdict(list)
    for row in step_rows:
        grouped[row["trial_id"]].append(row)

    trial_metrics = {}
    for trial_id, items in grouped.items():
        items = sorted(items, key=lambda row: int(row["step_index"]))
        min_ttc = average([])
        ttc_values = [safe_float(row["ttc"]) for row in items]
        valid_ttc = [value for value in ttc_values if value is not None]
        if valid_ttc:
            min_ttc = min(valid_ttc)

        trial_metrics[trial_id] = {
            "sum_abs_dv": sum(abs(float(row["dv"])) for row in items),
            "sum_abs_domega": sum(abs(float(row["domega"])) for row in items),
            "workload": sum(float(row["workload_l1"]) for row in items),
            "min_ttc": min_ttc,
        }
    return trial_metrics


def build_summary_rows(episode_rows, step_rows, group_keys):
    trial_step_metrics = build_trial_step_metrics(step_rows)
    grouped = defaultdict(list)
    for row in episode_rows:
        grouped[tuple(row[key] for key in group_keys)].append(row)

    summary = []
    for group_value in sorted(grouped):
        items = grouped[group_value]
        n = len(items)

        sum_abs_dv_values = []
        sum_abs_domega_values = []
        workload_values = []
        min_ttc_values = []
        for row in items:
            trial_metrics = trial_step_metrics.get(row["trial_id"], {})
            sum_abs_dv_values.append(trial_metrics.get("sum_abs_dv"))
            sum_abs_domega_values.append(trial_metrics.get("sum_abs_domega"))
            workload_values.append(trial_metrics.get("workload"))
            min_ttc_values.append(trial_metrics.get("min_ttc"))

        summary_row = {key: value for key, value in zip(group_keys, group_value)}
        summary_row.update(
            {
                "episodes": n,
                "success_rate": round(sum(1 for row in items if safe_bool(row["success"])) / n, 4),
                "collision_rate": round(sum(1 for row in items if safe_bool(row["collision"])) / n, 4),
                "timeout_rate": round(sum(1 for row in items if safe_bool(row["timeout"])) / n, 4),
                "avg_completion_time": round_or_blank(
                    average(safe_float(row["completion_time"]) for row in items)
                ),
                "avg_path_length": round_or_blank(
                    average(safe_float(row["path_length"]) for row in items)
                ),
                "avg_min_obstacle_distance": round_or_blank(
                    average(safe_float(row["min_obstacle_distance"]) for row in items)
                ),
                "avg_min_ttc": round_or_blank(average(min_ttc_values)),
                "avg_near_collision_count": round_or_blank(
                    average(safe_float(row["near_collision_count"]) for row in items)
                ),
                "avg_sum_abs_dv": round_or_blank(average(sum_abs_dv_values)),
                "avg_sum_abs_domega": round_or_blank(average(sum_abs_domega_values)),
                "avg_workload": round_or_blank(average(workload_values)),
                "avg_intervention_rate": round_or_blank(
                    average(safe_float(row["intervention_rate"]) for row in items)
                ),
                "avg_alpha": round_or_blank(
                    average(safe_float(row["avg_alpha"]) for row in items)
                ),
            }
        )
        summary.append(summary_row)
    return summary


def save_rows(rows, output_path, fieldnames):
    output = Path(output_path)
    if not output.is_absolute():
        output = ROOT_DIR / output
    output.parent.mkdir(parents=True, exist_ok=True)
    with open(output, "w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def main():
    args = parse_args()
    episode_rows = load_rows(args.episode_input)
    step_rows = load_rows(args.step_input)
    mode_summary = build_summary_rows(episode_rows, step_rows, ["mode"])
    scene_mode_summary = build_summary_rows(
        episode_rows, step_rows, ["scene", "mode", "controller_id"]
    )
    save_rows(mode_summary, args.mode_output, MODE_SUMMARY_FIELDS)
    save_rows(scene_mode_summary, args.scene_mode_output, SCENE_MODE_SUMMARY_FIELDS)
    print(f"Saved mode summary to: {args.mode_output}")
    print(f"Saved scene/mode summary to: {args.scene_mode_output}")


if __name__ == "__main__":
    main()
