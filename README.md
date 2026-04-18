# Smart Wheelchair Navigation in 2D Simulation

This repository contains a 2D smart wheelchair simulator for studying shared control in static indoor scenes.

The current project supports:

- Interactive keyboard testing
- Four controller modes (`M0` to `M3`)
- Batch experiment execution
- Episode-level and step-level logging
- Metric aggregation and plotting
- Risk-aware adaptive shared control

## Project Overview

The simulator lives in the [`Simulator`](./Simulator) directory.

Main components:

- [`Simulator/main.py`](./Simulator/main.py)
  - Main simulation entry
  - Interactive execution
  - Episode loop
  - Batch logging
- [`Simulator/controllers`](./Simulator/controllers)
  - Controller implementations (`M0` to `M3`)
- [`Simulator/controller.py`](./Simulator/controller.py)
  - Shared control utilities
  - Safety filtering
  - Assist controller
- [`Simulator/user_model`](./Simulator/user_model)
  - Scripted batch user
  - Noise / disturbance model
- [`Simulator/adaptive_alpha`](./Simulator/adaptive_alpha)
  - Risk computation
  - Adaptive `alpha(t)` generation
- [`Simulator/explain`](./Simulator/explain)
  - Explainability field definitions
- [`Simulator/runner`](./Simulator/runner)
  - Batch experiment runner
- [`Simulator/analysis`](./Simulator/analysis)
  - Metric aggregation
  - Plot generation
  - Statistical test placeholder
- [`Simulator/logs/format.md`](./Simulator/logs/format.md)
  - Logging schema
- [`Simulator/scenes`](./Simulator/scenes)
  - Scene definitions

## Controller Modes

The project currently uses four controller IDs:

- `M0`
  - Manual control
  - Executes user command directly
- `M1`
  - Fixed blending
  - Combines user command and assist command with fixed weight
- `M2`
  - Safety filter
  - Applies obstacle-aware filtering to user command
- `M3`
  - Adaptive shared control
  - Computes risk terms and adapts `alpha(t)` with a smooth mapping

Legacy mode aliases are still supported:

- `manual` -> `M0`
- `fixed_alpha` -> `M1`
- `safety_only` -> `M2`
- `adaptive_alpha` -> `M3`

## Scenes

Current default scenes are:

- `s0`: basic static obstacle scene
- `s1`: narrow corridor / bottleneck
- `s2`: narrow passage
- `s3`: T-junction
- `s4`: cluttered obstacle environment

## Requirements

Recommended environment:

- Python 3.10+
- `numpy`
- `pygame`
- `matplotlib` for plotting

If you use Conda, the repository already provides [`environment.yml`](./environment.yml).

If you use `pip`, install at least:

```powershell
pip install numpy pygame matplotlib
```

## Interactive Usage

From the repository root:

```powershell
cd Simulator
py main.py --scene scenes/s1.json --controller M3
```

You can also use legacy mode names:

```powershell
py main.py --scene scenes/s1.json --mode adaptive_alpha
```

Interactive keys:

- `Up / Down / Left / Right`: wheelchair teleoperation
- `1`: switch to `M0`
- `2`: switch to `M1`
- `3`: switch to `M2`
- `4`: switch to `M3`
- `R`: reset scene
- `L`: show / hide LiDAR

## Batch Experiments

Run a small batch example:

```powershell
cd Simulator
py runner\run_batch.py --scenes scenes/s0.json scenes/s1.json --methods M0 M1 M2 M3 --trials 5 --results-dir results
```

Run the default full configuration:

```powershell
cd Simulator
py runner\run_batch.py --scenes scenes/s0.json scenes/s1.json scenes/s2.json scenes/s3.json scenes/s4.json --methods M0 M1 M2 M3 --trials 20 --seed-base 0 --results-dir results
```

## Output Logs

Batch execution writes:

- `episode_results.csv`
- `episode_results.json`
- `step_results.csv`
- `step_results.json`
- `batch_summary.csv`
- `batch_summary.json`

See [`Simulator/logs/format.md`](./Simulator/logs/format.md) for detailed field definitions.

## Metrics and Tables

Generate summary tables from logs:

```powershell
cd Simulator
py analysis\build_metrics.py --episode-input results\episode_results.csv --step-input results\step_results.csv --mode-output results\mode_summary.csv --scene-mode-output results\scene_mode_summary.csv
```

Current main summary tables:

- `mode_summary.csv`
  - `mode`
  - `episodes`
  - `success_rate`
  - `collision_rate`
  - `avg_min_ttc`
  - `avg_workload`
  - `avg_completion_time`
- `scene_mode_summary.csv`
  - `scene`
  - `mode`
  - `controller_id`
  - `episodes`
  - `success_rate`
  - `avg_min_ttc`
  - `avg_workload`

## Plotting

Generate default result plots:

```powershell
cd Simulator
py analysis\plot_results.py --mode-summary results\mode_summary.csv --scene-mode-summary results\scene_mode_summary.csv --figures-dir analysis\figures
```

Current default plots use:

- `success_rate`
- `avg_workload`
- `avg_min_ttc`

If `matplotlib` is missing, the script prints a clear message instead of failing silently.

## Current Shared-Control Design

The current `M3` controller is based on:

- Safety distance
- TTC
- Corridor complexity
- User command instability
- Steering conflict between user and assist

These are combined into risk terms and mapped to `alpha(t)` by a smooth sigmoid-style mapping.

Explainability fields are written to logs, including:

- `alpha`
- `dominant_risk`
- `total_risk`
- `front_min`
- `d_min`
- `ttc`

## Repository Notes

- The project currently focuses on static scenes.
- The batch framework uses a scripted user plus noise model for repeatable comparisons.
- `analysis/stats_tests.py` is still a placeholder and has not been fully implemented.
- Several local `results_*` folders may exist during development; they are not required for normal use.

## Quick Workflow

1. Run one scene interactively to verify behavior.
2. Run batch experiments with `runner/run_batch.py`.
3. Build summary tables with `analysis/build_metrics.py`.
4. Generate figures with `analysis/plot_results.py`.
5. Use the CSV outputs for reports or further statistics.
