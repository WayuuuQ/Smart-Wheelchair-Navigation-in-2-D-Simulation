# Smart Wheelchair Navigation in 2D Simulation

This project is a 2D smart wheelchair simulation for testing manual control, safety filtering, and shared control strategies in static indoor scenes.

The simulator supports:

- Interactive keyboard control
- Four control modes
- Multiple benchmark scenes
- Batch experiment execution
- Episode-level and aggregated experiment metrics

## Control Modes

The project currently supports four modes:

- `manual`
  - Directly executes user input without intervention
- `safety_only`
  - Applies a safety filter to user input
  - Slows down or stops near obstacles and biases turning toward the more open side
- `fixed_alpha`
  - Blends user command and assist command with a fixed weight
- `adaptive_alpha`
  - Blends user command and assist command with a risk-dependent adaptive weight

## Project Structure

Main files:

- [main.py](./main.py)
  - Entry point
  - Interactive simulation loop
  - Batch experiment runner
  - Experiment metric recording and result export
- [controller.py](./controller.py)
  - Safety filter
  - Minimal assist controller
  - Fixed-alpha and adaptive-alpha blending logic
- [environment.py](./environment.py)
  - Scene loading
  - Obstacle and boundary collision checking
  - Goal checking
- [input_handler.py](./input_handler.py)
  - Keyboard teleoperation input
- [sensor.py](./sensor.py)
  - LiDAR simulation
- [renderer.py](./renderer.py)
  - Pygame visualization
- [wheelchair.py](./wheelchair.py)
  - Wheelchair kinematics
- [scenes](./scenes)
  - Benchmark scene definitions

## Requirements

Recommended environment:

- Python 3.10+
- `pygame`
- `numpy`

Install dependencies:

```powershell
pip install pygame numpy
```

## Run Interactive Simulation

Run a scene in one control mode:

```powershell
py main.py --scene scenes/s1.json --mode manual
```

Available modes:

```powershell
manual
safety_only
fixed_alpha
adaptive_alpha
```

During simulation:

- `Up / Down / Left / Right`: wheelchair teleoperation
- `1`: switch to `manual`
- `2`: switch to `safety_only`
- `3`: switch to `fixed_alpha`
- `4`: switch to `adaptive_alpha`
- `R`: reset scene
- `L`: toggle LiDAR rendering

## Run Batch Experiments

Run all default scenes and all control modes:

```powershell
py main.py --batch --results-dir results
```

Run selected scenes and modes:

```powershell
py main.py --batch --batch-scenes scenes/s0.json scenes/s1.json --batch-modes manual adaptive_alpha --results-dir results
```

Run multiple episodes:

```powershell
py main.py --batch --episodes 3 --results-dir results
```

## Output Files

After a batch run, the result directory contains:

- `episode_results.json`
- `episode_results.csv`
- `batch_summary.json`
- `batch_summary.csv`

### 1. Episode-Level Results

`episode_results.*` stores one row per episode.

Recorded fields include:

- `scene`
- `mode`
- `episode_index`
- `success`
- `collision`
- `timeout`
- `completion_time`
- `path_length`
- `min_obstacle_distance`
- `avg_obstacle_distance`
- `near_collision_count`
- `reverse_count`
- `reverse_time`
- `stuck_count`
- `recovery_count`
- `oscillation_count`
- `avg_abs_v_diff`
- `avg_abs_omega_diff`
- `intervention_rate`
- `avg_alpha`
- `max_alpha`

Notes:

- Path length is accumulated from pose increments.
- Obstacle distance is derived from LiDAR minimum range.
- `manual` mode keeps the same output schema, with unavailable control-sharing quantities filled as `0`.

### 2. Aggregated Batch Summary

`batch_summary.*` aggregates results by `(scene, mode)`.

Fields include:

- `episodes`
- `success_rate`
- `collision_rate`
- `timeout_rate`
- Average values of the numeric episode metrics

## Metric Threshold Constants

The following constants are defined in [main.py](./main.py):

- `NEAR_COLLISION_DISTANCE = 40.0`
- `INTERVENTION_V_THRESHOLD = 1.0`
- `INTERVENTION_OMEGA_THRESHOLD = 0.1`
- `OSCILLATION_OMEGA_THRESHOLD = 1.0`

## Scenes

The default batch runner uses:

- `scenes/s0.json`
- `scenes/s1.json`
- `scenes/s2.json`
- `scenes/s3.json`
- `scenes/s4.json`

These scenes cover:

- Basic static navigation
- Narrow corridor navigation
- Passage and bottleneck behavior
- T-junction navigation
- Cluttered environment navigation

## Current Shared Control Design

The current assist logic is intentionally lightweight:

- The assist controller uses pose, goal, and LiDAR
- The safety controller uses front and lateral obstacle distance
- Shared modes mainly affect steering while preserving user translational intent as much as possible

This keeps the project simple and suitable for controlled comparisons across modes.

## Recommended Workflow

1. Run interactive tests to verify behavior in a single scene
2. Run batch experiments across all scenes
3. Compare `episode_results.csv` for per-episode details
4. Compare `batch_summary.csv` for scene/mode-level statistics

## Example

Interactive:

```powershell
py main.py --scene scenes/s1.json --mode adaptive_alpha
```

Batch:

```powershell
py main.py --batch --results-dir results
```

## Notes

- This project currently focuses on static scenes.
- The batch framework uses an internal scripted path-following user for fair repeated evaluation.
- Scene `s1` was tuned to remain narrow but passable for all four control modes.
