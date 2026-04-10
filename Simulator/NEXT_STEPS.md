# Current Handoff

## Scope

This project now supports:

- Four control modes in `main.py`:
  - `manual`
  - `safety_only`
  - `fixed_alpha`
  - `adaptive_alpha`
- Batch experiment execution with:
  - `--batch`
  - `--episodes`
  - `--batch-scenes`
  - `--batch-modes`
  - `--results-dir`

## Main Files Changed

- `main.py`
- `controller.py`

No broad project refactor was done. The existing structure was kept.

## What Was Added

### 1. Batch experiment support

`main.py` can now run all scenes and modes automatically and save:

- `batch_summary.json`
- `batch_summary.csv`

### 2. Batch auto user

The batch runner no longer depends on keyboard input.

`BatchPathUser` was added in `main.py` and currently provides:

- Grid-based A* path planning
- 8-neighbor planning
- Obstacle-clearance cost in planning
- Waypoint tracking
- Local LiDAR-based reactive correction
- Simple stuck detection
- Replanning and recovery behavior

### 3. Shared controller updates

`controller.py` now contains a basic shared controller with:

- Safety filtering
- Minimal assist controller
- Fixed alpha blending
- Adaptive alpha blending

The shared modes were adjusted to be more conservative:

- Translational motion is mostly user-driven
- Shared control mainly adjusts steering
- Reverse speed is limited
- Front risk is used more than global minimum distance

## Current Verified State

The batch framework itself is working.

`manual` mode in batch experiments is now in a good state:

- `s0`: success
- `s1`: success
- `s2`: success
- `s3`: success
- `s4`: success

This means the batch baseline is no longer the main problem.

## Remaining Problem

The main remaining issue is still in `scenes/s1.json` for non-manual modes:

- `safety_only`
- `fixed_alpha`
- `adaptive_alpha`

These modes now avoid many direct collisions, but they can still enter a local deadlock near the later narrow corridor section and time out.

Latest re-checks show that the deadlock is specifically around the small obstacle near the late corridor bottleneck. The current batch user can reach that region reliably, but:

- `safety_only` still falls into rotate/reverse loops
- `fixed_alpha` still keeps some assist steering that does not help break the loop
- `adaptive_alpha` often reduces assist enough, but still does not escape the bottleneck

Recent attempts that did not fully solve it:

- Explicit `recovery_active` context passed from `BatchPathUser` to controller
- Denser waypoint retention in narrow cells
- Corridor context to reduce assist authority
- Recovery-aligned turn preference
- No-progress based escape replanning

This suggests the remaining issue is not just threshold tuning. The batch user and controller still disagree on how to pass the bottleneck.

Observed progression:

- Earlier versions collided there
- Later versions became safer but still oscillated
- Current versions are mostly getting trapped in a conservative rotate/reverse loop

The issue is now mainly controller behavior in constrained spaces, not batch execution logic.

## Suggested Next Step

Do not change the batch framework first.

Focus next on `controller.py`, specifically for `s1`-style narrow corridor behavior:

- Reduce oscillation in safety turning
- Prevent repeated left-right spin loops near local obstacles
- Make assist steering less aggressive in constrained corridors
- Possibly add a short directional commitment or hysteresis in avoidance
- Revisit reverse handling in constrained spaces
- Check whether assist steering should be temporarily suppressed when the auto user is already in recovery mode

The next likely productive direction is:

- Add an explicit "escape waypoint injection" mode in `BatchPathUser` that temporarily replaces the normal path-following target with a side-step target near the bottleneck, instead of only replanning globally
- Or, keep the batch user fixed and change the controller so recovery no longer introduces forward crawl / sign flips when `front_min` stays in the `40-50` range for a long time

## Useful Commands

### Run manual control in `s1`

```powershell
py main.py --scene scenes/s1.json --mode manual
```

### Run batch experiments

```powershell
py main.py --batch --results-dir results
```

### Run only `s1` for all modes

```powershell
py main.py --batch --episodes 1 --batch-scenes scenes/s1.json --batch-modes manual safety_only fixed_alpha adaptive_alpha --results-dir results_s1
```

### Syntax check

```powershell
py -m py_compile main.py controller.py
```
