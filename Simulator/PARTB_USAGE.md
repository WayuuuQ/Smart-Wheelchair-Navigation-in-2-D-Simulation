# Part B Usage (Controllers M0-M3)

This repo implements Part B controllers with a unified interface:

`action = controller.get_action(obs, user_cmd)` where `action["v"]`, `action["omega"]` are executed.

## How To Run (Interactive)

From repo root:

```bash
cd Simulator
python main.py --scene scenes/s1.json --controller M0
```

Controllers:
- `M0`: Manual, `u = u_h`
- `M1`: Fixed blending, `u = a0*u_h + (1-a0)*u_a`
- `M2`: Safety filter, `u = SafetyFilter(u_h)`
- `M3`: Skeleton (placeholder for Part C), currently behaves like manual but reserves fields (`alpha/goal_probs/risk_terms`)

During interactive rendering, you can also press keys:
- `1` -> M0, `2` -> M1, `3` -> M2, `4` -> M3

## Key Parameters (M1 / M2)

### M1: Fixed Blending Weight `a0`

Default `a0=0.6`. You can override with CLI:

```bash
cd Simulator
python main.py --scene scenes/s1.json --controller M1 --a0 0.6
python main.py --scene scenes/s1.json --controller M1 --a0 0.8
```

### M2 (and M1 local planner): Safety Thresholds

Both M2 safety filter and M1 local planner use the same distance thresholds (passed into `SharedController`):

- `--safety-distance` (default `75.0`): start scaling down forward motion / increasing avoidance influence
- `--stop-distance` (default `35.0`): hard-stop region for forward motion (safety override)

Example:

```bash
cd Simulator
python main.py --scene scenes/s1.json --controller M2 --safety-distance 90 --stop-distance 45
```

## Disable Periodic Console Logs (No Spam)

By default the simulator does not print periodic step logs.

Options:
- `--verbose`: enable low-frequency status prints (default off)
- `--log-every N`: when `--verbose` is set, print one status line every `N` steps (default `30`)

Examples:

```bash
cd Simulator
python main.py --scene scenes/s1.json --controller M1 --verbose
python main.py --scene scenes/s1.json --controller M1 --log-every 60
python main.py --scene scenes/s1.json --controller M1 --verbose --log-every 30
```

## Backward Compatibility: `--mode` (Deprecated)

`--mode` is kept for older scripts, but is deprecated (use `--controller`).

Mapping:
- `--mode manual` -> `--controller M0`
- `--mode fixed_alpha` -> `--controller M1`
- `--mode safety_only` -> `--controller M2`
- `--mode adaptive_alpha` -> `--controller M3`

## Quick Smoke Test (No Pygame Window)

Run:

```bash
python controllers/smoke_test_controllers.py
```

This constructs a minimal fake `obs` (only the fields actually provided by `main.py`) and checks that each controller returns an `action` dict containing at least:
`v`, `omega`, `u_h`, `u_a`, `alpha`.
