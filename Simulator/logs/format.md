# Log Format

This document freezes the first-pass logging contract for batch experiments.

## Output Files

- `episode_results.csv`: one row per trial
- `episode_results.json`: JSON copy of episode-level results
- `step_results.csv`: one row per simulation step in batch mode
- `step_results.json`: JSON copy of step-level results
- `batch_summary.csv`: grouped summary by `(scene, mode, controller_id)`
- `batch_summary.json`: JSON copy of grouped summary

Current limitation:
- goal accuracy, early accuracy, and wrong-commit are not emitted yet

## Episode-Level Fields

Required identifiers:

- `trial_id`: stable identifier, e.g. `s0_M0_000`
- `scene`: scenario JSON path
- `mode`: stable method name (`manual`, `fixed_alpha`, `safety_only`, `adaptive_alpha`)
- `controller_id`: controller code (`M0`, `M1`, `M2`, `M3`)
- `episode_index`: zero-based trial index within one `(scene, method)` group
- `seed`: integer seed used for the trial

Termination fields:

- `success`: boolean
- `collision`: boolean
- `timeout`: boolean
- `status`: string summary of terminal condition
- `completion_time`: seconds until terminal condition

Efficiency fields:

- `elapsed_time`: seconds
- `step_count`: number of simulation steps
- `path_length`: cumulative traveled distance in pixels
- `goal_distance`: final distance to goal in pixels

Safety fields:

- `min_obstacle_distance`: minimum LiDAR distance observed in the trial
- `min_ttc`: minimum forward time-to-collision estimate in seconds
- `avg_obstacle_distance`: mean LiDAR minimum distance across steps
- `near_collision_count`: number of steps where obstacle distance is below the near-collision threshold

Recovery and failure-mode fields:

- `reverse_count`: number of reverse motion segments
- `reverse_time`: total reverse time in seconds
- `stuck_count`: number of stuck detections
- `recovery_count`: number of recovery mode entries
- `oscillation_count`: number of left/right turning sign flips above the oscillation threshold

Shared-control workload fields:

- `avg_abs_v_diff`: mean absolute difference between executed and human linear velocity
- `avg_abs_omega_diff`: mean absolute difference between executed and human angular velocity
- `intervention_rate`: ratio of steps where controller changed the user command above thresholds
- `avg_alpha`: mean alpha value over control steps
- `max_alpha`: maximum alpha value over control steps
- `final_alpha`: alpha value on the final step

Debug-only fields:

- `final_pose`
- `final_user_cmd`
- `final_assist_cmd`

## Batch Summary Fields

Each row aggregates all episodes with the same `(scene, mode, controller_id)`:

- `episodes`
- `success_rate`
- `collision_rate`
- `timeout_rate`
- average versions of episode-level numeric metrics

## Step-Level Fields

Currently emitted in `step_results.*`:

- `trial_id`
- `scene`
- `mode`
- `controller_id`
- `episode_index`
- `seed`
- `t`
- `step_index`
- `pose_x`, `pose_y`, `pose_theta`
- `user_v`, `user_omega`
- `exec_v`, `exec_omega`
- `assist_v`, `assist_omega`
- `alpha`
- `dominant_risk`
- `total_risk`
- `front_min`
- `d_min`
- `ttc`
- `dv`
- `domega`
- `abs_v_diff`
- `abs_omega_diff`
- `workload_l1`
- `recovery_active`
- `intervened`

Reserved for future extension:
- `goal_accuracy`
- `early_accuracy_at_T`
- `wrong_commit`

## Thresholds

Current thresholds are defined in `main.py`:

- `NEAR_COLLISION_DISTANCE`
- `INTERVENTION_V_THRESHOLD`
- `INTERVENTION_OMEGA_THRESHOLD`
- `OSCILLATION_OMEGA_THRESHOLD`
