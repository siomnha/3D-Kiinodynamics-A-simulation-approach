# traj_gen mode replication analysis (for this repo)

> Note: direct online access to `github.com/TareqAlqutami/traj_gen` is blocked in this environment, so this analysis is based on common trajectory-generator mode families and is structured to be verified against the upstream repo once reachable.

## Mode characteristics and replication priority

| Mode | Typical geometry | Smoothness/derivative profile | Main usage | Parameters to expose | Replication priority |
|---|---|---|---|---|---|
| hover | single fixed point | trivial (zero vel/acc target) | controller idle/hold tests | center, duration | High |
| line | straight segment | constant-velocity friendly | tracking baseline, gain tuning | start/end, duration | High |
| circle | closed periodic loop | continuous curvature, constant-speed feasible | steady-state lateral tracking | center, radius, period | High |
| figure8 | two-lobe periodic loop | alternating curvature sign, richer excitation | coupled axis test and yaw policy test | center, scale x/y, period | High |
| helix/spiral | circle + monotonic z | 3D coupling stress test | ascent/descent under lateral motion | center, radius, turns, z gain | Medium |
| waypoint list | arbitrary polyline | quality depends on pruning/smoothing | mission-shaped trajectories | waypoint list, speed limits | High |
| random/lawnmower | coverage-style path | aggressive corners if unsmoothed | robustness and map coverage experiments | bounds, seed, step | Medium |

## Mapping into this repository

1. Keep `path_pruner` to clean A* local noise before smoothing.
2. Keep `trajectory_adapter` as the conversion stage from pruned waypoints to smooth sampled trajectory.
3. Add mode source generator for RViz/system tests independent from planner.
4. Keep `trajectory_sampler` as execution/output bridge toward `/space_cobot/pose` and `/trajectory/state`.

## What is already implemented now

- `trajectory_mode_generator` supports: `hover`, `line`, `circle`, `figure8`, `helix`.
- Output contract: `nav_msgs/Path` on `/trajectory/reference`.
- This lets you benchmark follower and controller layers without changing planner internals.

## Gap vs full traj_gen-style replication

To reach strict parity with mature minimum-snap generators:

1. Add true polynomial optimization backend (minimum snap / minimum jerk objective + continuity constraints).
2. Export reference states beyond pose (velocity, acceleration, jerk, yaw, yaw_rate).
3. Add corridor-constrained optimization or post-optimization collision repair.
4. Add mode-specific boundary conditions presets (e.g., stop-to-stop vs periodic loops).

## Recommended verification checklist against upstream `traj_gen`

When upstream access is available, compare and backfill:

- exact mode names and launch/config schema
- objective type (snap vs jerk) and polynomial order
- continuity order and endpoint constraints
- message types and expected sampling outputs
- parameter defaults and safety checks

## Parameter tuning map for nav6d A* + pruning + optimize_traj

All these are now primarily tuned in `nav6d_sim/nav_6d_optimize_traj.py` (legacy mirror exists in `trajectory_adapter`) as ROS parameters:

- `v_ref`, `a_ref`: nominal segment-time model.
- `max_velocity`, `max_acceleration`: safety caps for effective speed/accel assumptions.
- `time_scale`: global aggressiveness knob (`>1.0` slower/safer, `<1.0` faster).
- `corner_time_gain`: turn-aware penalty from local corner sharpness.
- `min_segment_time`: prevents very short unstable segments after pruning.
- `sample_dt`: output sample density (smaller => denser trajectory).

Suggested baseline for RViz then real follower integration:
- Start: `v_ref=1.2`, `a_ref=0.8`, `max_velocity=2.0`, `max_acceleration=1.5`, `time_scale=1.15`, `corner_time_gain=0.35`, `sample_dt=0.08`.
- If corners overshoot: raise `corner_time_gain` and `time_scale`.
- If too conservative: reduce `time_scale` toward `1.0`, then increase `v_ref` gradually while staying below caps.


## nav_6d integrated optimize_traj development entry
For current development focus, use `nav6d_sim/nav_6d_optimize_traj.py` as the single integration point.

Where to tune design parameters (all ROS params on this node):
- `v_ref`, `a_ref`: nominal time-allocation model.
- `max_velocity`, `max_acceleration`: hard dynamic assumptions.
- `time_scale`: global speed/safety scaling.
- `corner_time_gain`: extra slowdown on sharp pruned corners.
- `min_segment_time`, `sample_dt`: temporal granularity/stability.

Pipeline in this node:
1. subscribe `/nav6d/planner/path`
2. prune (duplicate removal + RDP)
3. allocate segment time with corner penalty
4. run internal `optimize_traj` trajectory generation
5. publish `/planning/pruned_path`, `/trajectory/reference`, `/trajectory/state`, `/space_cobot/pose`
