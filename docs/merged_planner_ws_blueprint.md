# Merged Planner Blueprint (planner_ws): A* + Gazebo OctoMap now, Replan Trigger next

This file is a clean integration format for your **planner_ws stage**.

Scope:
- achieve stable **A* + octomap_server in Gazebo** first,
- use **static snapshot planning + trigger-ready online monitor**,
- keep TF from **pathgazeobo** as current TF authority,
- adopt **continuous trajectory Pattern C** (global static + local dynamic safety layer).

> Note: In this repository snapshot, no `planner_ws/nav6d` source tree is present, so this blueprint is architecture-first and interface-driven for immediate integration.

---

## 1) Target operating mode (current phase)

### Phase target
1. Gazebo Harmonic + Iris depth camera builds online OctoMap.
2. Planner consumes **static snapshot map** for global A*.
3. Optimizer produces continuous trajectory reference.
4. Online OctoMap stream is monitored for replan conditions (design now, implementation next).

### Why this mode
- low integration risk now,
- deterministic global planning,
- still prepares for dynamic obstacle response.

---

## 2) Canonical merged graph (clean node format)

```text
[Gazebo + Iris depth]
   -> ros_gz_bridge/depth_image_proc
   -> /depth/points (PointCloud2)
   -> octomap_server
   -> /octomap_full (octomap_msgs/Octomap)

[Map snapshot manager]
   /octomap_full -> export snapshot.bt -> planner map reload event

[Mission manager / waypoint loader]
   /waypoints -> /nav6d/goal

[nav6d planner (planner_ws)]
   map snapshot + /nav6d/goal -> /nav6d/planner/path

[nav_6d_optimize_traj]
   /nav6d/planner/path -> /planning/pruned_path -> /trajectory/reference
                       -> /trajectory/state + /space_cobot/pose

[pathgazeobo TF authority]
   publish map->base_link (+ base_link->camera extrinsics)

[replan trigger monitor (future)]
   /octomap_full + /trajectory/reference + TF + progress metrics
   -> /planning/replan_request (Bool/Event)
```

---

## 3) TF policy (with pathgazeobo broadcast)

Use **single TF authority**:
- `map -> base_link`: from pathgazeobo bridge (current decision).
- camera extrinsic: one source only (either SDF-consistent static TF or bridge-provided TF).

Do not run multiple conflicting broadcasters for `map -> base_link` in parallel.

---

## 4) Static snapshot planning + online trigger analysis

## 4.1 Snapshot side (global planner input)

`SnapshotManager` behavior:
1. Subscribe `/octomap_full`.
2. Every `snapshot_period_s` or significant map update, serialize to `.bt`.
3. Notify planner to reload map (`/planning/map_reload` event/service).
4. Apply debounce/rate limit to avoid excessive planner churn.

Suggested parameters:
- `snapshot_period_s` (e.g. 3–10 s)
- `map_reload_min_interval_s` (e.g. 2–5 s)
- `map_change_voxel_threshold`
- `snapshot_path`

## 4.2 Online octomap message for trigger side (future replan)

Even if global planner stays snapshot-based, subscribe online `/octomap_full` for trigger detection:

Trigger checks from online map:
1. **Corridor occupancy delta**: newly occupied voxels intersect current trajectory corridor.
2. **Forward collision horizon**: occupancy appears within lookahead distance/time.
3. **Map confidence staleness**: no map updates for too long while vehicle is moving.

Output of monitor:
- publish `/planning/replan_request` with reason code:
  - `MAP_BLOCKED`, `PROGRESS_STUCK`, `TF_DEGRADED`, `SAFETY_MARGIN_LOW`.

---

## 5) Continuous trajectory Pattern C (recommended now)

Pattern C = **global static route + local dynamic safety adaptation**.

## 5.1 Global layer
- Plan A* from snapshot map through mission intent.
- Keep multi-waypoint continuity objective (not stop-go per waypoint).

## 5.2 Trajectory layer
- `nav_6d_optimize_traj` creates smooth sampled reference.
- Preserve pass-through behavior by avoiding hard stop at intermediate waypoints.

## 5.3 Local dynamic layer (future)
- Online monitor checks hazards near active trajectory.
- If hazard intersects corridor, request replan and splice new segment into remaining trajectory.

---

## 6) Interface contract for planner_ws integration

Required topics/services:

### Inputs
- `/nav6d/goal` (`geometry_msgs/PoseStamped`)
- map snapshot path or map reload event
- TF (`map -> base_link`)

### Outputs
- `/nav6d/planner/path` (`nav_msgs/Path`)

### New events (recommended)
- `/planning/map_reload` (event/service)
- `/planning/replan_request` (event with reason)
- `/planning/replan_status` (accepted/rejected/rate_limited)

---

## 7) Implementation order (clean rollout)

1. **Lock TF source** to pathgazeobo bridge.
2. Run Gazebo -> OctoMap pipeline; verify `/octomap_full` quality.
3. Add snapshot manager and manual map reload trigger.
4. Validate global A* from snapshot + optimizer output.
5. Add replan monitor in observe-only mode (log-only).
6. Enable replan request events with rate limit.
7. Add trajectory splice logic for uninterrupted flight.

---

## 8) Minimal acceptance checklist

- A* path generated from snapshot map on mission goal.
- Optimized trajectory published continuously on `/trajectory/reference`.
- TF tree stable with one `map -> base_link` authority.
- Online `/octomap_full` monitor detects blocked corridor and emits trigger reason.
- Replan request path exists (even if action not enabled yet).

---

## 9) Immediate next coding tasks (planner_ws)

1. `snapshot_manager` node (new):
   - `/octomap_full` subscriber,
   - `.bt` exporter,
   - planner reload signaling.

2. `replan_monitor` node (new, observe-only first):
   - subscribe map + active trajectory + TF,
   - compute corridor occupancy delta,
   - publish `replan_request` events.

3. `trajectory_stitcher` update (in optimizer side):
   - keep continuity when switching from old to replanned trajectory segment.

This reaches your requested milestone: **clean merged A* + octomap_server flow in Gazebo now, with future-ready replan trigger path.**
