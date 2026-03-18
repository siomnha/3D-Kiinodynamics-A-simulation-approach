# Online Merge Strategy: pathgazeobo + nav6d + nav6d_sim (ROS 2 Humble, Gazebo Harmonic)

This document is a **new procedure guide** (without modifying the current `README.md`) to explain:

1. How to merge the current packages into one online-capable pipeline.
2. What each procedure does.
3. What topic bridging is required between Gazebo and ROS 2.
4. How to handle the current limitation that nav6d A* is mainly used with prebuilt `.bt` files.
5. A **strategy-only** replan trigger design (not implemented yet) for online planning/failsafe.

---

## 1) Where each package should be in your workspace

Recommended layout:

```text
~/ros2_ws/src/
├── nav6d/                                # 3D A* planner package
├── nav6d_sim/                            # this repository root
│   ├── nav6d_sim/                        # Python nodes
│   └── pathgazeobo-main/                 # Gazebo/ArduPilot/octomap scripts & models
├── waypoint_rviz2_plugin/                # optional RViz waypoint tool
└── (optional) iris_description/
```

Why:
- `nav6d` handles A* search and occupancy query.
- `pathgazeobo-main` handles Gazebo Harmonic + ArduPilot Iris depth sensor + octomap pipeline scripts.
- `nav6d_sim` handles mission goal sequencing, pruning, and minimum-snap style trajectory generation.

---

## 2) Merge procedure (phased) and what each phase does

## Phase A — Build online map source from Gazebo

Use `pathgazeobo-main/scripts/run_iris_octomap_pipeline.sh` to run:
- Gazebo world,
- depth image/pointcloud bridging,
- and `octomap_server`.

What this phase is doing:
- Sensor simulation: Iris depth camera creates depth observations.
- Bridge: Gazebo messages become ROS 2 `Image`/`CameraInfo`/`PointCloud2`.
- Mapping: `octomap_server` fuses point cloud into OctoMap in real time.

Result:
- ROS 2 OctoMap topics become available for downstream planner consumption.

## Phase B — Goal and mission input

Use either:
- RViz waypoint tool (`/waypoints`) OR
- `mavproxy_waypoint_loader` from txt mission file (`/waypoints`) OR
- direct goal publisher to `/nav6d/goal`.

What this phase is doing:
- Converts user/operator mission intention into planner goals.
- `mission_manager` sequences waypoints and publishes one goal at a time.

## Phase C — Path planning and optimization chain

Pipeline:
- `nav6d` outputs `/nav6d/planner/path`.
- `nav_6d_optimize_traj` prunes path + allocates segment timing + outputs smooth reference trajectory.

What this phase is doing:
- Converts grid/graph A* output into flyable trajectory with velocity/acceleration-aware timing.

## Phase D — Control interface/flight integration (next coupling stage)

Current repo publishes trajectory/state topics for simulation and visualization.

What this phase is doing:
- Provides a stable interface layer for future ArduPilot setpoint or offboard-control integration.

---

## 3) Topic bridging plan (Gazebo <-> ROS 2)

`run_iris_octomap_pipeline.sh` supports two bridge modes.

## A) `image_proc` mode (default)

Gazebo -> ROS bridges:
- `.../sensor/<depth_sensor>/image` -> `sensor_msgs/msg/Image`
- `.../sensor/<depth_sensor>/camera_info` -> `sensor_msgs/msg/CameraInfo`

Then ROS processing:
- `depth_image_proc/point_cloud_xyz_node`: `Image + CameraInfo` -> `sensor_msgs/msg/PointCloud2` (`/depth/points` by default)

Then mapping:
- `octomap_server` subscribes `cloud_in` (typically `/depth/points`).

When to use:
- Best compatibility when point-cloud packed bridge is unstable or unavailable.

## B) `points_direct` mode

Gazebo -> ROS bridge:
- `.../sensor/<depth_sensor>/points` (Gazebo `PointCloudPacked`) -> ROS `sensor_msgs/msg/PointCloud2`

Then mapping:
- `octomap_server` subscribes this bridged point cloud directly.

When to use:
- Lower node count; good if your Gazebo point cloud bridge is reliable.

## TF requirement

For a global map (`frame_id = map`):
- You need a valid TF chain from map to the sensor/body frame.

If no stable global TF yet:
- Temporarily map in body frame for local mapping stability, then migrate to `map` frame once localization is stable.

---

## 4) Current limitation: nav6d often used with offline `.bt` maps

You noted correctly that nav6d A* currently behaves as if it expects a static/downloaded `.bt` map.

### Practical cooperation strategy (without deep planner rewrite)

### Strategy 1 — Sliding static snapshots (short-term, low risk)

Procedure:
1. Keep online `octomap_server` running.
2. Periodically export/refresh a `.bt` snapshot.
3. Trigger planner map reload at a fixed interval or event.

Effect:
- Near-online behavior with bounded staleness.
- Minimal nav6d code change if reload hooks exist.

Tradeoff:
- Not truly continuous occupancy updates.

### Strategy 2 — Add a map-adapter node (recommended mid-term)

Procedure:
1. Subscribe to online OctoMap message stream (`octomap_msgs/Octomap`).
2. Convert and feed occupancy updates into planner's expected format.
3. Maintain an in-memory occupancy structure for nav6d queries.

Effect:
- True online planning readiness.
- Cleaner architecture boundary between mapper and planner.

Tradeoff:
- Requires nav6d integration point for dynamic map ingestion.

### Strategy 3 — Global-static + local-dynamic hybrid (robust practical path)

Procedure:
- Global plan from slower/static map.
- Local collision monitor (or local planner) checks near-horizon hazards from live point cloud/octomap and triggers replan when blocked.

Effect:
- Strong operational safety without waiting for full planner refactor.

Tradeoff:
- Requires additional local safety logic.

---

## 5) Replanning trigger strategy (design only, not implemented yet)

Below is a starter strategy you can implement later.

## Trigger classes

1. **Map-change trigger**
   - Condition: Occupancy change intersects current trajectory corridor.
   - Action: Replan from current vehicle state to current mission goal.

2. **Progress/failsafe trigger**
   - Condition: No progress toward next waypoint for `T_stuck` seconds.
   - Action: Replan once; if repeated N times -> hover/loiter/RTL policy.

3. **Trajectory-feasibility trigger**
   - Condition: Required curvature/acceleration exceeds configured limits.
   - Action: Replan with relaxed timing or safer corridor.

4. **Localization-health trigger**
   - Condition: TF timeout, jump, or covariance beyond threshold.
   - Action: hold/hover + inhibit aggressive path updates until healthy.

## Suggested trigger parameters (initial)

- `replan_enable`: bool
- `replan_rate_limit_s`: minimum seconds between replans
- `corridor_radius_m`: collision-check radius around current trajectory
- `stuck_distance_m`, `stuck_time_s`
- `max_replan_retries`
- `failsafe_action`: `hover | land | rtl`

## Suggested replan flow

1. Detect trigger.
2. Check rate limiter and planner availability.
3. Snapshot current pose + current mission target.
4. Call planner.
5. If plan valid -> optimize and swap trajectory atomically.
6. If plan invalid -> execute configured failsafe action.

---

## 6) Suggested implementation order (safe rollout)

1. Stabilize online OctoMap + TF first.
2. Validate existing A* + optimizer chain with static map.
3. Add snapshot-based pseudo-online updates.
4. Introduce trigger manager node (monitor only, no action).
5. Enable active replan with rate limiting.
6. Add failsafe policy state machine.

---

## 7) Tuning notes for max velocity / acceleration (minimum-snap stage)

In `nav_6d_optimize_traj`, performance knobs are:
- `max_velocity`
- `max_acceleration`
- `v_ref`
- `a_ref`
- (supportive timing knobs: `time_scale`, `corner_time_gain`)

Practical method:
1. Increase `max_velocity` gradually.
2. Increase `max_acceleration` only if corner tracking is too sluggish.
3. Keep `v_ref <= max_velocity`, `a_ref <= max_acceleration`.
4. If oscillation/overshoot appears, lower `v_ref` first, then `a_ref`.

---

## 8) Minimal verification checklist

- Gazebo depth topic has data (`gz topic -e ...`).
- Bridged ROS point cloud has data.
- OctoMap topic updates continuously.
- `nav6d` receives goals and publishes planner path.
- `nav_6d_optimize_traj` publishes `/trajectory/reference`.
- TF tree is consistent for map and sensor frames.

This document is intended as the strategy baseline for your next implementation step: **online planning + replanning trigger manager**.
