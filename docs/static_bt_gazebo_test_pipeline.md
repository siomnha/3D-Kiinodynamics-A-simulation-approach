# Static `.bt` Gazebo Test Pipeline (nav6d + optimization + continuous trajectory)

This guide is for your **first test stage**:
- run Gazebo + OctoMap,
- save a static `.bt` map,
- run `nav6d` + `nav_6d_optimize_traj` with continuous trajectory behavior,
- feed waypoints from MAVProxy txt into `mission_manager`.

Target stack: ROS 2 Humble + Gazebo Harmonic.

---

## 1) Goal of this pipeline

You want to validate:
1. static map A* planning works,
2. optimized trajectory follows continuously (not stop-go style behavior),
3. waypoint mission from txt can drive `/waypoints -> /nav6d/goal`.

---

## 2) Nodes and topic flow

```text
[Waypoint txt]
   -> mavproxy_waypoint_loader
   -> /waypoints (nav_msgs/Path)
   -> mission_manager
   -> /nav6d/goal (PoseStamped)
   -> nav6d planner
   -> /nav6d/planner/path (Path)
   -> nav_6d_optimize_traj
   -> /trajectory/reference (Path)
   -> /space_cobot/pose + /trajectory/state
```

For static map test, planner map source is `.bt` file via `octomap_server`.

---

## 3) Workspace and build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4) Build OctoMap first in Gazebo (optional pre-step for creating `.bt`)

If you already have a valid `.bt`, skip to section 5.

Terminal A:
```bash
cd ~/ros2_ws/src/nav6d_sim/pathgazeobo-main
source /opt/ros/humble/setup.bash
bash scripts/run_iris_octomap_pipeline.sh
```

Let the drone/camera scan until map is complete enough.

---

## 5) How static `.bt` snapshot is taken

You asked whether snapshot is a “button press after map built” or criteria-based.

## Option A — Manual snapshot (recommended for first static test)

Think of this as your **button press** step:

```bash
source /opt/ros/humble/setup.bash
<<<<<<< codex/merge-3d-a-with-path-planning-4bx85b
# check exact executable name on your machine
ros2 pkg executables octomap_server

# most Humble installs use octomap_saver_node:
ros2 run octomap_server octomap_saver_node --ros-args \
  -p octomap_path:=/absolute/path/static_map.bt
```

If `octomap_saver_node` is not listed, install package binaries and retry:
```bash
sudo apt update
sudo apt install ros-humble-octomap-server
ros2 pkg executables octomap_server
=======
ros2 run octomap_server octomap_saver -f /absolute/path/static_map.bt
>>>>>>> main
```

When to run:
- after `/octomap_full` looks complete in RViz,
- after you fly/scan the required area.

This gives one stable map file for repeatable testing.

## Option B — Automatic snapshot manager criteria (already implemented)

Use `snapshot_manager` when you want periodic or change-based saves:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav6d_sim snapshot_manager --ros-args \
  -p octomap_topic:=/octomap_binary \
  -p snapshot_period_s:=5.0 \
  -p min_snapshot_interval_s:=2.0 \
  -p save_on_change_only:=true \
  -p mode:=overwrite \
  -p output_dir:=/tmp/octomap_snapshots \
  -p snapshot_prefix:=map_snapshot
```

Current criteria in node behavior:
- receives OctoMap message,
- checks map change checksum,
- saves only if changed (when `save_on_change_only=true`),
- rate-limits save by `min_snapshot_interval_s`,
- emits map reload event on success.

For your static test phase, **manual snapshot is simpler and recommended**.

---

## 6) Run static-map planner test (`.bt`)

Terminal 1 — load static `.bt`:
```bash
source /opt/ros/humble/setup.bash
ros2 run octomap_server octomap_server_node \
  --ros-args -p octomap_path:=/absolute/path/static_map.bt
```

Terminal 2 — nav6d planner:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav6d n6d_planner
```

Terminal 3 — optimizer (where max vel/accel params go):
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav6d_sim nav_6d_optimize_traj --ros-args \
  -p input_topic:=/nav6d/planner/path \
  -p v_ref:=1.2 \
  -p a_ref:=0.8 \
  -p max_velocity:=2.0 \
  -p max_acceleration:=1.5 \
  -p time_scale:=1.15 \
  -p corner_time_gain:=0.35
```

Terminal 4 — mission manager:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav6d_sim mission_manager
```

Terminal 5 — TF publisher (choose ONE authority in your system):
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav6d_sim tf_bridge
```

> If pathgazeobo already publishes dynamic `map -> base_link`, do not run conflicting TF publisher.

---

## 7) Where to tune max velocity / acceleration

Tune in **Terminal 3 command** (`nav_6d_optimize_traj`) by these params:
- `max_velocity` (hard speed cap in timing model)
- `max_acceleration` (hard accel cap)
- `v_ref` / `a_ref` (nominal cruise/accel targets)

Fast profile example:
```bash
ros2 run nav6d_sim nav_6d_optimize_traj --ros-args \
  -p v_ref:=2.0 -p a_ref:=1.2 \
  -p max_velocity:=3.0 -p max_acceleration:=2.0
```

Conservative profile example:
```bash
ros2 run nav6d_sim nav_6d_optimize_traj --ros-args \
  -p v_ref:=1.0 -p a_ref:=0.6 \
  -p max_velocity:=1.5 -p max_acceleration:=1.0
```

For continuity/stability:
- increase speed gradually,
- if overshoot appears, reduce `v_ref` first, then `a_ref`,
- increase `time_scale` slightly for smoother, safer motion.

---

## 8) MAVProxy waypoint txt -> mission_manager interaction

Terminal 6 — waypoint loader from txt:

### QGC WPL / MAVProxy mission format
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run nav6d_sim mavproxy_waypoint_loader --ros-args \
  -p waypoint_file:=/absolute/path/waypoints.txt \
  -p frame_id:=map \
  -p origin_lat:=22.304217 \
  -p origin_lon:=114.179840 \
  -p origin_alt:=0.0
```

### Plain local XYZ txt
```bash
ros2 run nav6d_sim mavproxy_waypoint_loader --ros-args \
  -p waypoint_file:=/absolute/path/waypoints_xyz.txt \
  -p frame_id:=map
```

Interaction behavior:
1. loader publishes `/waypoints` as `nav_msgs/Path`.
2. `mission_manager` subscribes `/waypoints`, stores list, starts from index 0.
3. `mission_manager` publishes one `/nav6d/goal` at a time.
4. on waypoint-reached threshold, it sends next goal.

So in this phase, mission dispatch is still sequential; continuity mainly comes from optimizer smoothing between planned path points.

---

## 9) Quick validation commands

```bash
ros2 topic echo /waypoints --once
ros2 topic echo /nav6d/goal --once
ros2 topic echo /nav6d/planner/path --once
ros2 topic echo /planning/pruned_path --once
ros2 topic echo /trajectory/reference --once
ros2 topic hz /space_cobot/pose
```

---

## 10) Recommended first-test recipe

1. Build map online in Gazebo.
2. Manually save one final static `.bt` (“button press” step).
3. Restart test using static `.bt` only.
4. Tune `max_velocity`/`max_acceleration` in optimizer command.
5. Feed txt mission through `mavproxy_waypoint_loader`.

After this passes, move to snapshot+replan monitor stage.
