# 3D-Kinodynamics-A-simulation-approach

This project provide a basic tutorial applying 3D kinodynamics A* algorithm developed by https://github.com/ItsNotSoftware/nav6d.


After building the packages, it allows you to: 

1.Run 3D kinodynamic A* algorithm in Rviz on a octomap

2.Pre-defined multiple waypoints your UAV want to travel easily

3.Visualize the flight path 



The packages are still in development and aim to replace original PD controller to ardupilot PID controller and add CraneAero model into the simulation. Also the algorithm constraints should be tuned on your specific mission

<img width="2486" height="1411" alt="Screenshot from 2026-02-19 17-34-36" src="https://github.com/user-attachments/assets/8e807ef0-258c-40f2-87c9-3f51d3a1ccd5" />


# nav6d_sim
On top of your Rviz and nav6d packages, there are still several to build

1. Go to your workspace
```
cd ~/ros2_ws/src
```

3. Create a python package
```
ros2 pkg create nav6d_sim --build-type ament_python --dependencies rclpy geometry_msgs nav_msgs tf2_ros
```

5. Go to node folder
```
cd ~/ros2_ws/src/nav6d_sim/nav6d_sim
```

7. Add the file in to the node folder

8. Build the package and source it
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav6d_sim --symlink-install
source install/setup.bash
```

10. Verify excutable and all done
```
ros2 pkg executables nav6d_sim
```



# 3D-waypoint-rviz tool plugin
In order to manage your waypoint easily, you also have to install the waypoint plugin

Please follow the reference:https://github.com/castacks/3d-waypoint-rviz2-plugin



# How to use

## Quick start (integrated pipeline)
If you only want to test the current integrated flow, the minimum set is:
1. RViz waypoint plugin
2. octomap server
3. `nav6d` planner
4. `nav6d_sim nav_6d_optimize_traj`
5. `nav6d_sim mission_manager`
6. `nav6d_sim tf_bridge`

Expected topic flow:
`/waypoints` -> `/nav6d/goal` -> `/nav6d/planner/path` -> `/planning/pruned_path` -> `/trajectory/reference` -> (`/trajectory/state`, `/space_cobot/pose`)

First, run RViz with the waypoint plugin (Terminal 1)
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch waypoint_rviz2_plugin rviz2.launch.py
```


Run your octomap (Terminal 2), if you don't have one, find on any open-source dataset for .bt file that fit your mission
```
source /opt/ros/humble/setup.bash

ros2 run octomap_server octomap_server_node \
  --ros-args -p octomap_path:=/path of your map
```


Run your UAV model (Terminal 3), if you don't have one, install the iris model provided
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run robot_state_publisher robot_state_publisher \
~/iris_ws/src/iris_description/urdf/iris.urdf
```

Open nav6d planner (Terminal 4)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d n6d_planner
```

### Configure A* obstacle distance margin (recommended)

Set obstacle margin in the **planner / occupancy inflation layer**, not in trajectory post-processing.

1. Inspect available planner params:
```
ros2 param list /n6d_planner
```

2. Check candidate margin-related params (names depend on nav6d version):
```
ros2 param get /n6d_planner robot_radius
ros2 param get /n6d_planner inflated_radius
ros2 param get /n6d_planner safety_margin
```

3. Set margin (example values):
```
ros2 param set /n6d_planner robot_radius 2.0
ros2 param set /n6d_planner inflated_radius 1.2
```

If your planner uses different parameter names, use `ros2 param list /n6d_planner` and set the equivalent obstacle inflation/safety fields.

If your current nav6d build does not expose `inflated_radius`, see `docs/nav6d_planner_margin_patch.md` for the required core-planner patch pattern and the example params file at `config/nav6d_planner_params.example.yaml`.

4. Double validation (recommended before flight tests):
```
ros2 param get /n6d_planner robot_radius
ros2 param get /n6d_planner inflated_radius
```
Expected current baseline in this repo guide:
- `robot_radius = 2.0`
- `inflated_radius = 1.2`

**Exact edit points (what to change):**
- **A* margin / inflation (required):** change parameters on node `/n6d_planner` (typically in nav6d launch file or via `ros2 param set`).
- **Primary knobs:** `robot_radius`, `inflated_radius`, and planner-specific `safety_margin` equivalent.
- **Current limit in this repo:** `nav_6d_optimize_traj` does not provide obstacle-distance guarantee parameters; it only smooths/samples trajectory geometry.

Open nav_6d optimize_traj integrated node (Terminal 5)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim nav_6d_optimize_traj --ros-args \
  -p v_ref:=1.2 -p a_ref:=0.8 \
  -p max_velocity:=2.0 -p max_acceleration:=1.5 \
  -p time_scale:=1.15 -p corner_time_gain:=0.35
```

This node contains the full chain in one place: A* input (`/nav6d/planner/path`) -> pruning -> optimize_traj/minimum-snap stage -> `/trajectory/reference` + `/trajectory/state` + `/space_cobot/pose`.

> Safety note: obstacle clearance margin should be configured in the upstream A* planner/map inflation stage. `nav_6d_optimize_traj` is a post-processor and does not do obstacle-distance collision checking by itself.

Legacy modular nodes (`path_pruner`, `trajectory_sampler`) are kept for compatibility; `trajectory_adapter` has been removed in favor of integrated `nav_6d_optimize_traj`.

Open Trajectory sampler (Terminal 7)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim trajectory_sampler
```

Set your tf config (Terminal 7), note that it define your UAV initial location. If in your simulation, your model jumps between initial location and the trajectory, stop this terminal before run again
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  -5 0.5 1.5 0 0 0 map base_link
```

Open tf bridge (Terminal 8)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim tf_bridge
```

(Optional legacy) Open Path follower (replays raw planner path without minimum-snap):
```
ros2 run nav6d_sim path_follower
```



Quick topic checks (optional)
```
ros2 topic echo /planning/pruned_path --once
ros2 topic echo /trajectory/reference --once
ros2 topic hz /space_cobot/pose
```

Mode generator (optional, for mode-by-mode RViz replication tests)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim trajectory_mode_generator --ros-args -p mode:=circle
```

Supported modes in this repo: `hover`, `line`, `circle`, `figure8`, `helix`.
Use this when you want to validate follower/controller behavior without depending on planner output.

# Setting
On Rviz, add new panel: Waypoint tools and set your waypoint 

On Rviz, set visualization:

Add a path display on /nav6d/planner/path

Add a MarkerArray display on /nav6d/planner/path_marker

Fixed frame: map

Add /Pointcloud2 or /octomap_full depends on your setting, on it sublevel topic, set the durability to transient local

Add a Pose display on /nav6d/goal

Add TF 

Add Robotmodel display on /spacecobot/pose or any source of your model


# Run
After you have done all the setting and all the terminals are running correctly
On the waypoint plugin, publish waypoints. It will then forward to mission manager and generate trajectory based on A*

troubleshooting with echo the data whatever you have missed. For example: ros2... echo.. /nav6d/planner/pose

# A* + Minimum Snap architecture (RViz-ready, controller-decoupled)
If your goal is **nav6d A* planning now**, and later plug in your own path follower + ArduPilot PID, use this architecture.

## 1) End-to-end pipeline
1. **Mission Manager** receives user waypoints (`/waypoints`) and sends sequential goals to nav6d (`/nav6d/goal`).
2. **nav6d planner** computes collision-free discrete path (`/nav6d/planner/path`).
3. **Pruning Stage (new, recommended)** applies LOS pruning or polynomial pruning to remove noisy local waypoints while preserving obstacle safety.
4. **Integrated optimize_traj stage** in `nav_6d_optimize_traj` converts pruned path into minimum-snap-style reference segments.
5. **Trajectory Sampler behavior** (inside integrated node) publishes by time:
   - RViz-friendly pose for visualization
   - reference states for future controllers
6. **Path Follower (future plugin)** consumes reference states and outputs control setpoints.
7. **ArduPilot Bridge (future plugin)** maps setpoints to MAVROS/MAVLink interfaces.

This keeps nav6d as a pure planner and makes downstream controller stacks replaceable.

## 2) Node boundary (recommended)
### Planning layer (keep now)
- `mission_manager` (already in this repo)
- `nav6d` planner (external package)

### Trajectory layer (add now for RViz full-chain test)
- `path_pruner`
  - input: `/nav6d/planner/path` (`nav_msgs/Path`)
  - output: `/planning/pruned_path` (`nav_msgs/Path`)
  - role: LOS pruning and/or polynomial pruning to reduce A* local waypoint noise
- `trajectory_sampler`
  - input: trajectory internal representation
  - output:
    - `/space_cobot/pose` (`geometry_msgs/PoseStamped`) for TF/RViz model movement
    - `/trajectory/state` (reserved; pose/vel/acc/yaw reference)

### Control layer (reserve for future)
- `path_follower_plugin`
  - input: `/trajectory/state`
  - output: `/control/setpoint`
- `ardupilot_bridge`
  - input: `/control/setpoint`
  - output: MAVROS/MAVLink topics/services

## 3) Interface contract (stable topics)
- Keep these as long-term stable interfaces:
  - `/nav6d/goal`
  - `/nav6d/planner/path`
  - `/planning/pruned_path`
  - `/trajectory/state` (new stable contract from trajectory to controller)
  - `/control/setpoint` (new stable contract from follower to flight stack)

By freezing these contracts, you can swap minimum-snap implementation or controller without touching nav6d.

## 4) Minimum-snap integration details
### 4.0 Where to tune optimize_traj design parameters
Tune these directly in `nav_6d_optimize_traj` ROS params (CLI `--ros-args -p ...` or launch file):
- `optimize_traj` is now the default integrated mode in `nav_6d_optimize_traj`.
- `v_ref`, `a_ref`: nominal timing targets.
- `max_velocity`, `max_acceleration`: hard caps on effective timing model.
- `time_scale`: global slowdown/speedup factor after nominal allocation.
- `corner_time_gain`: extra time added around sharp turns from pruned A* waypoints.
- `min_segment_time`, `sample_dt`: lower bound on segment duration and trajectory sampling density.

Recommended tuning order after nav6d A* + pruning:
1. Set `max_velocity` and `max_acceleration` from vehicle/controller limits.
2. Fit `v_ref` and `a_ref` slightly below limits for tracking margin.
3. Increase `time_scale` if overshoot appears in RViz/follower.
4. Increase `corner_time_gain` if turns are still too aggressive.
5. Reduce `sample_dt` for smoother visualization/feeding downstream controllers.

### 4.1 Pruning before minimum-snap (strongly recommended)
- **LOS pruning:** keep key turning points and remove intermediate collinear/visible points.
- **Polynomial pruning:** fit low-order curve locally and drop high-frequency zig-zag waypoints from A*.
- Keep an obstacle-safe check after pruning to avoid over-aggressive waypoint removal.

### 4.2 Input pre-processing after pruning
- Remove remaining duplicate/near-duplicate points.
- Optionally downsample long straight sections to reduce polynomial condition number.

### 4.3 Time allocation (critical)
For each segment i:
- `T_i = max(dist_i / v_ref, sqrt(dist_i / a_ref))`
- Then clamp with mission limits (`v_max`, `a_max`, optional `j_max`).

### 4.4 Boundary conditions
- Start/end velocity and acceleration default to zero for RViz simulation.
- Keep yaw simple at first:
  - either fixed yaw
  - or yaw from velocity direction

### 4.5 Safety after smoothing
Because polynomial curves may leave the A* free corridor:
- Sample smoothed trajectory at fixed `dt` (e.g., 0.05 s)
- Collision-check sampled points against map
- If failed: inflate timing or fall back to original A* polyline for that segment

Current repo status:
- Obstacle-safe margin guarantee is provided by A* occupancy/inflation configuration.
- `nav_6d_optimize_traj` currently performs geometric smoothing/sampling only.
- Replanning trigger and trajectory handover logic are intentionally reserved for future localization + online planning integration.



### 4.6 How to keep optimized trajectory inside safety margin
For future online deployment (simulation + real robot), recommended order:

1. **A* hard safety guarantee**: use inflated occupancy (robot radius + margin) so raw A* path is already obstacle-safe.
2. **Corridor-constrained optimization**: optimize inside a convex safe corridor built from inflated map/free-space decomposition.
3. **Dense post-check**: sample optimized trajectory densely and perform map collision checking with same inflated map.
   - In this repo, add this check at `nav6d_sim/nav_6d_optimize_traj.py` inside `path_callback`, right after `samples = self.optimize_traj(...)` and before publishing `/trajectory/reference`.
4. **Fallback policy**: if any sample violates margin, reject this segment and fall back to A* polyline or re-run optimization with larger timing / tighter constraints.
5. **Replan hook reserved**: keep trigger/handover interface reserved for future localization-driven online replanning (not implemented yet in this repo).

## 5) RViz full-structure test plan (what you want now)
Run the complete chain (planning + smoothing + sampling + visualization), while still not committing to a real controller:

1. waypoint plugin publishes `/waypoints`
2. mission manager sends `/nav6d/goal`
3. nav6d publishes `/nav6d/planner/path`
4. path pruner publishes `/planning/pruned_path`
5. integrated optimize_traj builds minimum-snap-style reference
6. integrated publisher outputs `/space_cobot/pose`
7. tf bridge broadcasts `map -> base_link`
8. RViz shows:
   - raw A* path (`/nav6d/planner/path`)
   - pruned path (`/planning/pruned_path`)
   - smoothed path (`/trajectory/reference`)
   - RobotModel + TF motion

This gives you an RViz-verifiable end-to-end architecture before adding follower and ArduPilot.

## 6) Suggested message reservation for future controller/ArduPilot
For `/trajectory/state` you can reserve fields equivalent to:
- position `(x,y,z)`
- velocity `(vx,vy,vz)`
- acceleration `(ax,ay,az)`
- yaw, yaw_rate
- timestamp

For `/control/setpoint` reserve:
- attitude target (roll/pitch/yaw) + thrust, or
- velocity target + yaw_rate (depending on flight stack mode)

## 7) Recommended rollout phases
- **Phase A (now):** nav6d + minimum-snap + RViz visualization chain
- **Phase B:** replace dummy sampler with real path follower
- **Phase C:** add ArduPilot bridge and tune PID/flight modes
- **Phase D:** add replanning trigger + trajectory handover logic

## Troubleshooting: no executable found
If `ros2 run nav6d_sim nav_6d_optimize_traj` reports **no executable found**, verify package registration and entry points:

```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nav6d_sim --symlink-install
source install/setup.bash
ros2 pkg executables nav6d_sim
```

Expected output should include: `nav_6d_optimize_traj` (and other nodes like `mission_manager`, `tf_bridge`).

If still missing, clean and rebuild:

```
cd ~/ros2_ws
rm -rf build/nav6d_sim install/nav6d_sim log
colcon build --packages-select nav6d_sim --symlink-install
source install/setup.bash
```
