# Real-world run commands (Octomap + Vicon + nav6d + MAVROS + ArduPilot)

This runbook starts all required parts for indoor testing.
It includes the full flying pipeline from loading your Octomap snapshot (`shit1.bt`)
to streaming setpoints through MAVROS.

## 0) Build and source workspace
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 1) Start Octomap server with your site snapshot (Terminal A)
`shit1.bt` is the map snapshot of your real flying environment.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run octomap_server octomap_server_node --ros-args \
  -p octomap_path:=/workspace/3D-Kiinodynamics-A-simulation-approach/shit1.bt
```

If your file is in another location, replace the `octomap_path`.

## 2) Start Vicon bridge (Terminal B)
Use your real Vicon host and target subject/segment names.
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run vicon_bridge vicon_bridge --ros-args \
  -p host_name:="192.168.1.10:801" \
  -p publish_specific_segment:=true \
  -p target_subject_name:="px4" \
  -p target_segment_name:="px4" \
  -p world_frame_id:="vicon_world" \
  -p tf_namespace:="vicon"
```

## 3) Start MAVROS connected to ArduPilot (Terminal C)
Example for UDP SITL/companion link (change FCU URL for hardware).
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

## 4) Launch nav6d real-world stack (Terminal D)
This launches:
- static map-vicon alignment tf
- tf_to_pose_bridge (Vicon TF -> /space_cobot/pose)
- mission_manager
- n6d_planner
- nav_6d_optimize_traj

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch nav6d_sim real_world_vicon_stack.launch.py
```

## 5) Start setpoint bridge in `mode=state` (Terminal E, recommended first)
`mode=state` forwards `/trajectory/state` directly to MAVROS local position setpoints.
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim ardupilot_mavros_setpoint_bridge --ros-args \
  -p mode:=state \
  -p state_topic:=/trajectory/state \
  -p vehicle_pose_topic:=/space_cobot/pose \
  -p setpoint_topic:=/mavros/setpoint_position/local
```

## 6) Alternative: setpoint bridge in `mode=path` (Terminal E alternative)
`mode=path` follows `/trajectory/reference` with lookahead and arrival-radius logic.
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim ardupilot_mavros_setpoint_bridge --ros-args \
  -p mode:=path \
  -p path_topic:=/trajectory/reference \
  -p vehicle_pose_topic:=/space_cobot/pose \
  -p arrival_radius_m:=0.4 \
  -p lookahead_index:=3 \
  -p setpoint_topic:=/mavros/setpoint_position/local
```

## 7) End-to-end flying pipeline (what makes the drone fly)
1. Push mission waypoints via MAVROS/MAVProxy so mission_manager receives `/mavros/mission/waypoints`.
2. Verify mission_manager is publishing `/nav6d/goal`.
3. nav6d planner computes path from current pose + Octomap (`shit1.bt` loaded in Terminal A).
4. `nav_6d_optimize_traj` turns that path into `/trajectory/reference` and `/trajectory/state`.
5. Start setpoint stream first (Terminal E already running).
6. Arm and switch to external setpoint-accepting flight mode (GUIDED/OFFBOARD depending on FCU config).
7. ArduPilot follows `/mavros/setpoint_position/local`.

## 8) Quick checks
```bash
ros2 topic echo /octomap_full --once
ros2 topic hz /space_cobot/pose
ros2 topic echo /nav6d/goal --once
ros2 topic echo /trajectory/state --once
ros2 topic echo /trajectory/reference --once
ros2 topic echo /mavros/setpoint_position/local --once
```

## Notes
- `nav_6d_optimize_traj` now publishes a reference pose on `/trajectory/reference_pose` (not `/space_cobot/pose`) to avoid conflicting with Vicon truth pose.
- Keep `/space_cobot/pose` as the real vehicle pose source from Vicon.


## Optional: continuous mission trajectory from all waypoints (no stop-and-go)
If you want one continuous trajectory over all waypoints, enable continuous mode in `mission_manager` and feed that path directly to the optimizer:

```bash
# mission_manager publishes full mission path once per mission update
ros2 run nav6d_sim mission_manager --ros-args   -p continuous_mission_mode:=true   -p continuous_path_topic:=/mission_manager/continuous_path

# optimizer consumes the full mission path directly
ros2 run nav6d_sim nav_6d_optimize_traj --ros-args   -p input_topic:=/mission_manager/continuous_path
```

This bypasses per-waypoint goal switching and is intended for pre-validated waypoint routes.

