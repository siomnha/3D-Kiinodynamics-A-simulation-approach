# Real-world run commands (Vicon + nav6d + MAVROS + ArduPilot)

This runbook starts all required parts for indoor testing.

## 0) Build and source workspace
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 1) Start Vicon bridge (Terminal A)
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

## 2) Start MAVROS connected to ArduPilot (Terminal B)
Example for UDP SITL/companion link (change FCU URL for hardware).
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

## 3) Launch nav6d real-world stack (Terminal C)
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

## 4) Start setpoint bridge in `mode=state` (Terminal D, recommended first)
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

## 5) Alternative: setpoint bridge in `mode=path` (Terminal D alternative)
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

## 6) Mission/arming workflow
1. Push mission waypoints via MAVROS/MAVProxy so mission_manager receives `/mavros/mission/waypoints`.
2. Verify mission_manager is publishing `/nav6d/goal`.
3. Start setpoint stream first (Terminal D already running).
4. Arm and switch to external setpoint-accepting flight mode (GUIDED/OFFBOARD depending on FCU config).

## 7) Quick checks
```bash
ros2 topic hz /space_cobot/pose
ros2 topic echo /nav6d/goal --once
ros2 topic echo /trajectory/state --once
ros2 topic echo /trajectory/reference --once
ros2 topic echo /mavros/setpoint_position/local --once
```

## Notes
- `nav_6d_optimize_traj` now publishes a reference pose on `/trajectory/reference_pose` (not `/space_cobot/pose`) to avoid conflicting with Vicon truth pose.
- Keep `/space_cobot/pose` as the real vehicle pose source from Vicon.
