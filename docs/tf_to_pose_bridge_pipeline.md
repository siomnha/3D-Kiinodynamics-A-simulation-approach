# TF to `/space_cobot/pose` bridge pipeline (Gazebo TF source mode)

This guide is for your current setup where:
- Gazebo dynamic pose is bridged to ROS `/tf`, and
- you need `/space_cobot/pose` for mission/planner integration.

A new node is provided:
- `nav6d_sim tf_to_pose_bridge`

It converts TF transform `map -> base_link` into `PoseStamped` on `/space_cobot/pose`.

Publishing period default is **0.05 s (20 Hz)**, matching `nav_6d_optimize_traj` state publish timer.

---

## 1) Why this bridge is needed

`mission_manager` and other nodes consume `/space_cobot/pose`, but pathgazeobo may only provide `/tf` from Gazebo dynamic pose.

So this node fills the gap:

```text
Gazebo dynamic pose -> /tf (map->base_link) -> tf_to_pose_bridge -> /space_cobot/pose
```

---

## 2) Start order

## Terminal A: Gazebo + dynamic pose to /tf bridge

Use your existing pathgazeobo command (example):

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/goaero_mission3/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V
```

## Terminal B: TF -> Pose bridge (new)

```bash
source /opt/ros/humble/setup.bash
source ~/planner_ws/install/setup.bash
ros2 run nav6d_sim tf_to_pose_bridge --ros-args \
  -p parent_frame:=map \
  -p child_frame:=base_link \
  -p pose_topic:=/space_cobot/pose \
  -p publish_period_s:=0.05
```

## Terminal C+: planner chain

- `octomap_server` (static or online)
- `nav6d n6d_planner`
- `nav6d_sim nav_6d_optimize_traj`
- `nav6d_sim mission_manager`
- `nav6d_sim mavproxy_waypoint_loader` (optional for mission txt)

---

## 3) Verify bridge is live

```bash
ros2 topic echo /tf --once
ros2 topic hz /space_cobot/pose
ros2 topic echo /space_cobot/pose --once
```

Expected:
- `/tf` has map->base_link transforms
- `/space_cobot/pose` near 20 Hz

---

## 4) Notes

1. Keep only one TF authority for `map -> base_link`.
2. If transform names differ in your world/model, change `parent_frame`/`child_frame`.
3. If no TF at startup, bridge waits and warns periodically until TF appears.

---

## 5) Full mission example with your mission file

```bash
source /opt/ros/humble/setup.bash
source ~/planner_ws/install/setup.bash

ros2 run nav6d_sim mavproxy_waypoint_loader --ros-args \
  -p waypoint_file:=/home/siomnha/ardupilot/mission.txt \
  -p frame_id:=map \
  -p origin_lat:=-35.3632621 \
  -p origin_lon:=149.1652374 \
  -p origin_alt:=10.0 \
  -p publish_period_s:=3600.0
```

This avoids repeated mission reset while still delivering waypoints to `mission_manager`.
