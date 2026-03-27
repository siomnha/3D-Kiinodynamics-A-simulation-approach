
## Real-world Vicon + MAVROS mission flow

Use this package in indoor Vicon tests with:
- `ros2-vicon-bridge-main` publishing TF for your tracked drone segment
- `nav6d_sim tf_to_pose_bridge` converting TF into `/space_cobot/pose`
- `nav6d_sim mission_manager` accepting either:
  - `nav_msgs/Path` on `/waypoints` (RViz / mavproxy_waypoint_loader)
  - `mavros_msgs/WaypointList` on `/mavros/mission/waypoints`

Launch the real stack:

```bash
ros2 launch nav6d_sim real_world_vicon_stack.launch.py
```

Before flight, set:
- `tf_to_pose_bridge.child_frame` to your actual Vicon segment frame (for example `vicon/iris/iris`)
- `mission_manager.origin_lat/lon/alt` when MAVROS missions are global (lat/lon/alt)
- optional `map_world_tf_bridge` static offset if your map frame differs from Vicon world origin.

> Update: `nav_6d_optimize_traj` reference pose output is `/trajectory/reference_pose`.
> For real-world tests, keep `/space_cobot/pose` fed by Vicon (`tf_to_pose_bridge`).
> The optimizer now resumes from the nearest point on a newly generated trajectory
> (instead of restarting from index 0), which reduces visible backward jumps.

See `nav6d_node/REAL_WORLD_RUN_COMMANDS.md` for complete commands, including
`ardupilot_mavros_setpoint_bridge` in `mode=state` and `mode=path`.
That runbook also includes the full real-flight pipeline from loading Octomap
snapshot `shit1.bt` to MAVROS setpoint flight.
