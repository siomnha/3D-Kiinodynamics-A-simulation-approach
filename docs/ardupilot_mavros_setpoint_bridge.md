# ArduPilot MAVROS setpoint bridge (from nav6d trajectory)

## Is this a path-following algorithm + GNC control law?

Short answer: **No**.

This node is an **interface bridge** that forwards trajectory references into MAVROS setpoint topics.

- Path following / GNC control law = computes control commands from state error dynamics.
- This bridge = publishes target pose setpoints at fixed rate.

So this is the integration step before (or alongside) advanced GNC tuning.

---

## New node

- executable: `nav6d_sim ardupilot_mavros_setpoint_bridge`
- output topic default: `/mavros/setpoint_position/local`
- publish rate default: 20 Hz (`publish_period_s=0.05`)

Supports two modes:
1. `mode:=state` (default)
   - consumes `/trajectory/state`
   - forwards latest state pose as MAVROS setpoint
2. `mode:=path`
   - consumes `/trajectory/reference`
   - follows path index with lookahead and arrival radius

---

## Run command (recommended start)

```bash
source /opt/ros/humble/setup.bash
source ~/planner_ws/install/setup.bash

ros2 run nav6d_sim ardupilot_mavros_setpoint_bridge --ros-args \
  -p mode:=state \
  -p state_topic:=/trajectory/state \
  -p setpoint_topic:=/mavros/setpoint_position/local \
  -p publish_period_s:=0.05
```

---

## Optional path mode

```bash
ros2 run nav6d_sim ardupilot_mavros_setpoint_bridge --ros-args \
  -p mode:=path \
  -p path_topic:=/trajectory/reference \
  -p vehicle_pose_topic:=/space_cobot/pose \
  -p lookahead_index:=3 \
  -p arrival_radius_m:=0.4 \
  -p publish_period_s:=0.05
```

---

## MAVROS-side quick checks

```bash
ros2 topic echo /mavros/setpoint_position/local --once
ros2 topic hz /mavros/setpoint_position/local
```

If these are active and ArduPilot is in a setpoint-accepting mode, vehicle motion control can follow.

---

## Notes

1. Ensure frame consistency (use `map` pipeline consistently).
2. Do not run competing setpoint publishers to the same MAVROS topic.
3. This bridge does not replace controller tuning; it provides trajectory-to-flight-stack integration.
