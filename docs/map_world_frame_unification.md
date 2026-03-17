# Map-World frame unification for Gazebo + nav6d pipeline

Yes — in your current architecture, the recommended clean structure is:

- keep Gazebo native frame as `world` (from Gazebo pose/TF bridge),
- define planning/global frame as `map`,
- add one static TF bridge `map -> world` so the whole planning pipeline can stay in `map`.

This avoids changing mission/planner/optimizer interfaces that already assume `map`.

---

## 1) New node: `map_world_tf_bridge`

A new node is provided:

- executable: `nav6d_sim map_world_tf_bridge`
- role: publish static transform `map -> world`
- default transform: identity (0,0,0,0,0,0)

### Run command (default identity)

```bash
source /opt/ros/humble/setup.bash
source ~/planner_ws/install/setup.bash

ros2 run nav6d_sim map_world_tf_bridge --ros-args \
  -p parent_frame:=map \
  -p child_frame:=world \
  -p x:=0.0 -p y:=0.0 -p z:=0.0 \
  -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0
```

---

## 2) How frame transform impacts each bridge

## A) `ros_gz_bridge` (sensor/image/pointcloud bridge)

Impact:
- It bridges message transport and message types.
- It does **not** automatically fix your frame-tree design conflicts.

Mitigation:
- Keep sensor messages and octomap frame config consistent with your chosen global frame (`map`).
- Ensure TF chain resolves from octomap frame to robot base and sensors.

## B) `gz_pose_tf` / dynamic pose -> `/tf`

Impact:
- This usually publishes `world -> base_link`.
- Your planner stack often expects `map -> base_link`.

Mitigation:
- Add exactly one `map -> world` static transform (this node),
- then `map -> base_link` becomes available through TF composition.

---

## 3) Conflict risks and how to avoid

1. **Duplicate dynamic TF authorities**
   - Do not publish `map -> base_link` from multiple nodes simultaneously.

2. **Mismatched global frame**
   - If octomap uses `map` but pose only exists in `world` without bridge, planner may fail current-pose lookup.

3. **Wrong child frame names**
   - If Gazebo publishes e.g. `iris/base_link` or different model link name, use that as `child_frame` in `tf_to_pose_bridge`.

---

## 4) Recommended full chain (clean)

1. Gazebo world and depth pipeline.
2. dynamic pose -> `/tf` (`world -> base_link`).
3. run `map_world_tf_bridge` (static `map -> world`).
4. run `tf_to_pose_bridge` (`map -> base_link` lookup) -> `/space_cobot/pose`.
5. run mission_manager + nav6d + optimize_traj.

---

## 5) Verification

```bash
ros2 topic echo /tf --once
ros2 run tf2_tools view_frames
ros2 topic hz /space_cobot/pose
ros2 topic echo /space_cobot/pose --once
```

Expected:
- TF tree includes `map -> world -> base_link`.
- `/space_cobot/pose` is live (20 Hz if `publish_period_s:=0.05`).
