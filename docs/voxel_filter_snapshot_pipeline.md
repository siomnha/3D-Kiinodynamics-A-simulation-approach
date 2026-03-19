# Voxel filter for lighter OctoMap snapshots

This guide adds `voxel_filter_optimized` in front of `octomap_server` so you can rebuild a denser-looking but computationally lighter snapshot map.

## What the node does

- subscribes to a `PointCloud2` topic (default `/fused_cloud`)
- keeps only the newest cloud (`BEST_EFFORT`, depth=1)
- drops stale clouds if they are older than `max_message_age_s`
- downsamples with an Open3D voxel grid
- optionally applies a statistical outlier filter
- republishes the reduced cloud (default `/filtered_cloud`)

## New executable

```bash
ros2 run nav6d_sim voxel_filter_optimized --ros-args \
  -p input_topic:=/depth/points \
  -p output_topic:=/filtered_cloud \
  -p voxel_size:=0.10 \
  -p max_points:=15000 \
  -p drop_old_messages:=true
```

## Recommended snapshot build chain

Terminal A — Gazebo + depth pipeline only, or your existing point cloud source.

Terminal B — voxel filter:

```bash
source /opt/ros/humble/setup.bash
source ~/planner_ws/install/setup.bash
ros2 run nav6d_sim voxel_filter_optimized --ros-args \
  -p input_topic:=/depth/points \
  -p output_topic:=/filtered_cloud \
  -p voxel_size:=0.10 \
  -p max_points:=15000
```

Terminal C — OctoMap server using the filtered cloud:

```bash
source /opt/ros/humble/setup.bash
ros2 run octomap_server octomap_server_node --ros-args \
  -p frame_id:=map \
  -p resolution:=0.20 \
  -p sensor_model/max_range:=10.0 \
  -r cloud_in:=/filtered_cloud
```

Terminal D — save a snapshot after mapping is complete:

```bash
source /opt/ros/humble/setup.bash
ros2 run octomap_server octomap_saver_node --ros-args \
  -p octomap_path:=/home/siomnha/planner_ws/static_map.bt
```

## Practical tuning

- start with `voxel_size:=0.10`
- increase to `0.15` or `0.20` if RViz or CPU still lags
- keep `resolution:=0.20` to `0.30` in `octomap_server`
- reduce `max_range` if distant clutter is not useful

## Verify

```bash
ros2 topic hz /filtered_cloud
ros2 topic echo /octomap_full --once
```

If `/filtered_cloud` is stable and `/octomap_full` builds correctly, save the new snapshot and use that `.bt` file for static planning.
