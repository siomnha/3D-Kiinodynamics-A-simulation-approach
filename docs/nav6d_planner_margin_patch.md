# nav6d planner margin patch guide

This repository does **not** contain the core `nav6d` C++ planner source for `n6d_planner`, so the planner itself cannot be edited directly from this checkout. Use this guide to patch the `nav6d` package in your planner workspace.

## Target values

Use these planner values for your current vehicle assumptions:

- `robot_radius = 2.0`
- `inflated_radius = 1.2`
- `max_expansions = 200000`

Recommended YAML example:

```yaml
/n6d_planner:
  ros__parameters:
    robot_radius: 2.0
    inflated_radius: 1.2
    max_expansions: 200000
```

An example file is included in this repo at `config/nav6d_planner_params.example.yaml`.

## What each parameter does

- `robot_radius`: the physical collision radius of the drone used by the planner.
- `inflated_radius`: an additional safety buffer around occupied cells beyond the robot radius.
- `max_expansions`: the A* search budget; larger values allow harder detours at higher computation cost.

A useful mental model is:

```text
effective_clearance_radius = robot_radius + inflated_radius
```

## How to add `inflated_radius` to the core algorithm

If your current `n6d_planner` only exposes `robot_radius`, add `inflated_radius` as a separate ROS parameter in the planner node and use the sum of both when checking occupancy clearance.

### 1. Declare and read the parameter in the planner node

In the `n6d_planner` constructor (often `node.cpp`, `planner_node.cpp`, or similar), add:

```cpp
this->declare_parameter<double>("robot_radius", 2.0);
this->declare_parameter<double>("inflated_radius", 1.2);
this->declare_parameter<int>("max_expansions", 200000);

robot_radius_ = this->get_parameter("robot_radius").as_double();
inflated_radius_ = this->get_parameter("inflated_radius").as_double();
max_expansions_ = this->get_parameter("max_expansions").as_int();
```

Add a new member variable in the planner class:

```cpp
double robot_radius_{2.0};
double inflated_radius_{1.2};
int max_expansions_{200000};
```

### 2. Use both radii in collision checking

Wherever the planner currently checks whether a point, node, or edge is too close to occupied cells, replace use of only `robot_radius_` with the combined margin:

```cpp
const double clearance_radius = robot_radius_ + inflated_radius_;
```

Then use `clearance_radius` for occupancy / distance checks.

Typical places to patch:

- node validity check
- edge interpolation check
- obstacle-neighborhood search around a sampled state
- occupancy inflation routine, if the planner pre-inflates the map

### 3. Example collision-check pattern

If the current code does something like:

```cpp
bool Planner::isStateValid(const Eigen::Vector3d & p) const {
  return map_->isFreeSphere(p, robot_radius_);
}
```

change it to:

```cpp
bool Planner::isStateValid(const Eigen::Vector3d & p) const {
  const double clearance_radius = robot_radius_ + inflated_radius_;
  return map_->isFreeSphere(p, clearance_radius);
}
```

If the planner instead iterates neighbor voxels, use `clearance_radius` as the search radius for occupied cells.

### 4. Make `max_expansions` actually drive A*

If your planner already prints messages like `A* expansion limit reached (60000)`, find the loop that increments the expansion counter and compare against the configurable member:

```cpp
if (expansions >= max_expansions_) {
  RCLCPP_WARN(this->get_logger(),
              "A* expansion limit reached (%d).",
              max_expansions_);
  return false;
}
```

If the log still prints a hard-coded number after `ros2 param set`, the planner is likely caching the startup value only. In that case, set the parameter at launch time or add a parameter callback.

### 5. Optional: support live parameter updates

If you want runtime updates from `ros2 param set`, register a parameter callback and refresh the cached class members when `robot_radius`, `inflated_radius`, or `max_expansions` changes.

## Launching with persistent values

Once the core planner supports `inflated_radius`, launch with:

```bash
ros2 run nav6d n6d_planner --ros-args \
  -p robot_radius:=2.0 \
  -p inflated_radius:=1.2 \
  -p max_expansions:=200000
```

Or use a params file:

```bash
ros2 run nav6d n6d_planner --ros-args \
  --params-file /absolute/path/nav6d_planner_params.example.yaml
```

## How this relates to this repository

The trajectory node in this repository is only a post-processor. Obstacle clearance must be guaranteed upstream by the A* planner / occupancy inflation layer, not by `nav_6d_optimize_traj`.
