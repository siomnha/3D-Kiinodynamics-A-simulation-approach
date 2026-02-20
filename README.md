# 3D-Kinodynamics-A-simulation-approach

This project provide a basic tutorial applying 3D kinodynamics A* algorithm developed by https://github.com/ItsNotSoftware/nav6d.


After building the packages, it allows you to: 

1.Run a basic A* algorithm in Rviz on a octomap

2.Pre-defined multiple waypoints your UAV want to travel

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


Open Path follower (Terminal 5)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim path_follower
```



Open Mission manager (Terminal 6)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run nav6d_sim mission_manager
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
