from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'cd ~/planner_ws/pathgazeobo-main && '
            'gz sim -v4 -r goaero_mission3_v1.sdf'
        ],
        output='screen'
    )

    # ArduPilot SITL / MAVProxy
    mavproxy = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            'cd ~/ardupilot && '
            'source /opt/ros/humble/setup.bash && '
            'sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console'
        ],
        output='screen'
    )

    # Static TF: map -> world
    map_world_tf_bridge = Node(
        package='nav6d_sim',
        executable='map_world_tf_bridge',
        name='map_world_tf_bridge',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'world',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        }]
    )

    # Gazebo <-> ROS bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/front_depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/goaero_mission3/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
        ]
    )

    # PoseArray -> TF bridge
    pose_tf_bridge = Node(
        package='gz_pose_tf',
        executable='pose_tf_bridge',
        name='pose_tf_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    # TF -> /space_cobot/pose
    tf_to_pose_bridge = Node(
        package='nav6d_sim',
        executable='tf_to_pose_bridge',
        name='tf_to_pose_bridge',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'base_link',
            'pose_topic': '/space_cobot/pose',
            'publish_period_s': 0.05,
        }]
    )

    # Planner
    n6d_planner = Node(
        package='nav6d',
        executable='n6d_planner',
        name='n6d_planner',
        output='screen',
    )

    # Trajectory optimiser
    nav_6d_optimize_traj = Node(
        package='nav6d_sim',
        executable='nav_6d_optimize_traj',
        name='nav_6d_optimize_traj',
        output='screen',
        parameters=[{
            'input_topic': '/nav6d/planner/path',
            'v_ref': 1.2,
            'a_ref': 0.8,
            'max_velocity': 2.0,
            'max_acceleration': 1.5,
            'time_scale': 1.15,
            'corner_time_gain': 0.35,
        }]
    )

    # Mission manager
    mission_manager = Node(
        package='nav6d_sim',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )

    return LaunchDescription([
        gazebo,

        # Give Gazebo a moment to come up
        # TimerAction(period=2.0, actions=[mavproxy]),
        TimerAction(period=2.0, actions=[map_world_tf_bridge]),
        TimerAction(period=3.0, actions=[ros_gz_bridge]),
        TimerAction(period=4.0, actions=[pose_tf_bridge]),
        TimerAction(period=4.5, actions=[tf_to_pose_bridge]),
        TimerAction(period=5.0, actions=[n6d_planner]),
        TimerAction(period=5.5, actions=[nav_6d_optimize_traj]),
        TimerAction(period=6.0, actions=[mission_manager]),
    ])
