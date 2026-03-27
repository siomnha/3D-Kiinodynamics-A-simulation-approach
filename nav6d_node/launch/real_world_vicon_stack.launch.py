from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Optional static alignment between nav6d map and vicon world frame.
    map_world_tf_bridge = Node(
        package='nav6d_sim',
        executable='map_world_tf_bridge',
        name='map_world_tf_bridge',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'vicon_world',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
        }],
    )

    tf_to_pose_bridge = Node(
        package='nav6d_sim',
        executable='tf_to_pose_bridge',
        name='tf_to_pose_bridge',
        output='screen',
        parameters=[{
            'parent_frame': 'map',
            'child_frame': 'vicon/px4/px4',  # adjust subject/segment to your Vicon object
            'pose_topic': '/space_cobot/pose',
            'publish_period_s': 0.02,
            'lookup_timeout_s': 0.02,
        }],
    )

    n6d_planner = Node(
        package='nav6d',
        executable='n6d_planner',
        name='n6d_planner',
        output='screen',
    )

    nav_6d_optimize_traj = Node(
        package='nav6d_sim',
        executable='nav_6d_optimize_traj',
        name='nav_6d_optimize_traj',
        output='screen',
        parameters=[{
            'input_topic': '/nav6d/planner/path',
            'v_ref': 1.0,
            'a_ref': 0.7,
            'max_velocity': 1.5,
            'max_acceleration': 1.0,
            'time_scale': 1.2,
            'corner_time_gain': 0.4,
            'resume_on_replan': True,
            'pose_topic': '/trajectory/reference_pose',
        }],
    )

    mission_manager = Node(
        package='nav6d_sim',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'waypoints_topic': '/waypoints',
            'mavros_waypoints_topic': '/mavros/mission/waypoints',
            'goal_topic': '/nav6d/goal',
            'pose_topic': '/space_cobot/pose',
            'goal_frame_id': 'map',
            'arrival_threshold_m': 0.5,
            'mavros_waypoints_are_global': True,
            # Set these to your indoor lab local-map origin before flight:
            'origin_lat': 0.0,
            'origin_lon': 0.0,
            'origin_alt': 0.0,
        }],
    )

    return LaunchDescription([
        map_world_tf_bridge,
        tf_to_pose_bridge,
        n6d_planner,
        nav_6d_optimize_traj,
        mission_manager,
    ])
