from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare the use_sim_time argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Default to true for simulation time
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the use_sim_time configuration
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("go2_config"),
                'launch',
                'gazebo_mid360.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Launch mapping module
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fast_lio"),
            'launch',
            'mapping.launch.py'
            )
        ),
        launch_arguments={'config_file': 'unitree_go2_mid360.yaml', 'use_sim_time': use_sim_time}.items()
    )

    # Waypoint Follower Node
    waypoint_follower_node = Node(
        package='waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Local Map Creator Node
    local_map_node = Node(
        package='local_map_creator',
        executable='local_map_creator',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Static Transform Node for Robot-Camera connection
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '1.57', '0', '0', 'odom', 'body'],
        output='screen',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Bug1 Walker Node
    bug1_walker_node = Node(
        package='cw1_team_9',
        executable='bug1',
        name='bug1',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Go-to-Point Node
    go_to_point_node = Node(
        package='cw1_team_9',
        executable='go_to_point',
        name='go_to_point',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Edge Follower Node
    edge_follower_node = Node(
        package='cw1_team_9',
        executable='edge_follower',
        name='edge_follower',
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Organizing launch groups
    gazebo_group = GroupAction([gazebo_launch])
    mapping_group = GroupAction([mapping_launch])
    local_map_group = GroupAction([local_map_node])
    static_tf_group = GroupAction([static_tf_node])
    waypoint_follower_group = GroupAction([waypoint_follower_node, bug1_walker_node, go_to_point_node, edge_follower_node])

    return LaunchDescription([
        declare_use_sim_time,
        gazebo_group,
        mapping_group,
        local_map_group,
        static_tf_group,
        waypoint_follower_group
    ])

