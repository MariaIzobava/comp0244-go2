from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ros2 launch go2_config gazebo_mid360.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("go2_config"),
                'launch',
                'gazebo_mid360.launch.py'
            )
        )
    )

    # ros2 launch fast_lio mapping.launch.py
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fast_lio"),
            'launch',
            'mapping.launch.py'
            )
        ),
        launch_arguments={'config_file': 'unitree_go2_mid360.yaml'}.items()
    )

    # ros2 run waypoint_follower waypoint_follower
    waypoint_follower_node = ExecuteProcess(
        cmd=['ros2', 'run', 'waypoint_follower', 'waypoint_follower'],
        output='screen'
    )
    
    # ros2 run local_map_creator local_map_creator
    local_map_node = ExecuteProcess(
        cmd=['ros2', 'run', 'local_map_creator', 'local_map_creator'],
        output='screen'
    )

    # ros2 run static_tf_node to connect the robot with the camera
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
        output='screen'
    )

    # segment groups
    gazebo_group = GroupAction([
        gazebo_launch
    ])

    mapping_group = GroupAction([
        mapping_launch
    ])

    waypoint_follower_group = GroupAction([
        waypoint_follower_node
    ])
    
    local_map_group = GroupAction([
        local_map_node
    ])

    static_tfe_group = GroupAction([
        static_tf_node
    ])

    return LaunchDescription([
        gazebo_group,
        mapping_group,
        waypoint_follower_group,
        local_map_group,
        static_tfe_group
    ])
