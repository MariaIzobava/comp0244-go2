import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cw_1_team_22',
            executable='bug0',
            name='bug0'),
  ])
