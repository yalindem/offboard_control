import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([

        # Launch your C++ offboard control node
        launch_ros.actions.Node(
            package='offboard_control',
            executable='offboard_control_node',
            name='offboard_control_node',
            output='screen'
        ),
    ])