import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch your C++ offboard control node
        launch_ros.actions.Node(
            package='offboard_control',
            executable='sensor_combined_listener_node',
            name='sensor_combined_listener_node',
            output='screen'
        ),
    ])