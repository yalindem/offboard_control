import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch the first process for PX4
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'offboard_control', 'start_px4'],
            output='screen'
        ),
        # Launch your C++ offboard control node
        launch_ros.actions.Node(
            package='offboard_control',
            executable='sensor_combined_listener_node',
            name='sensor_combined_listener_node',
            output='screen'
        ),
    ])