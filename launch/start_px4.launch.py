


from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    # Dizinler - kişiselleştirilmelidir
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    agent_dir = os.path.expanduser('~/Micro-XRCE-DDS-Agent')
    agent_port = '8888'

    # Micro XRCE-DDS Agent başlat
    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
        cwd=agent_dir,
        output="screen"
    )

    # PX4 SITL + Gazebo başlat
    px4_gz = ExecuteProcess(
        cmd=["make", "px4_sitl", "gz_x500_mono_cam"],
        cwd=px4_dir,
        output="screen",
        additional_env={"PX4_GZ_WORLD": "walls"}
    )

    # (İsteğe Bağlı) ROS <-> Gazebo köprüsü örneği
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
           '/world/walls/model/x500_mono_cam_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image',
           '/world/walls/model/x500_mono_cam_0/link/lidar_link/sensor/front_lidar/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
           '/world/walls/model/x500_mono_cam_0/link/lidar_link/sensor/front_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen'
    )

    offboard_controller_node = Node(
        package='offboard_control',
        executable='offboard_control_node',
        output='screen'
    )

    range_detector_node = Node(
        package='offboard_control',
        executable='range_detector_node',
        output='screen'
    )

    return LaunchDescription([
        microxrce_agent,
        px4_gz,
        ros_gz_bridge,
        offboard_controller_node,
        range_detector_node
    ])
