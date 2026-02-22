import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'offboard_control'
    urdf_file_name = 'x500_flow.urdf'

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher: URDF'deki base_link -> rotor bağlantılarını yayınlar
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='offboard_control',
            executable='offboard_control_node',
            name='offboard_control_node',
            output='screen'
        ),
        # Senin yazdığın C++ nodu (Odom -> Base_link yayınlayan)
        Node(
            package='offboard_control',
            executable='target_localizer_node', # C++ dosyanın executable adı
            name='target_localizer',
            output='screen'
        ),
        
        # Statik Transform: map -> odom (Harita ve kalkış noktası aynı kabul edilir)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    ])