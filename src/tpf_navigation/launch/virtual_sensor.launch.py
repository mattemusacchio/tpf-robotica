from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tpf_navigation')

    landmarks_file = LaunchConfiguration('landmarks_file')
    odom_topic = LaunchConfiguration('odom_topic')
    max_range = LaunchConfiguration('max_range_m')
    sigma_range = LaunchConfiguration('sigma_range')
    sigma_bearing = LaunchConfiguration('sigma_bearing')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'landmarks_file',
            default_value=PathJoinSubstitution([pkg, 'config', 'virtual_landmarks.yaml']),
            description='Path to virtual_landmarks.yaml',
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('max_range_m', default_value='3.5'),
        DeclareLaunchArgument('sigma_range', default_value='0.05'),
        DeclareLaunchArgument('sigma_bearing', default_value='0.02'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='tpf_navigation',
            executable='virtual_aruco_sensor',
            name='virtual_aruco_sensor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'landmarks_file': landmarks_file,
                'odom_topic': odom_topic,
                'max_range_m': max_range,
                'sigma_range': sigma_range,
                'sigma_bearing': sigma_bearing,
            }],
        ),
    ])
