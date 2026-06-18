from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('tpf_slam'),
                'config',
                'graph_slam_frontend.yaml',
            ]),
            description='Path to the Graph SLAM front-end ROS parameters file.',
        ),
        Node(
            package='tpf_slam',
            executable='graph_slam_frontend_node',
            name='graph_slam_frontend_node',
            output='screen',
            parameters=[config_file],
        ),
    ])