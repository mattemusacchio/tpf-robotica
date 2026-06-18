from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    aruco_config_file = LaunchConfiguration('aruco_config_file')
    slam_config_file = LaunchConfiguration('slam_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'aruco_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('tpf_perception'),
                'config',
                'aruco_detector.yaml',
            ]),
            description='Path to the ArUco detector ROS parameters file.',
        ),
        DeclareLaunchArgument(
            'slam_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('tpf_slam'),
                'config',
                'graph_slam_frontend.yaml',
            ]),
            description='Path to the Graph SLAM front-end ROS parameters file.',
        ),
        Node(
            package='tpf_perception',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            parameters=[aruco_config_file],
        ),
        Node(
            package='tpf_slam',
            executable='graph_slam_frontend_node',
            name='graph_slam_frontend_node',
            output='screen',
            parameters=[slam_config_file],
        ),
    ])