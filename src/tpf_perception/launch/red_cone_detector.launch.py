"""Launch the Parte C red-cone detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tpf_perception')
    params_file = PathJoinSubstitution([pkg, 'config', 'red_cone_detector.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        Node(
            package='tpf_perception',
            executable='red_cone_detector_node',
            name='red_cone_detector_node',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),
    ])
