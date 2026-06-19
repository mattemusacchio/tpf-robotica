from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('tpf_slam'),
        'rviz',
        'part_a_graph_slam.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value='data/rosbags/laberinto'),
        DeclareLaunchArgument('optimized_graph_path', default_value='log/laberinto_optimized_graph.json'),
        DeclareLaunchArgument('resolution', default_value='0.08'),
        DeclareLaunchArgument('max_range_m', default_value='5.0'),
        DeclareLaunchArgument('scan_stride', default_value='20'),
        DeclareLaunchArgument('beam_stride', default_value='10'),
        DeclareLaunchArgument('inflate_radius_m', default_value='0.08'),
        DeclareLaunchArgument('min_occupied_component_cells', default_value='4'),
        DeclareLaunchArgument('use_tf_static', default_value='false'),
        DeclareLaunchArgument('laser_x_m', default_value='-0.04'),
        DeclareLaunchArgument('laser_y_m', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw_rad', default_value='1.5707963267948966'),
        DeclareLaunchArgument('publish_every_scans', default_value='10'),
        DeclareLaunchArgument('playback_rate_hz', default_value='8.0'),
        DeclareLaunchArgument('max_processed_scans', default_value='0'),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='tpf_slam',
            executable='progressive_mapping_demo_node',
            name='part_a_graph_slam_demo_node',
            output='screen',
            parameters=[{
                'bag_path': LaunchConfiguration('bag_path'),
                'optimized_graph_path': LaunchConfiguration('optimized_graph_path'),
                'map_topic': '/map',
                'path_topic': '/belief',
                'landmarks_topic': '/landmarks',
                'poses_topic': '/poses_guardadas',
                'status_topic': '/slam/part_a_status',
                'resolution': ParameterValue(LaunchConfiguration('resolution'), value_type=float),
                'max_range_m': ParameterValue(LaunchConfiguration('max_range_m'), value_type=float),
                'scan_stride': ParameterValue(LaunchConfiguration('scan_stride'), value_type=int),
                'beam_stride': ParameterValue(LaunchConfiguration('beam_stride'), value_type=int),
                'inflate_radius_m': ParameterValue(LaunchConfiguration('inflate_radius_m'), value_type=float),
                'min_occupied_component_cells': ParameterValue(
                    LaunchConfiguration('min_occupied_component_cells'),
                    value_type=int,
                ),
                'use_tf_static': ParameterValue(LaunchConfiguration('use_tf_static'), value_type=bool),
                'laser_x_m': ParameterValue(LaunchConfiguration('laser_x_m'), value_type=float),
                'laser_y_m': ParameterValue(LaunchConfiguration('laser_y_m'), value_type=float),
                'laser_yaw_rad': ParameterValue(LaunchConfiguration('laser_yaw_rad'), value_type=float),
                'publish_every_scans': ParameterValue(LaunchConfiguration('publish_every_scans'), value_type=int),
                'playback_rate_hz': ParameterValue(LaunchConfiguration('playback_rate_hz'), value_type=float),
                'max_processed_scans': ParameterValue(LaunchConfiguration('max_processed_scans'), value_type=int),
                'loop': ParameterValue(LaunchConfiguration('loop'), value_type=bool),
            }],
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
