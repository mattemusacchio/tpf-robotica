"""Parte C stack: localization, planning, control, and red-cone goals.

Typical rosbag validation:
    ros2 launch tpf_navigation part_c_cone_search.launch.py
    ros2 bag play data/rosbags/laberinto_conos --clock
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('tpf_navigation')
    pkg_perception = FindPackageShare('tpf_perception')
    nav_params = PathJoinSubstitution([pkg_nav, 'config', 'navigation_params.yaml'])
    cone_params = PathJoinSubstitution([pkg_perception, 'config', 'red_cone_detector.yaml'])
    rviz_config = PathJoinSubstitution([pkg_nav, 'rviz', 'part_b_navigation.rviz'])

    default_map = os.path.abspath('log/maps/laberinto_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    n_particles = LaunchConfiguration('n_particles')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock; true for rosbag validation.'),
        DeclareLaunchArgument('map_yaml', default_value=default_map,
                              description='Occupancy map generated from the real maze.'),
        DeclareLaunchArgument('odom_topic', default_value='/tb4_0/odom'),
        DeclareLaunchArgument('scan_topic', default_value='/tb4_0/scan'),
        DeclareLaunchArgument('image_topic', default_value='/tb4_0/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/tb4_0/oakd/rgb/preview/camera_info'),
        DeclareLaunchArgument('n_particles', default_value='800'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_yaml}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': ['map_server']}],
            output='screen',
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='tpf_navigation',
                executable='mcl_localizer',
                name='mcl_localizer',
                parameters=[nav_params,
                            {'use_sim_time': use_sim_time,
                             'num_particles': n_particles,
                             'init_x_m': -0.34,
                             'init_y_m': 1.19,
                             'init_pos_std_m': 0.30,
                             'odom_topic': odom_topic,
                             'scan_topic': scan_topic}],
                output='screen',
            ),
            Node(
                package='tpf_navigation',
                executable='astar_planner',
                name='astar_planner',
                parameters=[nav_params,
                            {'use_sim_time': use_sim_time,
                             'scan_topic': scan_topic}],
                output='screen',
            ),
            Node(
                package='tpf_navigation',
                executable='pure_pursuit',
                name='pure_pursuit',
                parameters=[nav_params,
                            {'use_sim_time': use_sim_time}],
                output='screen',
            ),
            Node(
                package='tpf_navigation',
                executable='navigation_sm',
                name='navigation_sm',
                parameters=[nav_params,
                            {'use_sim_time': use_sim_time,
                             'obstacle_confirm_hits': 2}],
                output='screen',
            ),
            Node(
                package='tpf_perception',
                executable='red_cone_detector_node',
                name='red_cone_detector_node',
                parameters=[cone_params,
                            {'use_sim_time': use_sim_time,
                             'image_topic': image_topic,
                             'camera_info_topic': camera_info_topic}],
                output='screen',
            ),
        ]),

        TimerAction(period=3.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]),
    ])
