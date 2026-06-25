"""Complete Parte B navigation stack launch file.

Usage:
    ros2 launch tpf_navigation part_b_navigation.launch.py
    ros2 launch tpf_navigation part_b_navigation.launch.py world:=custom_casa_obs
    ros2 launch tpf_navigation part_b_navigation.launch.py rviz:=false n_particles:=1000
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('tpf_navigation')
    params_file = PathJoinSubstitution([pkg_nav, 'config', 'navigation_params.yaml'])
    landmarks_file = PathJoinSubstitution([pkg_nav, 'config', 'virtual_landmarks.yaml'])
    map_file = PathJoinSubstitution([pkg_nav, 'maps', 'casa_map.yaml'])
    rviz_config = PathJoinSubstitution([pkg_nav, 'rviz', 'part_b_navigation.rviz'])

    world = LaunchConfiguration('world')
    n_particles = LaunchConfiguration('n_particles')
    use_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_pkg = get_package_share_directory('turtlebot3_custom_simulation')

    return LaunchDescription([
        # ── Arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('world', default_value='custom_casa',
                              description='Gazebo world: custom_casa | custom_casa_obs'),
        DeclareLaunchArgument('n_particles', default_value='500'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── Gazebo simulation ─────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'custom_casa.launch.py')),
        ),

        # ── Map server (static occupancy map) ────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_file}],
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

        # ── Navigation nodes (delayed to let Gazebo start) ───────────
        TimerAction(period=5.0, actions=[
            Node(
                package='tpf_navigation',
                executable='virtual_aruco_sensor',
                name='virtual_aruco_sensor',
                parameters=[params_file,
                            {'use_sim_time': use_sim_time,
                             'landmarks_file': landmarks_file}],
                output='screen',
            ),

            Node(
                package='tpf_navigation',
                executable='mcl_localizer',
                name='mcl_localizer',
                parameters=[params_file,
                            {'use_sim_time': use_sim_time,
                             'num_particles': n_particles,
                             'landmarks_file': landmarks_file}],
                output='screen',
            ),

            Node(
                package='tpf_navigation',
                executable='astar_planner',
                name='astar_planner',
                parameters=[params_file,
                            {'use_sim_time': use_sim_time}],
                output='screen',
            ),

            Node(
                package='tpf_navigation',
                executable='pure_pursuit',
                name='pure_pursuit',
                parameters=[params_file,
                            {'use_sim_time': use_sim_time}],
                output='screen',
            ),

            Node(
                package='tpf_navigation',
                executable='navigation_sm',
                name='navigation_sm',
                parameters=[params_file,
                            {'use_sim_time': use_sim_time}],
                output='screen',
            ),
        ]),

        # ── RViz ──────────────────────────────────────────────────────
        TimerAction(period=6.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
                additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
                output='screen',
            ),
        ]),
    ])
