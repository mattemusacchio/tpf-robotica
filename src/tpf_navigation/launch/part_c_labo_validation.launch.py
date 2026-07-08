"""Parte B/C validation over the laboratory bag and map (tb4_1).

This is the rehearsal launch for the already-built lab map:

    ros2 launch tpf_navigation part_c_labo_validation.launch.py rviz:=false
    ros2 bag play data/labo --clock

It reuses the rosbag validation stack but defaults every argument to the
laboratory capture: map ``log/maps/labo_map_v2.yaml``, robot namespace
``tb4_1``, and ``navigation_params_labo.yaml``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav = FindPackageShare('tpf_navigation')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('n_particles', default_value='2500'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_nav, 'launch', 'part_c_rosbag_validation.launch.py',
                ])
            ),
            launch_arguments={
                'map_yaml': os.path.abspath('log/maps/labo_map_v2.yaml'),
                'nav_params': PathJoinSubstitution([
                    pkg_nav, 'config', 'navigation_params_labo.yaml',
                ]),
                'odom_topic': '/tb4_1/odom',
                'scan_topic': '/tb4_1/scan',
                'image_topic': '/tb4_1/oakd/rgb/preview/image_raw',
                'camera_info_topic': '/tb4_1/oakd/rgb/preview/camera_info',
                'init_x': '1.10',
                'init_y': '-1.10',
                'init_std': '0.80',
                'use_static_calibration': 'false',
                'rviz': LaunchConfiguration('rviz'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'n_particles': LaunchConfiguration('n_particles'),
            }.items(),
        ),
    ])
