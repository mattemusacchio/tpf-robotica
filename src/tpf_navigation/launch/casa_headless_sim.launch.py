"""Headless Gazebo launch for casa.world — gzserver only, no gzclient display."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_sim = get_package_share_directory('turtlebot3_custom_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(pkg_sim, 'worlds', 'casa.world')
    models_path = os.path.join(pkg_sim, 'worlds')
    os.environ['GAZEBO_MODEL_PATH'] = (
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    return LaunchDescription([
        # gzserver only — no GUI/display needed
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),
        # robot_state_publisher can start immediately (no Gazebo dependency)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # Delay spawn until gzserver has fully loaded casa.world and registered
        # the /spawn_entity service (casa.world takes ~25s on WSL gzserver).
        TimerAction(period=25.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
            ),
        ]),
    ])
