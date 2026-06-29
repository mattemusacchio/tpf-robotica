"""Parte C — ROBOT REAL (sesion de laboratorio, closed-loop).

A diferencia de part_c_rosbag_validation (open-loop, para el bag), aca el robot se
mueve de verdad: corre localizacion (MCL) + planner (A*) + control (Pure Pursuit)
+ maquina de estados + deteccion de conos.

Parametros del robot (topicos, transformada del laser, ruido de movimiento) viven
en config/navigation_params_real.yaml -> editar AHI si los topicos del lab cambian.

Uso tipico:
    ros2 launch tpf_navigation part_c_real_robot.launch.py
    # luego en RViz: "2D Pose Estimate" sobre la posicion real del robot.

Si el mapa del lab no es el del bag, pasar otro:
    ros2 launch tpf_navigation part_c_real_robot.launch.py map_yaml:=/ruta/al/mapa.yaml
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
    # Fuente de verdad de parametros del robot real:
    nav_params = PathJoinSubstitution([pkg_nav, 'config', 'navigation_params_real.yaml'])
    cone_params = PathJoinSubstitution([pkg_perception, 'config', 'red_cone_detector.yaml'])
    rviz_config = PathJoinSubstitution([pkg_nav, 'rviz', 'part_b_navigation.rviz'])

    default_map = os.path.abspath('log/maps/laberinto_map.yaml')

    map_yaml = LaunchConfiguration('map_yaml')
    robot = LaunchConfiguration('robot')
    use_rviz = LaunchConfiguration('rviz')

    # Cambiar de robot = cambiar `robot:=tb4_1`. Los remappings llevan TODO al
    # namespace elegido (se aplican siempre, sin depender de la precedencia de
    # parametros). Con robot:=tb4_0 quedan identidad (comportamiento por defecto).
    ns = ['/', robot]
    remap_scan_ns = ('/tb4_0/scan', ns + ['/scan'])        # mcl/astar (param /tb4_0/scan)
    remap_scan_plain = ('/scan', ns + ['/scan'])           # navigation_sm (hardcodea /scan)
    remap_odom = ('/tb4_0/odom', ns + ['/odom'])
    remap_image = ('/tb4_0/oakd/rgb/preview/image_raw', ns + ['/oakd/rgb/preview/image_raw'])
    remap_caminfo = ('/tb4_0/oakd/rgb/preview/camera_info', ns + ['/oakd/rgb/preview/camera_info'])
    remap_cmdvel = ('/cmd_vel', ns + ['/cmd_vel'])         # salida hacia el robot real

    return LaunchDescription([
        # Robot real -> reloj real (NO sim time).
        DeclareLaunchArgument('map_yaml', default_value=default_map,
                              description='Mapa del laberinto (de Parte A). Cambiar si el lab usa otro.'),
        DeclareLaunchArgument('robot', default_value='tb4_0',
                              description='Namespace del robot: tb4_0 | tb4_1 (cambia scan/odom/imagen/cmd_vel).'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'use_sim_time': False,
                         'autostart': True,
                         'node_names': ['map_server']}],
            output='screen',
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='tpf_navigation',
                executable='mcl_localizer',
                name='mcl_localizer',
                parameters=[nav_params],
                remappings=[remap_scan_ns, remap_odom],
                output='screen',
            ),
            Node(
                package='tpf_navigation',
                executable='astar_planner',
                name='astar_planner',
                parameters=[nav_params],
                remappings=[remap_scan_ns],
                output='screen',
            ),
            Node(
                package='tpf_navigation',
                executable='pure_pursuit',
                name='pure_pursuit',
                parameters=[nav_params],
                remappings=[remap_cmdvel],
                output='screen',
            ),
            Node(
                package='tpf_navigation',
                executable='navigation_sm',
                name='navigation_sm',
                parameters=[nav_params],
                remappings=[remap_scan_plain, remap_cmdvel],
                output='screen',
            ),
            Node(
                package='tpf_perception',
                executable='red_cone_detector_node',
                name='red_cone_detector_node',
                parameters=[cone_params, {'use_sim_time': False}],
                remappings=[remap_image, remap_caminfo],
                output='screen',
            ),
        ]),

        TimerAction(period=3.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': False}],
                condition=IfCondition(use_rviz),
                output='screen',
            ),
        ]),
    ])
