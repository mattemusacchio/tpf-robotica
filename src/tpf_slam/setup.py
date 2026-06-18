from setuptools import find_packages, setup

package_name = 'tpf_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/graph_slam_frontend.launch.py',
            'launch/aruco_graph_frontend.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/graph_slam_frontend.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TPF Robotica Team',
    maintainer_email='mmatt@example.com',
    description='Graph SLAM front-end for odometry and ArUco landmark observations.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_slam_frontend_node = tpf_slam.graph_slam_frontend_node:main',
            'graph_slam_backend = tpf_slam.graph_slam_backend:main',
            'offline_rosbag_graph_builder = tpf_slam.offline_rosbag_graph_builder:main',
            'occupancy_grid_builder = tpf_slam.occupancy_grid_builder:main',
        ],
    },
)