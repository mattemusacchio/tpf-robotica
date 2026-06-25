from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tpf_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matteo',
    maintainer_email='mmusacchio@soflex.com.ar',
    description='Parte B: MCL localization, A* planning, Pure Pursuit control and state machine.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_aruco_sensor = tpf_navigation.virtual_aruco_sensor:main',
            'mcl_localizer = tpf_navigation.mcl_localizer:main',
            'astar_planner = tpf_navigation.astar_planner:main',
            'pure_pursuit = tpf_navigation.pure_pursuit:main',
            'navigation_sm = tpf_navigation.navigation_sm:main',
        ],
    },
)
