from setuptools import find_packages, setup

package_name = 'tpf_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aruco_detector.launch.py']),
        ('share/' + package_name + '/config', ['config/aruco_detector.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TPF Robotica Team',
    maintainer_email='mmatt@example.com',
    description='Perception nodes for ArUco landmark detection in the robotics final project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = tpf_perception.aruco_detector_node:main',
        ],
    },
)
