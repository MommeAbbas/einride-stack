from glob import glob
from setuptools import find_packages, setup

package_name = 'slam'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Project contributors', maintainer_email='maintainers@example.invalid',
    description='SLAM, localization, dock mapping, snapshots, and wheel/IMU odometry', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'slam_wheel_odometry_node = slam.wheel_odometry_node:main',
        'slam_skeleton_node = slam.skeleton_node:main',
        'slam_slam_node = slam.slam_node:main',
    ]},
)
