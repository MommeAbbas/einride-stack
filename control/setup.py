from glob import glob
from setuptools import find_packages, setup

package_name = 'control'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Project contributors', maintainer_email='maintainers@example.invalid',
    description='Nav2 coordination, docking, collision guarding, and recovery control', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'control_escape_node = control.escape_node:main',
        'control_guard_deadband = control.guard_deadband:main',
        'control_docking_adapter = control.docking_adapter:main',
        'control_navigation_relay = control.navigation_relay:main',
        'control_direct_command_node = control.direct_command_node:main',
        'control_skeleton_node = control.skeleton_node:main',
        'control_coordinator_node = control.nav2_coordinator:main',
        'control_depth_node = control.depth_node:main',
    ]},
)
