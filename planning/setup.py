from glob import glob
from setuptools import find_packages, setup

package_name = 'planning'
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
    description='Mission planning, dock occupancy, exploration, and camera coverage', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'planning_coverage_node = planning.coverage_node:main',
        'planning_skeleton_node = planning.skeleton_node:main',
        'planning_planning_node = planning.mission_node:main',
    ]},
)
