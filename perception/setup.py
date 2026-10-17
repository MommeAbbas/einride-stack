from glob import glob
from setuptools import find_packages, setup

package_name = 'perception'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.txt')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Project contributors', maintainer_email='maintainers@example.invalid',
    description='Detector adapters, sensor fusion, tracking, prediction, and dock-state inference', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'perception_detector_node = perception.detector_node:main',
        'perception_skeleton_node = perception.skeleton_node:main',
        'perception_dock_node = perception.dock_node:main',
        'perception_perception_node = perception.perception_node:main',
    ]},
)
