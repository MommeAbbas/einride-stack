from glob import glob
from setuptools import find_packages, setup

package_name = 'sim'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.txt') + glob('config/*.json')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Project contributors', maintainer_email='maintainers@example.invalid',
    description='Multi-robot simulation, mission emulation, metrics, and experiment tools', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'sim_world_node = sim.world_node:main',
        'sim_truth_node = sim.truth_node:main',
        'sim_map_node = sim.map_node:main',
        'sim_traffic_node = sim.traffic_node:main',
        'sim_organiser_node = sim.organiser_node:main',
        'sim_collision_node = sim.collision_node:main',
        'sim_detector_node = sim.detector_node:main',
        'sim_metrics_node = sim.metrics_node:main',
        'sim_experiment_node = sim.runner:main',
        'sim_fixture_node = sim.fixture_node:main',
        'sim_visualization_node = sim.visualization_node:main',
        'sim_skeleton_node = sim.skeleton_node:main',
    ]},
)
