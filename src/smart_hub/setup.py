# Author: Mikhail Li (12236295)
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'smart_hub'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'models'), glob('models/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Smart Hub Team',
    maintainer_email='team@example.com',
    description='Smart Mobility Hub application nodes (rclpy).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_runner = smart_hub.scenario_runner:main',
            'passenger_detector = smart_hub.passenger_detector:main',
            'demand_predictor = smart_hub.demand_predictor:main',
            'bus_manager = smart_hub.bus_manager:main',
            'parking_manager = smart_hub.parking_manager:main',
            'traffic_monitor = smart_hub.traffic_monitor:main',
            'route_planner = smart_hub.route_planner:main',
            'robot_manager = smart_hub.robot_manager:main',
            'hub_coordinator = smart_hub.hub_coordinator:main',
            'safety_supervisor = smart_hub.safety_supervisor:main',
            'actuator_bridge = smart_hub.actuator_bridge:main',
            'metrics_logger = smart_hub.metrics_logger:main',
            'dashboard_marker_publisher = smart_hub.dashboard_marker_publisher:main',
            'scalability_benchmark = smart_hub.scalability_benchmark:main',
            'hub_ui = smart_hub.hub_ui:main',
            'train_demand_model = smart_hub.train_demand_model:main',
            'run_test_matrix = smart_hub.test_matrix:main',
            'video_source = smart_hub.video_source:main',
            'evaluate_s1_video = smart_hub.evaluate_s1_video:main',
        ],
    },
)
