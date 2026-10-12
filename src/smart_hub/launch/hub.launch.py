"""Bring up the full hub: ros2 launch smart_hub hub.launch.py scenario:=S8 seed:=42 mode:=sim."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = [
    'scenario_runner',
    'passenger_detector',
    'demand_predictor',
    'bus_manager',
    'parking_manager',
    'traffic_monitor',
    'route_planner',
    'robot_manager',
    'hub_coordinator',
    'safety_supervisor',
    'actuator_bridge',
    'metrics_logger',
    'dashboard_marker_publisher',
]


def generate_launch_description():
    share = get_package_share_directory('smart_hub')
    params = os.path.join(share, 'config', 'hub.yaml')
    scenario = LaunchConfiguration('scenario')
    seed = LaunchConfiguration('seed')
    mode = LaunchConfiguration('mode')

    nodes = []
    for name in NODES:
        extra = {}
        if name == 'scenario_runner':
            extra = {'scenario': scenario, 'seed': seed,
                     'scenario_dir': os.path.join(share, 'scenarios')}
        elif name == 'actuator_bridge':
            extra = {'mode': mode}
        nodes.append(Node(
            package='smart_hub', executable=name, name=name,
            output='screen', parameters=[params, extra],
        ))

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='S1', description='S1..S9'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('mode', default_value='sim', description='sim | hardware'),
        *nodes,
    ])
