"""Bring up the full hub: ros2 launch smart_hub hub.launch.py scenario:=S8 seed:=42 mode:=sim."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    sim_only = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))

    nodes = []
    for name in NODES:
        extra = {}
        condition = None
        if name == 'scenario_runner':
            # The simulator replaces the physical world; hardware mode never starts it.
            extra = {'scenario': scenario, 'seed': ParameterValue(seed, value_type=int),
                     'scenario_dir': os.path.join(share, 'scenarios')}
            condition = sim_only
        elif name == 'actuator_bridge':
            extra = {'mode': mode}
        nodes.append(Node(
            package='smart_hub', executable=name, name=name,
            output='screen', parameters=[params, extra], condition=condition,
        ))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(share, 'config', 'hub.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='S1', description='S1..S9'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('mode', default_value='sim', description='sim | hardware'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz'),
        *nodes,
        rviz,
    ])
