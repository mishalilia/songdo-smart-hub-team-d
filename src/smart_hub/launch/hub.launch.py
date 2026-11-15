# Author: Mikhail Li (12236295)
"""
Bring up the full hub.

  ros2 launch smart_hub hub.launch.py scenario:=S8 seed:=42 mode:=sim [variant:=...]
      [rviz:=true] [record:=true]

record:=true writes a rosbag of the assignment 20.2 topics to evidence/bags/.
"""
from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

BAG_TOPICS = [
    '/hub/queue', '/hub/demand', '/hub/vehicles', '/hub/bus_arrival',
    '/parking/state', '/traffic/edges', '/crossing/state',
    '/robot/odom', '/robot/cmd_vel_candidate', '/robot/cmd_vel',
    '/bus/cmd_vel_candidate', '/bus/cmd_vel', '/hub/estop',
    '/hub/safety', '/hub/events', '/tf', '/tf_static',
    '/sim/truth/vehicles', '/sim/truth/pedestrians', '/hub/metrics',
]

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
    'hub_ui',
]


def generate_launch_description():
    share = get_package_share_directory('smart_hub')
    params = os.path.join(share, 'config', 'hub.yaml')
    with open(params) as f:
        hub = yaml.safe_load(f)['hub_coordinator']['ros__parameters']
    scenario = LaunchConfiguration('scenario')
    seed = LaunchConfiguration('seed')
    mode = LaunchConfiguration('mode')
    sim_only = IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))

    nodes = []
    for name in NODES:
        extra = {}
        condition = None
        layers = [params]
        if name == 'scenario_runner':
            # The simulator replaces the physical world; hardware mode never starts it.
            extra = {'scenario': scenario, 'seed': ParameterValue(seed, value_type=int),
                     'variant': LaunchConfiguration('variant'),
                     'scenario_dir': os.path.join(share, 'scenarios')}
            condition = sim_only
        elif name == 'actuator_bridge':
            extra = {'mode': mode}
        elif name == 'hub_ui':
            extra = {'port': ParameterValue(LaunchConfiguration('ui_port'), value_type=int),
                     'mode': mode}
        elif name == 'hub_coordinator':
            # Keys present in hub.yaml under the node name outrank a dict passed as /** by
            # launch_ros, so the coordinator gets one merged dict with the arguments on top.
            layers = [dict(hub)]
            extra = {'dispatch_mode': LaunchConfiguration('dispatch_mode'),
                     'dispatch_high': ParameterValue(LaunchConfiguration('dispatch_high'),
                                                     value_type=float),
                     'dispatch_low': ParameterValue(LaunchConfiguration('dispatch_low'),
                                                    value_type=float)}
        nodes.append(Node(
            package='smart_hub', executable=name, name=name,
            output='screen', parameters=layers + [extra], condition=condition,
        ))

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(share, 'config', 'hub.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    stamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o',
             ['evidence/bags/', scenario, '_s', seed, '_' + stamp], *BAG_TOPICS],
        output='log', condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='S1', description='S1..S9'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('mode', default_value='sim', description='sim | hardware'),
        DeclareLaunchArgument('rviz', default_value='false', description='start RViz'),
        DeclareLaunchArgument('variant', default_value='', description='scenario test variant'),
        DeclareLaunchArgument('record', default_value='false', description='record a rosbag'),
        DeclareLaunchArgument('ui_port', default_value='8080', description='web UI port'),
        DeclareLaunchArgument('dispatch_mode', default_value='predictive',
                              description='predictive | reactive bus dispatch'),
        # Threshold ablation (evidence/dispatch_ablation): defaults are the config values.
        DeclareLaunchArgument('dispatch_high', default_value=str(hub['dispatch_high'])),
        DeclareLaunchArgument('dispatch_low', default_value=str(hub['dispatch_low'])),
        *nodes,
        rviz,
        bag,
    ])
