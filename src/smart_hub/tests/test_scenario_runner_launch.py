# Author: Mikhail Li (12236295)
"""launch_testing: scenario_runner publishes observations and serves /scenario/reset."""
import os
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
from smart_hub.qos import R10, R100
from smart_hub_interfaces.msg import HubEvent, VehicleStateArray
from smart_hub_interfaces.srv import ResetScenario

SCENARIO_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scenarios')


@pytest.mark.launch_test
def generate_test_description():
    runner = Node(package='smart_hub', executable='scenario_runner',
                  parameters=[{'scenario': 'S1', 'seed': 42, 'scenario_dir': SCENARIO_DIR,
                               'reset_ack_nodes': ['scenario_runner_test'],
                               'reset_ack_timeout_sec': 2.0}])
    return LaunchDescription([runner, launch_testing.actions.ReadyToTest()])


class TestScenarioRunner(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('scenario_runner_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_vehicles_published(self):
        received = []
        sub = self.node.create_subscription(
            VehicleStateArray, '/hub/vehicles', received.append, R10)
        deadline = time.monotonic() + 10.0
        while len(received) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        self.assertGreaterEqual(len(received), 5)
        ids = {v.vehicle_id for v in received[-1].vehicles}
        self.assertIn('bus1', ids)
        self.assertIn('robot1', ids)

    def test_reset_returns_new_run_id_after_ack(self):
        events = self.node.create_publisher(HubEvent, '/hub/events', R100)

        def ack(msg):                      # behave like a node that resets and acknowledges
            if msg.event == 'RESET':
                events.publish(HubEvent(event='RESET_ACK', entity_id='scenario_runner_test',
                                        correlation_id=msg.correlation_id))

        self.node.create_subscription(HubEvent, '/hub/events', ack, R100)
        client = self.node.create_client(ResetScenario, '/scenario/reset')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        req = ResetScenario.Request(scenario_id='S1', seed=7, preserve_logs=True)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success, future.result().reason)
        self.assertTrue(future.result().run_id.startswith('S1-s7-'))
        bad = client.call_async(ResetScenario.Request(scenario_id='S99', seed=1))
        rclpy.spin_until_future_complete(self.node, bad, timeout_sec=10.0)
        self.assertFalse(bad.result().success)
