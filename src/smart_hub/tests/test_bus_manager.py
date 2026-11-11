# Author: Vadim Lifrov (12240192)
"""Bus manager tests (10, T05 / T06): goal validation, ETA, docking gate."""
import time

import pytest
import rclpy
from smart_hub.bus_manager import BusManager, Mission
from smart_hub_interfaces.action import DockBus
from smart_hub_interfaces.msg import SafetyState


@pytest.fixture
def node():
    rclpy.init()
    n = BusManager()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def ready(n, pose=(1.0, 2.0, 1.5708, 0.0)):
    now = time.monotonic()
    n.pose = (*pose, now)
    n.safety = (SafetyState(stop_latched=False, motion_permitted=True), now)


def goal(bus='bus1', bay='B', duration=240.0):
    return DockBus.Goal(bus_id=bus, bay_id=bay, max_duration_sec=duration)


def test_goal_validation(node):
    from rclpy.action import GoalResponse
    assert node.on_goal(goal()) == GoalResponse.REJECT          # no pose / safety yet
    ready(node)
    assert node.on_goal(goal(bus='bus9')) == GoalResponse.REJECT
    assert node.on_goal(goal(bay='P1')) == GoalResponse.REJECT
    assert node.on_goal(goal(duration=0.0)) == GoalResponse.REJECT
    assert node.on_goal(goal()) == GoalResponse.ACCEPT
    assert node.on_goal(goal()) == GoalResponse.REJECT          # busy: one mission


def test_eta_from_remaining_route_and_speed(node):
    ready(node)
    m = Mission(goal(), time.monotonic() + 240)
    m.route, m.i = ['A', 'N', 'Cn', 'B'], 1
    node.smoothed_speed = 0.0
    assert node.eta(m) == (-1.0, False)                           # stopped: ETA invalid
    node.smoothed_speed = 0.4
    eta, valid = node.eta(m)
    # 8 m cruising at 0.4 m/s, the last 1 m at the 0.12 m/s docking speed.
    assert valid and eta == pytest.approx(8.0 / 0.4 + 1.0 / 0.12)


def test_docking_needs_pose_yaw_and_one_second_hold(node):
    ready(node, pose=(8.0, 4.0, 0.0, 0.0))
    m = Mission(goal(), time.monotonic() + 240)
    m.state, m.dock_yaw = 'DOCK', 0.0
    node.dock(m, 100.0)
    assert m.state == 'DOCK' and m.dock_since == 100.0
    node.dock(m, 100.5)
    assert m.state == 'DOCK'
    node.dock(m, 101.1)
    assert m.state == 'BOARD' and m.door_phase == 'OPENING'
    misaligned = Mission(goal(), time.monotonic() + 240)
    misaligned.state, misaligned.dock_yaw = 'DOCK', 0.5
    node.dock(misaligned, 200.0)
    assert misaligned.dock_since is None                           # yaw outside 0.17 rad
