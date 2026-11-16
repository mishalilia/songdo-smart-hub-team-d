# Author: Maftuna Khalimjonova (12236681)
"""Safety supervisor tests (16.2, 18.7, T16): latch, watchdogs, crossing, clearance."""
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from smart_hub.safety_supervisor import SafetySupervisor
from smart_hub_interfaces.msg import CrossingState
from smart_hub_interfaces.srv import ClearSafetyStop
from std_msgs.msg import Bool


@pytest.fixture
def node():
    rclpy.init()
    n = SafetySupervisor()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def drive(n, agent='robot', v=0.2, pose=(3.0, 6.0, 0.0)):
    now = time.monotonic()
    twist = Twist()
    twist.linear.x = v
    n.candidates[agent] = (twist, now)
    n.poses[agent] = (*pose, 0.0, now)
    return now


def test_starts_latched_and_zero(node):
    now = drive(node)
    out, reason = node.evaluate('robot', now)
    assert out.linear.x == 0.0 and reason == 'LATCHED_STARTUP_LATCHED'


def test_clear_then_motion_clamped(node):
    node.poses['bus'] = (1.0, 2.0, 0.0, 0.0, time.monotonic())
    drive(node)
    res = node.on_clear_stop(ClearSafetyStop.Request(operator_id='t'), ClearSafetyStop.Response())
    assert res.success
    now = drive(node, v=1.0)
    out, reason = node.evaluate('robot', now)
    assert out.linear.x == pytest.approx(0.25) and reason == ''      # speed cap


def test_estop_latches_and_false_never_clears(node):
    node.stop_latched = False
    node.on_estop(Bool(data=True))
    node.on_estop(Bool(data=False))
    assert node.stop_latched and node.latch_reason == 'ESTOP'


def test_stale_command_and_pose_give_zero(node):
    node.stop_latched = False
    drive(node)
    node.candidates['robot'] = (node.candidates['robot'][0], time.monotonic() - 1.0)
    assert node.evaluate('robot', time.monotonic())[1] == 'CMD_STALE'
    drive(node)
    node.poses['robot'] = (3.0, 6.0, 0.0, 0.0, time.monotonic() - 2.0)
    assert node.evaluate('robot', time.monotonic())[1] == 'POSE_STALE'


def test_no_crossing_entry_without_ownership(node):
    node.stop_latched = False
    now = drive(node, pose=(5.0, 5.2, -1.5708))           # at W-Cn, heading into C
    out, reason = node.evaluate('robot', now)
    assert out.linear.x == 0.0 and reason == 'CROSSING_NOT_GRANTED'
    owned = CrossingState(owner_id='robot1', state='OWNED', valid=True)
    node.crossing = (owned, now)
    node.pedestrian_in_c = (False, now)
    assert node.evaluate('robot', now)[1] == ''


def test_clearance_to_obstacle_ahead(node):
    node.stop_latched = False
    now = drive(node, pose=(6.3, 2.0, 0.0))
    node.others = [('O', 'obstacle', 7.0, 2.0)]
    out, reason = node.evaluate('robot', now)
    assert out.linear.x == 0.0 and reason == 'CLEARANCE_O'
