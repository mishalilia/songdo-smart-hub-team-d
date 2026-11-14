# Author: Maftuna Khalimjonova (12236681)
"""Safety supervisor tests (16.2, 18.7, T16): latch, watchdogs, crossing, clearance."""
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.duration import Duration
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


def drive(n, agent='robot', v=0.2, pose=(3.0, 6.0, 0.0), w=0.0):
    now = time.monotonic()
    twist = Twist()
    twist.linear.x = v
    twist.angular.z = w
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
    owned.header.stamp = node.get_clock().now().to_msg()
    node.crossing.put(owned, now)
    node.pedestrian_in_c = (False, now)
    assert node.evaluate('robot', now)[1] == ''


def test_retained_crossing_grant_expires(node):
    """A retained OWNED sample is not permission once invalid, old or mis-stamped."""
    node.stop_latched = False
    now = drive(node, pose=(5.0, 5.2, -1.5708))
    node.pedestrian_in_c = (False, now)
    ros_now = node.get_clock().now()

    def grant(stamp_offset_sec=0.0, valid=True, receipt=now):
        msg = CrossingState(owner_id='robot1', state='OWNED', valid=valid)
        msg.header.stamp = (ros_now + Duration(seconds=stamp_offset_sec)).to_msg()
        node.crossing.put(msg, receipt)
        return node.evaluate('robot', now)[1]

    assert grant() == ''
    assert grant(valid=False) == 'CROSSING_NOT_GRANTED'
    assert grant(receipt=now - 1.5) == 'CROSSING_NOT_GRANTED'        # local receipt age
    assert grant(stamp_offset_sec=-1.5) == 'CROSSING_NOT_GRANTED'    # observation age
    assert grant(stamp_offset_sec=5.0) == 'CROSSING_NOT_GRANTED'     # future stamp
    node.crossing.put(CrossingState(owner_id='robot1', valid=True), now)
    assert node.evaluate('robot', now)[1] == 'CROSSING_NOT_GRANTED'  # no stamp at all


def test_clearance_to_obstacle_ahead(node):
    node.stop_latched = False
    now = drive(node, pose=(6.3, 2.0, 0.0))
    node.others = [('O', 'obstacle', 7.0, 2.0)]
    out, reason = node.evaluate('robot', now)
    assert out.linear.x == 0.0 and reason == 'CLEARANCE_O'


def test_turning_into_an_object_outside_the_forward_cone(node):
    """Swept arc: the object is abeam (outside the cone) but the turn drives into it."""
    node.stop_latched = False
    node.others = [('car7', 'car', 3.1, 6.6)]             # 0.6 m to the left, gap 0.10 m
    now = drive(node, v=0.25, w=0.0)
    assert node.evaluate('robot', now)[1] == ''           # straight on passes it
    now = drive(node, v=0.25, w=0.8)
    out, reason = node.evaluate('robot', now)
    assert out.linear.x == 0.0 and reason == 'SWEPT_CLEARANCE_car7'
    assert out.angular.z == pytest.approx(0.8)            # rotation in place stays allowed
    now = drive(node, v=0.25, w=-0.8)
    assert node.evaluate('robot', now)[1] == ''           # turning away is not a hazard


def test_lateral_swept_hazard_and_free_passage(node):
    node.stop_latched = False
    # Pedestrian 46 degrees off the heading: the forward cone misses it, the swept
    # footprint (0.20 + 0.20 m radii, 0.05 m side margin) does not.
    node.pedestrians = ([(3.36, 6.37)], time.monotonic())
    now = drive(node, v=0.25)
    assert node.evaluate('robot', now)[1] == 'SWEPT_CLEARANCE_pedestrian'
    # Queue passengers stand 0.6 m beside the lane: the robot passes them.
    node.pedestrians = ([(3.2, 6.6), (3.4, 5.4)], time.monotonic())
    assert node.evaluate('robot', drive(node, v=0.25))[1] == ''
    # Already past (behind the robot): moving away, no inhibit.
    node.pedestrians = ([(2.8, 6.3)], time.monotonic())
    assert node.evaluate('robot', drive(node, v=0.25))[1] == ''


def test_standstill_is_measured_after_the_zero_command(node):
    """E-stop: zero command and measured standstill are separate observations."""
    events = []
    node.event = lambda entity, name, reason='', correlation_id='': events.append(
        (entity, name, reason))
    node.stop_latched = False
    node.poses['bus'] = (1.0, 2.0, 0.0, 0.0, time.monotonic())
    node.pose_stamp['bus'] = node.ros_now()
    drive(node)
    node.poses['robot'] = (3.0, 6.0, 0.0, 0.2, time.monotonic())     # still rolling
    node.pose_stamp['robot'] = node.ros_now()
    node.on_estop(Bool(data=True))
    node.tick()
    names = [(e, n) for e, n, _ in events]
    assert ('safety_supervisor', 'STOP_LATCHED') in names
    assert ('robot1', 'STANDSTILL_MEASURED') not in names   # pose predates the stop
    node.poses['robot'] = (3.02, 6.0, 0.0, 0.2, time.monotonic())
    node.pose_stamp['robot'] = node.ros_now()
    node.tick()
    assert ('robot1', 'STANDSTILL_MEASURED') not in [(e, n) for e, n, _ in events]
    node.poses['robot'] = (3.03, 6.0, 0.0, 0.0, time.monotonic())
    node.pose_stamp['robot'] = node.ros_now()
    node.tick()
    reason = [r for e, n, r in events if (e, n) == ('robot1', 'STANDSTILL_MEASURED')]
    assert len(reason) == 1 and reason[0].startswith('cause=ESTOP latency_ms=')
    assert 'zero_cmd_ms=' in reason[0] and 'observer_delay_ms=' in reason[0]
