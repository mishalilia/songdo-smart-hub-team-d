# Author: Maftuna Khalimjonova (12236681)
"""
safety_supervisor (scenario S9).

The ONLY publisher of final velocities (/robot/cmd_vel, /bus/cmd_vel). 20 Hz tick on a
monotonic clock (assignment 16.2, 18.7): stop latch, e-stop input, candidate-command and
pose freshness, crossing permission, obstacle clearance with stopping distance, bus door
interlock and speed caps. Starts latched; /safety/clear_stop unlatches only when the e-stop
is released, critical data is fresh, no hazard is present and motion is zero.
"""
import math
import time

from geometry_msgs.msg import PoseArray, Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from smart_hub import hub_map
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.qos import R1, R10, R100, R1T
from smart_hub_interfaces.msg import (
    CrossingState, HubEvent, QueueState, SafetyState, VehicleStateArray,
)
from smart_hub_interfaces.srv import ClearSafetyStop
from std_msgs.msg import Bool

AGENTS = {'robot': 'robot1', 'bus': 'bus1'}
RADIUS = {'robot': 0.20, 'bus': 0.30, 'car': 0.30, 'obstacle': 0.35, 'pedestrian': 0.20}
STOPPED_MPS = 0.02


def footprint_in_c(x, y, radius):
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    return xmin - radius <= x <= xmax + radius and ymin - radius <= y <= ymax + radius


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('robot_speed_limit', 0.25)
        self.declare_parameter('bus_speed_limit', 0.40)
        self.declare_parameter('max_angular_rad_s', 0.8)
        self.declare_parameter('clearance_m', 0.20)
        self.declare_parameter('reaction_time_sec', 0.10)
        self.declare_parameter('min_deceleration_mps2', 0.5)
        check_parameters(self)

        self.stop_latched = True
        self.latch_reason = 'STARTUP_LATCHED'
        self.estop_asserted = False
        self.candidates = {a: (None, -1e9) for a in AGENTS}
        self.poses = {a: None for a in AGENTS}            # (x, y, yaw, speed, receipt)
        self.others = []                                   # (id, kind, x, y)
        self.pedestrians = ([], -1e9)                      # detected foot-points, receipt
        self.crossing = (None, -1e9)
        self.pedestrian_in_c = (False, -1e9)
        self.door_open = False
        self.inhibit = {a: '' for a in AGENTS}
        self.run_id = ''
        self.tick_count = 0
        self.last_state = None

        self.final_pubs = {
            'robot': self.create_publisher(Twist, '/robot/cmd_vel', R1),
            'bus': self.create_publisher(Twist, '/bus/cmd_vel', R1),
        }
        self.safety_pub = self.create_publisher(SafetyState, '/hub/safety', R1T)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)

        self.create_subscription(
            TwistStamped, '/robot/cmd_vel_candidate', lambda m: self.on_candidate('robot', m), R1)
        self.create_subscription(
            TwistStamped, '/bus/cmd_vel_candidate', lambda m: self.on_candidate('bus', m), R1)
        self.create_subscription(Bool, '/hub/estop', self.on_estop, R1T)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(Bool, '/bus/door_open', self.on_door, R1)
        self.create_subscription(PoseArray, '/sim/pedestrians', self.on_pedestrians, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_service(ClearSafetyStop, '/safety/clear_stop', self.on_clear_stop)

        self.create_timer(0.05, self.tick)

    # ------------------------------------------------------------------ helpers
    def event(self, entity_id, name, reason='', correlation_id=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id = self.run_id
        msg.entity_id = entity_id
        msg.event = name
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events_pub.publish(msg)

    def latch(self, reason):
        if not self.stop_latched:
            self.event('safety_supervisor', 'STOP_LATCHED', reason)
        self.stop_latched = True
        self.latch_reason = reason

    def publish_zero(self):
        for pub in self.final_pubs.values():
            pub.publish(Twist())

    # ------------------------------------------------------------------ inputs
    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.run_id = msg.run_id
        ack_reset(self, self.events_pub, msg)

    def on_candidate(self, agent, msg):
        self.candidates[agent] = (msg.twist, time.monotonic())

    def on_estop(self, msg):
        receipt = time.monotonic()
        self.estop_asserted = bool(msg.data)
        if msg.data:
            was_latched = self.stop_latched
            self.stop_latched, self.latch_reason = True, 'ESTOP'
            self.publish_zero()  # immediate zero; do not wait for the next tick
            if not was_latched:
                self.event('safety_supervisor', 'STOP_LATCHED', 'ESTOP latency_ms=%.2f' % (
                    (time.monotonic() - receipt) * 1000.0))
        # A false message never clears the latch.

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self.poses['robot'] = (p.position.x, p.position.y, yaw,
                               abs(msg.twist.twist.linear.x), time.monotonic())

    def on_vehicles(self, msg):
        now = time.monotonic()
        others = []
        for v in msg.vehicles:
            age = (self.get_clock().now() - Time.from_msg(v.header.stamp)).nanoseconds / 1e9
            if v.vehicle_id == AGENTS['bus'] and v.valid and age <= 1.0:
                # Observation-stamp freshness: a frozen bus pose becomes stale (5, 16.1).
                self.poses['bus'] = (v.pose.x, v.pose.y, v.pose.theta, v.speed_mps, now)
            if v.valid:
                others.append((v.vehicle_id, v.kind, v.pose.x, v.pose.y))
        self.others = others

    def on_crossing(self, msg):
        self.crossing = (msg, time.monotonic())

    def on_queue(self, msg):
        self.pedestrian_in_c = (not msg.valid or msg.crossing_occupied, time.monotonic())

    def on_pedestrians(self, msg):
        self.pedestrians = ([(p.position.x, p.position.y) for p in msg.poses], time.monotonic())

    def on_door(self, msg):
        self.door_open = bool(msg.data)

    # ------------------------------------------------------------------ checks
    def pose_fresh(self, agent, now):
        p = self.poses[agent]
        return p is not None and now - p[4] <= self.get_parameter('pose_timeout_sec').value

    def crossing_reason(self, agent, x, y, yaw, v, now):
        r = RADIUS[agent]
        msg, receipt = self.crossing
        crossing_ok = msg is not None and msg.valid and now - receipt <= 1.0
        owner = crossing_ok and msg.owner_id == AGENTS[agent]
        ped, ped_receipt = self.pedestrian_in_c
        pedestrian = ped or now - ped_receipt > 1.0
        inside = footprint_in_c(x, y, r)
        if inside:
            return 'PEDESTRIAN_IN_C' if pedestrian and v > 0 else ''
        if v <= 0:
            return ''
        look = v * 0.5 + self.get_parameter('clearance_m').value
        if footprint_in_c(x + look * math.cos(yaw), y + look * math.sin(yaw), r) and not owner:
            return 'CROSSING_NOT_GRANTED'
        return ''

    def clearance_reason(self, agent, x, y, yaw, v):
        if v <= 0:
            return ''
        tau = self.get_parameter('reaction_time_sec').value
        a_min = self.get_parameter('min_deceleration_mps2').value
        need = self.get_parameter('clearance_m').value + v * tau + v * v / (2.0 * a_min)
        peds, receipt = self.pedestrians
        nearby = list(self.others)
        if time.monotonic() - receipt <= 1.0:
            nearby += [('pedestrian', 'pedestrian', px, py) for px, py in peds]
        for oid, kind, ox, oy in nearby:
            if oid == AGENTS[agent]:
                continue
            dx, dy = ox - x, oy - y
            dist = math.hypot(dx, dy)
            gap = dist - RADIUS[agent] - RADIUS.get(kind, 0.30)
            ahead = dist > 1e-6 and (dx * math.cos(yaw) + dy * math.sin(yaw)) / dist > 0.7
            if ahead and gap < need:
                return 'CLEARANCE_' + oid
        return ''

    def evaluate(self, agent, now):
        """Return (Twist to publish, reason). Zero Twist whenever any check fails."""
        twist, received = self.candidates[agent]
        if self.stop_latched:
            return Twist(), 'LATCHED_' + self.latch_reason
        if twist is None or now - received > self.get_parameter('command_timeout_sec').value:
            moving = twist is not None and (twist.linear.x != 0.0 or twist.angular.z != 0.0)
            return Twist(), 'CMD_STALE' if moving else ''
        if not self.pose_fresh(agent, now):
            return Twist(), 'POSE_STALE'
        if agent == 'bus' and self.door_open:
            return Twist(), 'DOOR_OPEN'
        limit = self.get_parameter(agent + '_speed_limit').value
        w_max = self.get_parameter('max_angular_rad_s').value
        out = Twist()
        out.linear.x = max(-limit, min(limit, twist.linear.x))
        out.angular.z = max(-w_max, min(w_max, twist.angular.z))
        x, y, yaw, _, _ = self.poses[agent]
        reason = self.crossing_reason(agent, x, y, yaw, out.linear.x, now) or \
            self.clearance_reason(agent, x, y, yaw, out.linear.x)
        if reason:
            out.linear.x = 0.0  # rotation in place stays allowed
        return out, reason

    # ------------------------------------------------------------------ loop
    def tick(self):
        now = time.monotonic()
        for agent, pub in self.final_pubs.items():
            out, reason = self.evaluate(agent, now)
            pub.publish(out)
            if reason != self.inhibit[agent] and not reason.startswith('LATCHED'):
                if reason:
                    self.event(AGENTS[agent], 'SAFETY_INHIBIT', reason)
                elif self.inhibit[agent]:
                    self.event(AGENTS[agent], 'SAFETY_CLEAR', self.inhibit[agent])
            self.inhibit[agent] = reason
        self.tick_count += 1
        state = SafetyState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        state.stop_latched = self.stop_latched
        state.motion_permitted = not self.stop_latched and not self.estop_asserted
        state.reason = self.latch_reason if self.stop_latched else ','.join(
            '%s:%s' % (AGENTS[a], r) for a, r in self.inhibit.items() if r)
        key = (state.stop_latched, state.motion_permitted, state.reason)
        if self.tick_count % 2 == 0 or key != self.last_state:   # 10 Hz + on change
            self.safety_pub.publish(state)
            self.last_state = key

    def on_clear_stop(self, request, response):
        now = time.monotonic()
        problems = []
        if self.estop_asserted:
            problems.append('ESTOP_ASSERTED')
        for agent in AGENTS:
            if not self.pose_fresh(agent, now):
                problems.append('STALE_' + agent.upper())
                continue
            x, y, yaw, speed, _ = self.poses[agent]
            if speed > STOPPED_MPS:
                problems.append('MOVING_' + agent.upper())
            for oid, kind, ox, oy in self.others:
                if oid != AGENTS[agent] and math.hypot(ox - x, oy - y) < \
                        RADIUS[agent] + RADIUS.get(kind, 0.30):
                    problems.append('CONTACT_%s_%s' % (agent.upper(), oid))
        if problems:
            response.success = False
            response.reason = ','.join(problems)
            self.event('safety_supervisor', 'CLEAR_STOP_REJECTED', response.reason,
                       request.operator_id)
            return response
        self.stop_latched = False
        self.latch_reason = ''
        response.success = True
        response.reason = 'OK'
        self.event('safety_supervisor', 'STOP_CLEARED', request.acknowledgement,
                   request.operator_id)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
