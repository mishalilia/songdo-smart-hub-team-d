# Author: Vadim Lifrov (12240192)
"""
bus_manager (scenario S3).

DockBus action server (/bus/dock), assignment 10 and 18.6:
IDLE -> APPROACH -> (WAIT) -> DOCK -> BOARD -> COMPLETE; SAFE_STOP on cancel, safety latch,
stale pose or timeout. Plans with /route/compute, acquires /crossing/request before its
footprint enters C (renews while inside, releases after clearance), docks when within
0.20 m / 0.17 rad of bay B for 1 s with speed < 0.02 m/s, then opens the door
(/bus/door_request, measured /bus/door_open), holds zero motion for the boarding dwell and
closes the door before completing. Departure is a separate transition after the goal
completes: the bus returns to depot A on the one-way loop B-U-E-J-Cs-A (same crossing
rules) and becomes IDLE (available) again. Publishes /hub/bus_arrival (state, ETA from
remaining route length and smoothed speed) at 2 Hz and a candidate velocity at 20 Hz.
"""
import math
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from smart_hub import hub_map
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.path_follower import follow, route_length, wrap
from smart_hub.qos import R1, R10, R100, R1T
from smart_hub_interfaces.action import DockBus
from smart_hub_interfaces.msg import BusArrival, HubEvent, SafetyState, VehicleStateArray
from smart_hub_interfaces.srv import ComputeRoute, RequestCrossing
from std_msgs.msg import Bool

BUS_ID = 'bus1'
RADIUS = 0.30
WAYPOINT_TOLERANCE_M = 0.10
STOPPED_MPS = 0.02
SERVICE_TIMEOUT_SEC = 1.0
SERVICE_RETRIES = 2
RETRY_SEC = 0.5
RENEW_PERIOD_SEC = 0.5
DOOR_TIMEOUT_SEC = 3.0
DOOR_RESEND_SEC = 1.0
DOCK_SLOW_MPS = 0.12
DOCK_SLOW_RADIUS_M = 1.0
ETA_CHECK_DISTANCE_M = 5.0
DEPOT = 'A'
DEPART_EXCLUDED = ['B-Cn']          # one-way loop: depart via U-E-J-Cs


def segment_crosses_c(p, q):
    return hub_map.segment_intersects_rect(p, q, hub_map.CROSSING_C, inflate=RADIUS)


def footprint_in_c(x, y):
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    return xmin - RADIUS <= x <= xmax + RADIUS and ymin - RADIUS <= y <= ymax + RADIUS


class Mission:

    def __init__(self, goal, deadline):
        self.bay = goal.bay_id or hub_map.BUS_BAY
        self.deadline = deadline
        self.goal_handle = None
        self.goal_id = ''
        self.done = threading.Event()
        self.terminal = None              # (status, final_state, reason)
        self.state = 'APPROACH'
        self.route = []
        self.i = 0
        self.edge_checked = False
        self.planned = False
        self.retry_at = 0.0
        self.dock_since = None
        self.dock_yaw = 0.0
        self.door_phase = ''
        self.door_since = 0.0
        self.door_cmd_at = 0.0
        self.cancel_since = None
        self.depart = False
        self.eta_checkpoint = None        # (time, eta) when 5 m remained (assignment 10.3)
        self.eta_invalid_sec = 0.0
        self.last_tick = None


class BusManager(Node):

    def __init__(self):
        super().__init__('bus_manager')
        self.declare_parameter('bus_speed_limit', 0.40)
        self.declare_parameter('max_angular_rad_s', 0.8)
        self.declare_parameter('pose_tolerance_m', 0.20)
        self.declare_parameter('yaw_tolerance_rad', 0.17)
        self.declare_parameter('boarding_dwell_sec', 8.0)
        check_parameters(self)
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        self.mission = None
        self.departure = None             # Mission returning to the depot after a goal
        self.pose = None                  # (x, y, yaw, speed, receipt)
        self.smoothed_speed = 0.0
        self.safety = (None, -1e9)
        self.door_open = False
        self.others = []
        self.crossing_token = ''
        self.last_renew = 0.0
        self.pending = {}
        self.last_state = 'IDLE'
        self.run_id = ''

        self.arrival_pub = self.create_publisher(BusArrival, '/hub/bus_arrival', R10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/bus/cmd_vel_candidate', R1)
        self.door_req_pub = self.create_publisher(Bool, '/bus/door_request', R1)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)

        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10,
                                 callback_group=self.cb)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T,
                                 callback_group=self.cb)
        self.create_subscription(Bool, '/bus/door_open', self.on_door, R1,
                                 callback_group=self.cb)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100,
                                 callback_group=self.cb)

        self.route_client = self.create_client(
            ComputeRoute, '/route/compute', callback_group=self.cb)
        self.crossing_client = self.create_client(
            RequestCrossing, '/crossing/request', callback_group=self.cb)

        self.action_server = ActionServer(
            self, DockBus, '/bus/dock',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )
        self.create_timer(0.05, self.tick, callback_group=self.cb)
        self.create_timer(0.5, self.publish_arrival, callback_group=self.cb)

    # ------------------------------------------------------------------ helpers
    def event(self, name, reason='', correlation_id=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id = self.run_id
        msg.entity_id = BUS_ID
        msg.event = name
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events_pub.publish(msg)

    def command(self, v=0.0, w=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bus/base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def door(self, m, open_, now):
        self.door_req_pub.publish(Bool(data=open_))
        m.door_cmd_at = now

    def pose_fresh(self, now):
        return self.pose is not None and now - self.pose[4] <= 1.0

    def safety_latched(self, now):
        msg, receipt = self.safety
        return msg is None or now - receipt > 1.0 or msg.stop_latched

    def nearest_node(self):
        nodes = {n for e in hub_map.VEHICLE_EDGES for n in e}
        return min(nodes, key=lambda n: math.dist(self.pose[:2], hub_map.NODES[n]))

    # ------------------------------------------------------------------ inputs
    def on_vehicles(self, msg):
        now = time.monotonic()
        others = []
        for v in msg.vehicles:
            if v.vehicle_id == BUS_ID:
                # Freshness uses the observation stamp: a frozen source stays stale (5).
                age = (self.get_clock().now() - Time.from_msg(v.header.stamp)).nanoseconds / 1e9
                if v.valid and age <= 1.0:
                    self.pose = (v.pose.x, v.pose.y, v.pose.theta, v.speed_mps, now)
                    self.smoothed_speed = 0.8 * self.smoothed_speed + 0.2 * v.speed_mps
            elif v.valid:
                others.append((v.vehicle_id, v.pose.x, v.pose.y))
        self.others = others

    def on_safety(self, msg):
        self.safety = (msg, time.monotonic())

    def on_door(self, msg):
        self.door_open = bool(msg.data)

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.run_id = msg.run_id
            with self.lock:
                if self.mission is not None:
                    self.finish(self.mission, 'abort', 'IDLE', 'SCENARIO_RESET')
                self.departure = None
                self.crossing_token = ''
                self.last_state = 'IDLE'
        ack_reset(self, self.events_pub, msg)

    # ------------------------------------------------------------------ action lifecycle
    def on_goal(self, goal):
        now = time.monotonic()
        reason = ''
        if goal.bus_id != BUS_ID:
            reason = 'INVALID_BUS_ID'
        elif (goal.bay_id or hub_map.BUS_BAY) != hub_map.BUS_BAY:
            reason = 'INVALID_BAY'
        elif goal.max_duration_sec <= 0.0:
            reason = 'INVALID_DEADLINE'
        elif not self.pose_fresh(now):
            reason = 'STALE_BUS_POSE'
        elif self.safety_latched(now):
            reason = 'UNSAFE_STARTUP'
        with self.lock:
            if not reason and (self.mission is not None or self.departure is not None):
                reason = 'BUSY'
            if reason:
                self.event('GOAL_REJECTED', 'dock ' + reason)
                return GoalResponse.REJECT
            self.mission = Mission(goal, now + goal.max_duration_sec)
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        with self.lock:
            m = self.mission
            if m is None or m.goal_handle is not None:
                goal_handle.abort()
                return DockBus.Result(success=False, final_state='IDLE', reason='NOT_ACTIVE')
            m.goal_handle = goal_handle
            m.goal_id = ''.join('%02x' % b for b in bytes(goal_handle.goal_id.uuid))
        self.event('GOAL_ACCEPTED', 'dock', m.goal_id)
        while not m.done.wait(0.5):              # feedback at 2 Hz
            goal_handle.publish_feedback(self.feedback(m))
        status, final_state, reason = m.terminal
        if status == 'succeed':
            goal_handle.succeed()
        elif status == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.event('GOAL_' + status.upper(), reason, m.goal_id)
        return DockBus.Result(success=status == 'succeed', final_state=final_state,
                              reason=reason)

    def finish(self, m, status, final_state, reason):
        if m.terminal is not None:
            return
        self.command()
        m.terminal = (status, final_state, reason)
        m.state = final_state
        if self.mission is m:
            self.mission = None
        self.last_state = final_state
        if status == 'succeed':
            d = Mission(SimpleNamespace(bay_id=DEPOT), math.inf)
            d.depart, d.goal_id = True, m.goal_id
            self.departure, self.last_state = d, 'DEPART'
        m.done.set()

    def remaining(self, m):
        if self.pose is None or not m.route or m.i >= len(m.route):
            return 0.0
        return math.dist(self.pose[:2], hub_map.NODES[m.route[m.i]]) + \
            route_length(m.route[m.i:], hub_map.NODES)

    def eta(self, m):
        """
        (eta_sec, valid) from the remaining route length (10.2).

        Cruise at the smoothed speed, the last DOCK_SLOW_RADIUS_M at the docking speed.
        Invalid (eta = -1) while the bus is not moving or waits without an estimate.
        """
        if m is None:
            return -1.0, False
        if m.state in ('DOCK', 'BOARD'):
            return 0.0, True
        if m.state != 'APPROACH' or self.smoothed_speed < 0.05:
            return -1.0, False
        rest = self.remaining(m)
        slow = min(rest, DOCK_SLOW_RADIUS_M)
        cruise = max(self.smoothed_speed, DOCK_SLOW_MPS)
        return (rest - slow) / cruise + slow / DOCK_SLOW_MPS, True

    def track_eta(self, m, now):
        """Record the ETA at 5 m remaining and the time the ETA was invalid."""
        if m.depart or not m.planned:
            return
        eta, valid = self.eta(m)
        if m.last_tick is not None and not valid and m.state in ('APPROACH', 'WAIT'):
            m.eta_invalid_sec += now - m.last_tick
        m.last_tick = now
        if m.eta_checkpoint is None and valid and m.state == 'APPROACH' and \
                self.remaining(m) <= ETA_CHECK_DISTANCE_M:
            m.eta_checkpoint = (now, eta)
            self.event('ETA_AT_5M', 'eta_sec=%.2f' % eta, m.goal_id)

    def feedback(self, m):
        fb = DockBus.Feedback()
        with self.lock:
            fb.phase = m.state
            fb.distance_remaining_m = float(self.remaining(m))
            eta, valid = self.eta(m)
            fb.eta_sec = float(eta if valid else -1.0)
        return fb

    def publish_arrival(self):
        with self.lock:
            m = self.mission
            eta, valid = self.eta(m)
            state = m.state if m is not None else self.last_state
            if self.departure is not None:
                state = 'DEPART'
        msg = BusArrival()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.bus_id = BUS_ID
        msg.state = state
        msg.eta_sec = float(eta) if valid else -1.0
        msg.eta_valid = valid
        msg.door_open = self.door_open
        self.arrival_pub.publish(msg)

    # ------------------------------------------------------------------ service calls
    def call(self, name, client, request, now):
        attempt = self.pending.get(name, (None, 0.0, 0))[2]
        future = client.call_async(request) if client.service_is_ready() else None
        self.pending[name] = (future, now, attempt + 1)

    def poll(self, name, now):
        future, sent, attempt = self.pending[name]
        if future is not None and future.done():
            del self.pending[name]
            try:
                return 'done', future.result()
            except Exception as exc:
                return 'failed', str(exc)
        if now - sent > SERVICE_TIMEOUT_SEC:
            if future is not None:
                future.cancel()
            if attempt > SERVICE_RETRIES:
                del self.pending[name]
                return 'failed', 'TIMEOUT'
            return 'timeout', None
        return 'wait', None

    def request_crossing(self, operation, now):
        req = RequestCrossing.Request()
        req.requester_id = BUS_ID
        req.operation = operation
        req.token = self.crossing_token
        self.call('crossing_' + operation, self.crossing_client, req, now)

    def service_crossing_lease(self, now):
        for op in ('RENEW', 'RELEASE'):
            name = 'crossing_' + op
            if name not in self.pending:
                continue
            status, res = self.poll(name, now)
            if status == 'done' and op == 'RELEASE' and res.granted:
                self.event('CROSSING_RELEASED', '', self.crossing_token)
                self.crossing_token = ''
            elif status == 'done' and op == 'RENEW' and not res.granted:
                self.event('CROSSING_RENEW_LOST', res.reason)
                self.crossing_token = ''
            elif status in ('timeout', 'failed'):
                self.pending.pop(name, None)
        if not self.crossing_token or self.pose is None:
            return
        m = self.mission or self.departure
        x, y = self.pose[0], self.pose[1]
        heading_into_c = m is not None and bool(m.route) and m.i < len(m.route) and \
            segment_crosses_c((x, y), hub_map.NODES[m.route[m.i]])
        if not footprint_in_c(x, y) and not heading_into_c:
            if 'crossing_RELEASE' not in self.pending:
                self.request_crossing('RELEASE', now)
        elif now - self.last_renew >= RENEW_PERIOD_SEC and 'crossing_RENEW' not in self.pending:
            self.last_renew = now
            self.request_crossing('RENEW', now)

    # ------------------------------------------------------------------ control loop
    def tick(self):
        now = time.monotonic()
        with self.lock:
            self.service_crossing_lease(now)
            m = self.mission
            if m is None and self.departure is not None:
                if self.pose_fresh(now) and not self.safety_latched(now):
                    self.approach(self.departure, now)
                else:
                    self.command()
                return
            if m is None or m.goal_handle is None or m.terminal is not None:
                return
            if m.goal_handle.is_cancel_requested and m.cancel_since is None:
                m.cancel_since = now
                self.event('CANCEL_ACCEPTED', m.state, m.goal_id)
            if m.cancel_since is not None:
                self.command()
                if self.door_open and now - m.door_cmd_at > DOOR_RESEND_SEC:
                    self.door(m, False, now)
                if self.pose_fresh(now) and self.pose[3] < STOPPED_MPS:
                    self.finish(m, 'canceled', 'SAFE_STOP', 'CANCELED latency_ms=%.1f' % (
                        (now - m.cancel_since) * 1000.0))
                elif now - m.cancel_since > 2.0:
                    self.finish(m, 'abort', 'SAFE_STOP', 'STOP_NOT_CONFIRMED')
                return
            if not self.pose_fresh(now):
                self.finish(m, 'abort', 'SAFE_STOP', 'STALE_BUS_POSE')
            elif self.safety_latched(now):
                self.finish(m, 'abort', 'SAFE_STOP', 'SAFE_STOP')
            elif now > m.deadline:
                self.finish(m, 'abort', 'SAFE_STOP', 'TIMEOUT')
            elif m.state in ('APPROACH', 'WAIT'):
                self.track_eta(m, now)
                self.approach(m, now)
            elif m.state == 'DOCK':
                self.dock(m, now)
            elif m.state == 'BOARD':
                self.board(m, now)

    def plan(self, m, now):
        if 'route' not in self.pending:
            req = ComputeRoute.Request()
            req.start_node = self.nearest_node()
            req.goal_node = m.bay
            req.agent_type = 'bus'
            req.excluded_edges = DEPART_EXCLUDED if m.depart else []
            self.call('route', self.route_client, req, now)
            return
        status, res = self.poll('route', now)
        if status == 'timeout':
            self.pending.pop('route', None)
        elif status == 'failed' and m.depart:
            m.state, m.retry_at = 'WAIT', now + 2.0
        elif status == 'failed':
            self.finish(m, 'abort', 'SAFE_STOP', 'PLANNER_UNAVAILABLE')
        elif status == 'done' and not res.success:
            m.state, m.retry_at = 'WAIT', now + 2.0
            self.event('ROUTE_UNAVAILABLE', res.reason, m.goal_id)
        elif status == 'done':
            m.route, m.i, m.planned, m.state = list(res.node_ids), 0, True, 'APPROACH'
            if len(m.route) > 1:
                (ax, ay), (bx, by) = hub_map.NODES[m.route[-2]], hub_map.NODES[m.route[-1]]
                m.dock_yaw = math.atan2(by - ay, bx - ax)
            else:
                m.dock_yaw = self.pose[2]
            self.event('ROUTE', '-'.join(m.route), m.goal_id)

    def bay_clear(self):
        bx, by = hub_map.NODES[hub_map.BUS_BAY]
        return all(math.hypot(x - bx, y - by) > RADIUS + 0.4 for _, x, y in self.others)

    def acquire_crossing(self, m, now):
        """Return True once the token is held; stop outside C while waiting."""
        if 'crossing_ACQUIRE' not in self.pending:
            self.request_crossing('ACQUIRE', now)
            return False
        status, res = self.poll('crossing_ACQUIRE', now)
        if status == 'done' and res.granted:
            self.crossing_token, self.last_renew = res.token, now
            self.event('CROSSING_ACQUIRED', '', res.token)
            return True
        if status in ('done', 'failed'):
            m.state, m.retry_at = 'WAIT', now + RETRY_SEC
        elif status == 'timeout':
            self.pending.pop('crossing_ACQUIRE', None)
        return False

    def approach(self, m, now):
        self.command()
        if now < m.retry_at:
            return
        if not m.planned:
            self.plan(m, now)
            return
        x, y, yaw = self.pose[0], self.pose[1], self.pose[2]
        target = m.route[m.i]
        last = m.i == len(m.route) - 1
        if not m.edge_checked:
            if last and not m.depart and not self.bay_clear():
                m.state, m.retry_at = 'WAIT', now + RETRY_SEC
                return
            needs_c = segment_crosses_c((x, y), hub_map.NODES[target]) and \
                not footprint_in_c(x, y)
            if needs_c and not self.crossing_token and not self.acquire_crossing(m, now):
                return
            m.edge_checked, m.state = True, 'APPROACH'
        tx, ty = hub_map.NODES[target]
        v_max = self.get_parameter('bus_speed_limit').value
        if last and math.hypot(tx - x, ty - y) < DOCK_SLOW_RADIUS_M:
            v_max = DOCK_SLOW_MPS                                   # final approach
        v, w, dist = follow(x, y, yaw, tx, ty, v_max,
                            self.get_parameter('max_angular_rad_s').value, final=last)
        tolerance = self.get_parameter('pose_tolerance_m').value * 0.5 if last \
            else WAYPOINT_TOLERANCE_M
        if dist > tolerance:
            self.command(v, w)
            return
        m.i += 1
        m.edge_checked = False
        if not last:
            self.approach(m, now)
            return
        if m.depart:
            self.command()
            self.departure, self.last_state = None, 'IDLE'
            self.event('DEPOT_ARRIVED', 'route %s' % '-'.join(m.route), m.goal_id)
            return
        m.state, m.dock_since = 'DOCK', None
        reason = 'at bay %s invalid_eta_s=%.1f' % (m.bay, m.eta_invalid_sec)
        if m.eta_checkpoint is not None:
            t5, eta5 = m.eta_checkpoint
            reason += ' eta_err_s=%.2f' % ((now - t5) - eta5)   # actual - predicted
        self.event('DOCKING', reason, m.goal_id)

    def dock(self, m, now):
        x, y, yaw, speed, _ = self.pose
        bx, by = hub_map.NODES[m.bay]
        pos_err = math.hypot(bx - x, by - y)
        yaw_err = wrap(m.dock_yaw - yaw)
        if pos_err > self.get_parameter('pose_tolerance_m').value:
            m.route, m.i, m.state, m.edge_checked = [m.bay], 0, 'APPROACH', True
            return
        if abs(yaw_err) > self.get_parameter('yaw_tolerance_rad').value:
            self.command(0.0, max(-0.4, min(0.4, 1.5 * yaw_err)))   # align in place
            m.dock_since = None
            return
        self.command()
        if speed >= STOPPED_MPS:
            m.dock_since = None
        elif m.dock_since is None:
            m.dock_since = now
        elif now - m.dock_since >= 1.0:
            self.event('DOCKED', 'pos_err_m=%.3f yaw_err_rad=%.3f' % (pos_err, yaw_err),
                       m.goal_id)
            m.state, m.door_phase, m.door_since = 'BOARD', 'OPENING', now
            self.door(m, True, now)

    def board(self, m, now):
        self.command()                                              # zero while doors move
        dwell = self.get_parameter('boarding_dwell_sec').value
        if m.door_phase == 'OPENING':
            if self.door_open:
                m.door_phase, m.door_since = 'OPEN', now
                self.event('DOOR_OPENED', '', m.goal_id)
            elif now - m.door_since > DOOR_TIMEOUT_SEC:
                self.finish(m, 'abort', 'SAFE_STOP', 'DOOR_OPEN_FAILED')
            elif now - m.door_cmd_at > DOOR_RESEND_SEC:
                self.door(m, True, now)
        elif m.door_phase == 'OPEN' and now - m.door_since >= dwell:
            m.door_phase, m.door_since = 'CLOSING', now
            self.door(m, False, now)
        elif m.door_phase == 'CLOSING':
            if not self.door_open:
                self.event('DOOR_CLOSED', '', m.goal_id)
                self.finish(m, 'succeed', 'COMPLETE', 'DOCKED_AND_BOARDED')
            elif now - m.door_since > DOOR_TIMEOUT_SEC:
                self.finish(m, 'abort', 'SAFE_STOP', 'DOOR_CLOSE_FAILED')
            elif now - m.door_cmd_at > DOOR_RESEND_SEC:
                self.door(m, False, now)


def main(args=None):
    rclpy.init(args=args)
    node = BusManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
