# Author: Timur Klechshev (12235650)
"""
robot_manager (scenario S7).

Action servers /robot/escort (EscortPassenger) and /robot/navigate_to (NavigateTo);
/robot/confirm_ready (std_srvs/Trigger). Short 20 Hz state-machine ticks (no blocking
loops): TO_PICKUP -> WAIT_READY -> ESCORT -> ARRIVED; PLANNING, WAIT_CROSSING and
STOPPING pause motion. Routes come from /route/compute and crossing ownership from
/crossing/request. Before and while traversing an edge the robot checks /traffic/edges and
replans around a blocked edge from the last safe node. See assignment 7.2, 14, 18.5.
"""
import math
import threading
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from smart_hub import hub_map
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.path_follower import follow, route_length
from smart_hub.qos import R1, R10, R100, R1T
from smart_hub.route_engine import DEFAULT_SPEED_MPS, edge_cost, plan
from smart_hub_interfaces.action import EscortPassenger, NavigateTo
from smart_hub_interfaces.msg import EdgeStateArray, HubEvent, SafetyState
from smart_hub_interfaces.srv import ComputeRoute, RequestCrossing
from std_srvs.srv import Trigger

ROBOT_ID = 'robot1'
RADIUS = 0.20
WAYPOINT_TOLERANCE_M = 0.08
STOPPED_MPS = 0.02
SERVICE_TIMEOUT_SEC = 1.0
SERVICE_RETRIES = 2
CROSSING_RETRY_SEC = 0.5
RENEW_PERIOD_SEC = 0.5
REPLAN_WAIT_SEC = 2.0
EDGES_STALE_SEC = 1.5


def segment_crosses_c(p, q):
    return hub_map.segment_intersects_rect(p, q, hub_map.CROSSING_C, inflate=RADIUS)


def footprint_in_c(x, y):
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    return xmin - RADIUS <= x <= xmax + RADIUS and ymin - RADIUS <= y <= ymax + RADIUS


class Mission:

    def __init__(self, kind, request, deadline):
        self.kind = kind                                  # escort | navigate
        self.request_id = request.request_id
        self.deadline = deadline
        self.goal_handle = None
        self.done = threading.Event()
        self.terminal = None                              # (status, reason)
        if kind == 'escort':
            self.legs = [('TO_PICKUP', request.pickup_node),
                         ('ESCORT', request.destination_node)]
        else:
            self.legs = [('NAVIGATE', request.goal_node)]
        self.leg = 0
        self.state = 'PLANNING'
        self.route = []
        self.i = 0
        self.current_node = ''
        self.edge_checked = False
        self.excluded = set()
        self.ready = False
        self.wait_since = 0.0
        self.arrive_since = None
        self.cancel_since = None
        self.retry_at = 0.0
        self.travelled = 0.0
        self.replans = 0
        self.last_replan_check = -math.inf

    @property
    def phase(self):
        return self.legs[self.leg][0]

    @property
    def leg_goal(self):
        return self.legs[self.leg][1]

    @property
    def final_leg(self):
        return self.leg == len(self.legs) - 1


class RobotManager(Node):

    def __init__(self):
        super().__init__('robot_manager')
        self.declare_parameter('robot_speed_limit', 0.25)
        self.declare_parameter('max_angular_rad_s', 0.8)
        self.declare_parameter('arrival_tolerance_m', 0.20)
        self.declare_parameter('ready_timeout_sec', 20.0)
        self.declare_parameter('replan_improvement_ratio', 0.15)
        self.declare_parameter('replan_period_sec', 2.0)
        check_parameters(self)
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        self.mission = None
        self.pose = None                     # (x, y, yaw, v, receipt)
        self.safety = (None, -1e9)
        self.edges = ({}, -1e9)
        self.crossing_token = ''
        self.last_renew = 0.0
        self.pending = {}                    # name -> (future, sent_at, attempt)
        self.run_id = ''

        self.cmd_pub = self.create_publisher(TwistStamped, '/robot/cmd_vel_candidate', R1)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10,
                                 callback_group=self.cb)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T,
                                 callback_group=self.cb)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10,
                                 callback_group=self.cb)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100,
                                 callback_group=self.cb)

        self.route_client = self.create_client(
            ComputeRoute, '/route/compute', callback_group=self.cb)
        self.crossing_client = self.create_client(
            RequestCrossing, '/crossing/request', callback_group=self.cb)
        self.create_service(Trigger, '/robot/confirm_ready', self.on_confirm_ready,
                            callback_group=self.cb)

        self.escort_server = ActionServer(
            self, EscortPassenger, '/robot/escort',
            execute_callback=self.execute,
            goal_callback=lambda g: self.on_goal('escort', g),
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )
        self.navigate_server = ActionServer(
            self, NavigateTo, '/robot/navigate_to',
            execute_callback=self.execute,
            goal_callback=lambda g: self.on_goal('navigate', g),
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )
        self.create_timer(0.05, self.tick, callback_group=self.cb)

    # ------------------------------------------------------------------ helpers
    def event(self, name, reason='', correlation_id=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id = self.run_id
        msg.entity_id = ROBOT_ID
        msg.event = name
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events_pub.publish(msg)

    def command(self, v=0.0, w=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'robot/base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def odom_fresh(self, now):
        return self.pose is not None and now - self.pose[4] <= 1.0

    def safety_latched(self, now):
        msg, receipt = self.safety
        return msg is None or now - receipt > 1.0 or msg.stop_latched

    def nearest_node(self):
        x, y = self.pose[0], self.pose[1]
        return min(hub_map.NODES, key=lambda n: math.dist((x, y), hub_map.NODES[n]))

    @staticmethod
    def robot_nodes():
        return {n for e in hub_map.ROBOT_EDGES for n in e}

    # ------------------------------------------------------------------ inputs
    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        with self.lock:
            if self.pose is not None and self.mission is not None:
                self.mission.travelled += math.dist(self.pose[:2], (p.position.x, p.position.y))
            self.pose = (p.position.x, p.position.y, yaw, abs(msg.twist.twist.linear.x),
                         time.monotonic())

    def on_safety(self, msg):
        self.safety = (msg, time.monotonic())

    def on_edges(self, msg):
        self.edges = ({e.edge_id: e for e in msg.edges}, time.monotonic())

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.run_id = msg.run_id
            with self.lock:
                if self.mission is not None:
                    self.finish(self.mission, 'abort', 'SCENARIO_RESET')
                self.crossing_token = ''
        ack_reset(self, self.events_pub, msg)

    def on_confirm_ready(self, request, response):
        with self.lock:
            m = self.mission
            if m is None or m.state != 'WAIT_READY':
                response.success = False
                response.message = 'NOT_WAITING_FOR_READY'
                self.event('READY_REJECTED', response.message)
                return response
            m.ready = True
        response.success = True
        response.message = 'READY_ACCEPTED'
        self.event('READY_CONFIRMED', '', m.request_id)
        return response

    # ------------------------------------------------------------------ action lifecycle
    def on_goal(self, kind, goal):
        now = time.monotonic()
        nodes = self.robot_nodes()
        targets = [goal.pickup_node, goal.destination_node] if kind == 'escort' \
            else [goal.goal_node]
        reason = ''
        if not goal.request_id:
            reason = 'INVALID_REQUEST_ID'
        elif any(t not in nodes for t in targets):
            reason = 'INVALID_NODE'
        elif goal.max_duration_sec <= 0.0:
            reason = 'INVALID_DEADLINE'
        elif not self.odom_fresh(now):
            reason = 'NO_ODOMETRY'
        elif self.safety_latched(now):
            reason = 'UNSAFE_STARTUP'
        else:
            start = self.nearest_node()
            for t in targets:
                if start != t and not plan(start, t, 'robot')[0]:
                    reason = 'UNREACHABLE_' + t
                start = t
        with self.lock:
            if not reason and self.mission is not None:
                reason = 'BUSY'
            if reason:
                self.event('GOAL_REJECTED', '%s %s' % (kind, reason), goal.request_id)
                return GoalResponse.REJECT
            # Reserve the single active-goal slot atomically.
            self.mission = Mission(kind, goal, now + goal.max_duration_sec)
            self.mission.current_node = self.nearest_node()
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        with self.lock:
            m = self.mission
            if m is None or m.goal_handle is not None:
                goal_handle.abort()
                res = (EscortPassenger if hasattr(goal_handle.request, 'pickup_node')
                       else NavigateTo).Result()
                res.reason = 'NOT_ACTIVE'
                return res
            m.goal_handle = goal_handle
        self.event('GOAL_ACCEPTED', m.kind, m.request_id)
        while not m.done.wait(0.5):              # feedback at 2 Hz
            goal_handle.publish_feedback(self.feedback(m))
        status, reason = m.terminal
        result = self.result(m, status == 'succeed', reason)
        if status == 'succeed':
            goal_handle.succeed()
        elif status == 'canceled':
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.event('GOAL_' + status.upper(), reason, m.request_id)
        return result

    def finish(self, m, status, reason):
        if m.terminal is not None:
            return
        self.command()
        m.terminal = (status, reason)
        if self.mission is m:
            self.mission = None
        m.done.set()

    def remaining(self, m):
        if self.pose is None:
            return 0.0
        rest = 0.0
        if m.route and m.i < len(m.route):
            rest = math.dist(self.pose[:2], hub_map.NODES[m.route[m.i]]) + \
                route_length(m.route[m.i:], hub_map.NODES)
        prev = m.leg_goal
        for _, goal in m.legs[m.leg + 1:]:
            rest += route_length(plan(prev, goal, 'robot')[0], hub_map.NODES)
            prev = goal
        return rest

    def feedback(self, m):
        fb = (EscortPassenger if m.kind == 'escort' else NavigateTo).Feedback()
        with self.lock:
            remaining = self.remaining(m)
            fb.phase = '%s/%s' % (m.phase, m.state)
            total = m.travelled + remaining
            # Progress = travelled / (travelled + remaining); it may drop after a detour.
            fb.progress = float(m.travelled / total) if total > 1e-6 else 0.0
            fb.distance_remaining_m = float(remaining)
        return fb

    def result(self, m, success, reason):
        res = (EscortPassenger if m.kind == 'escort' else NavigateTo).Result()
        res.success = success
        res.reason = reason
        res.final_node = m.current_node
        if m.kind == 'escort':
            res.travelled_distance_m = float(m.travelled)
        elif self.pose is not None:
            res.final_error_m = float(math.dist(self.pose[:2], hub_map.NODES[m.leg_goal]))
        return res

    # ------------------------------------------------------------------ service calls
    def call(self, name, client, request, now):
        attempt = self.pending.get(name, (None, 0.0, 0))[2]
        future = client.call_async(request) if client.service_is_ready() else None
        self.pending[name] = (future, now, attempt + 1)

    def poll(self, name, now):
        """Return ('wait' | 'done' | 'timeout' | 'failed', result)."""
        future, sent, attempt = self.pending[name]
        if future is not None and future.done():
            del self.pending[name]
            try:
                return 'done', future.result()
            except Exception as exc:
                return 'failed', str(exc)
        if now - sent > SERVICE_TIMEOUT_SEC:
            if future is not None:
                future.cancel()   # a late reply is ignored
            if attempt > SERVICE_RETRIES:
                del self.pending[name]
                return 'failed', 'TIMEOUT'
            return 'timeout', None
        return 'wait', None

    def request_route(self, m, now):
        req = ComputeRoute.Request()
        req.start_node = m.current_node
        req.goal_node = m.leg_goal
        req.agent_type = 'robot'
        req.excluded_edges = sorted(m.excluded)
        self.call('route', self.route_client, req, now)

    def request_crossing(self, operation, now):
        req = RequestCrossing.Request()
        req.requester_id = ROBOT_ID
        req.operation = operation
        req.token = self.crossing_token
        self.call('crossing_' + operation, self.crossing_client, req, now)

    def service_crossing_lease(self, now):
        """Renew a held token; release it once the footprint has cleared C."""
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
        m = self.mission
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
            if m is None or m.goal_handle is None or m.terminal is not None:
                return
            if m.goal_handle.is_cancel_requested and m.state != 'STOPPING':
                m.state, m.cancel_since = 'STOPPING', now
                self.event('CANCEL_ACCEPTED', '', m.request_id)
            if m.state == 'STOPPING':
                self.command()
                if self.odom_fresh(now) and self.pose[3] < STOPPED_MPS:
                    self.finish(m, 'canceled', 'CANCELED latency_ms=%.1f' % (
                        (now - m.cancel_since) * 1000.0))
                elif now - m.cancel_since > 2.0:
                    self.finish(m, 'abort', 'STOP_NOT_CONFIRMED')
                return
            if not self.odom_fresh(now):
                self.finish(m, 'abort', 'ODOM_STALE')
            elif self.safety_latched(now):
                self.finish(m, 'abort', 'SAFE_STOP')
            elif now > m.deadline:
                self.finish(m, 'abort', 'TIMEOUT')
            else:
                getattr(self, 'state_' + m.state.lower())(m, now)

    def state_planning(self, m, now):
        self.command()
        if now < m.retry_at:
            return
        if 'route' not in self.pending:
            self.request_route(m, now)
            return
        status, res = self.poll('route', now)
        if status == 'timeout':
            self.request_route(m, now)
        elif status == 'failed':
            self.finish(m, 'abort', 'PLANNER_UNAVAILABLE')
        elif status == 'done' and not res.success:
            if res.reason == 'NO_ROUTE':
                self.event('NO_ROUTE', 'excluded=%s' % ','.join(sorted(m.excluded)),
                           m.request_id)
                self.finish(m, 'abort', 'NO_ROUTE')
            else:
                m.retry_at = now + REPLAN_WAIT_SEC
                self.event('ROUTE_UNAVAILABLE', res.reason, m.request_id)
        elif status == 'done':
            m.route, m.i, m.edge_checked = list(res.node_ids), 0, False
            m.state = 'MOVING'
            self.event('ROUTE', '%s %s cost=%.1fs' % (
                m.phase, '-'.join(m.route), res.estimated_cost_sec), m.request_id)

    def route_cost(self, nodes):
        """Cost of a node path with the planner's formula and the current edge states."""
        edges, _ = self.edges
        cost = 0.0
        for a, b in zip(nodes, nodes[1:]):
            e = edges.get(hub_map.edge_id(a, b))
            if e is None or not e.valid or e.blocked:
                return math.inf
            speed = max(e.mean_speed_mps, 0.05) if e.density_ratio > 0.0 else DEFAULT_SPEED_MPS
            cost += edge_cost(hub_map.edge_length(a, b), speed, e.density_ratio)
        return cost

    def state_check_route(self, m, now):
        """Switch route only if the alternative is at least replan_improvement_ratio cheaper."""
        self.command()
        if 'route' not in self.pending:
            self.request_route(m, now)
            return
        status, res = self.poll('route', now)
        if status == 'wait':
            return
        m.state = 'MOVING'
        remaining = m.route[m.i - 1:]
        if status != 'done' or not res.success or list(res.node_ids) == remaining:
            return
        current = self.route_cost(remaining)
        ratio = self.get_parameter('replan_improvement_ratio').value
        if res.estimated_cost_sec <= (1.0 - ratio) * current:
            self.event('REPLAN_CONGESTION', '%s (%.1fs) -> %s (%.1fs)' % (
                '-'.join(remaining), current, '-'.join(res.node_ids), res.estimated_cost_sec),
                m.request_id)
            m.route, m.i, m.edge_checked = list(res.node_ids), 0, False
            m.replans += 1

    def edge_status(self, edge_id, now):
        """'ok', 'blocked' (obstructed edge) or 'no_data' (stale or unknown edge state)."""
        edges, receipt = self.edges
        if now - receipt > EDGES_STALE_SEC:
            return 'no_data'
        e = edges.get(edge_id)
        if e is None or not e.valid:
            return 'no_data'
        return 'blocked' if e.blocked else 'ok'

    def state_moving(self, m, now):
        x, y, yaw = self.pose[0], self.pose[1], self.pose[2]
        target = m.route[m.i]
        prev = m.route[m.i - 1] if m.i > 0 else None
        if prev is not None:
            status = self.edge_status(hub_map.edge_id(prev, target), now)
            if status == 'no_data':
                self.command()           # stale critical data: hold position
                return
            if status == 'blocked':
                # Stop and replan from the last safe node (reversing to it if mid-edge).
                self.command()
                m.excluded.add(hub_map.edge_id(prev, target))
                m.current_node = prev
                m.replans += 1
                m.state = 'PLANNING'
                self.event('REROUTE', 'edge %s blocked' % hub_map.edge_id(prev, target),
                           m.request_id)
                return
        if not m.edge_checked:
            needs_c = segment_crosses_c((x, y), hub_map.NODES[target]) and \
                not footprint_in_c(x, y)
            if needs_c and not self.crossing_token:
                m.state = 'WAIT_CROSSING'
                self.state_wait_crossing(m, now)
                return
            m.edge_checked = True
        tx, ty = hub_map.NODES[target]
        last = m.i == len(m.route) - 1
        v, w, dist = follow(x, y, yaw, tx, ty, self.get_parameter('robot_speed_limit').value,
                            self.get_parameter('max_angular_rad_s').value, final=last)
        tolerance = self.get_parameter('arrival_tolerance_m').value * 0.5 if last \
            else WAYPOINT_TOLERANCE_M
        if dist > tolerance:
            self.command(v, w)
            return
        m.current_node = target
        m.i += 1
        m.edge_checked = False
        if not last:
            if now - m.last_replan_check >= self.get_parameter('replan_period_sec').value:
                # Congestion-only replanning at a node, at most once per period (13.2).
                m.last_replan_check = now
                m.state = 'CHECK_ROUTE'
                self.command()
                return
            self.state_moving(m, now)
            return
        self.command()
        if m.final_leg:
            m.state, m.arrive_since = 'ARRIVING', None
        else:
            m.state, m.wait_since, m.ready = 'WAIT_READY', now, False
            self.event('PICKUP_ARRIVED', 'waiting for /robot/confirm_ready', m.request_id)

    def state_wait_crossing(self, m, now):
        self.command()
        if now < m.retry_at:
            return
        if 'crossing_ACQUIRE' not in self.pending:
            self.request_crossing('ACQUIRE', now)
            return
        status, res = self.poll('crossing_ACQUIRE', now)
        if status == 'timeout':
            self.request_crossing('ACQUIRE', now)
        elif status == 'failed':
            m.retry_at = now + CROSSING_RETRY_SEC
        elif status == 'done' and res.granted:
            self.crossing_token = res.token
            self.last_renew = now
            m.state = 'MOVING'
            m.edge_checked = True
            self.event('CROSSING_ACQUIRED', '', res.token)
        elif status == 'done':
            m.retry_at = now + CROSSING_RETRY_SEC   # bounded back-off

    def state_wait_ready(self, m, now):
        self.command()
        if m.ready:
            m.leg += 1
            m.excluded.clear()
            m.state = 'PLANNING'
        elif now - m.wait_since > self.get_parameter('ready_timeout_sec').value:
            self.finish(m, 'abort', 'READY_TIMEOUT')

    def state_arriving(self, m, now):
        self.command()
        error = math.dist(self.pose[:2], hub_map.NODES[m.leg_goal])
        if error > self.get_parameter('arrival_tolerance_m').value:
            m.route, m.i, m.state = [m.leg_goal], 0, 'MOVING'
            return
        if self.pose[3] >= STOPPED_MPS:
            m.arrive_since = None
        elif m.arrive_since is None:
            m.arrive_since = now
        elif now - m.arrive_since >= 1.0:
            m.current_node = m.leg_goal
            self.event('ARRIVED', 'error_m=%.3f replans=%d' % (error, m.replans), m.request_id)
            self.finish(m, 'succeed', 'ARRIVED')


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
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
