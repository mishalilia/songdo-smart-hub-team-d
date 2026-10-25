# Author: Komronbek Tohirov (12235649)
"""
hub_coordinator (scenarios S2 dispatch, S8).

Crossing mutex: /crossing/request server (ACQUIRE / RENEW / RELEASE) backed by
crossing_arbiter.CrossingArbiter, /crossing/state at 10 Hz, grant/release events.
Occupancy comes from /hub/queue (pedestrians in C) and /hub/vehicles (agent footprints).
See assignment 7.1, 15.2, 18.4.
Demand dispatch (9.3): dispatch_policy.DispatchPolicy on /hub/demand at 1 Hz requests one
DockBus goal at a time; outcome events DISPATCH_REQUESTED / DISPATCH_RESULT.
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smart_hub import hub_map
from smart_hub.crossing_arbiter import CrossingArbiter
from smart_hub.dispatch_policy import DispatchPolicy
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100, R1T
from smart_hub_interfaces.action import DockBus, EscortPassenger
from smart_hub_interfaces.msg import (
    BusArrival, CrossingState, DemandPrediction, EdgeStateArray, HubEvent, ParkingState,
    QueueState, VehicleStateArray,
)
from smart_hub_interfaces.srv import RequestCrossing

STALE_SEC = 1.0
AGENT_RADIUS = {'robot': 0.20, 'bus': 0.30, 'car': 0.30}


def footprint_in_c(x, y, radius):
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    return xmin - radius <= x <= xmax + radius and ymin - radius <= y <= ymax + radius


class HubCoordinator(Node):

    def __init__(self):
        super().__init__('hub_coordinator')
        self.declare_parameter('dispatch_high', 6.0)
        self.declare_parameter('dispatch_low', 4.0)
        self.declare_parameter('dispatch_cooldown_sec', 30.0)
        self.declare_parameter('crossing_clear_sec', 2.0)
        self.declare_parameter('crossing_lease_sec', 2.0)
        self.declare_parameter('dispatch_mode', 'predictive')   # predictive | reactive
        check_parameters(self)

        self.arbiter = CrossingArbiter(
            clear_sec=self.get_parameter('crossing_clear_sec').value,
            lease_sec=self.get_parameter('crossing_lease_sec').value)
        self.queue_state = None
        self.queue_receipt = -1e9
        self.occupants = set()
        self.vehicles_receipt = -1e9
        self.run_id = ''
        self.last_state = None
        self.policy = DispatchPolicy(
            high=self.get_parameter('dispatch_high').value,
            low=self.get_parameter('dispatch_low').value,
            cooldown_sec=self.get_parameter('dispatch_cooldown_sec').value)
        self.bus_state = (None, -1e9)
        self.dock_goal = None          # outstanding DockBus goal handle (or 'SENT')

        self.crossing_pub = self.create_publisher(CrossingState, '/crossing/state', R1T)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)

        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(DemandPrediction, '/hub/demand', self.on_demand, R10)
        self.create_subscription(BusArrival, '/hub/bus_arrival', self.on_bus_arrival, R10)
        self.create_subscription(ParkingState, '/parking/state', self.on_parking, R1T)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)

        self.create_service(RequestCrossing, '/crossing/request', self.on_crossing_request)
        self.dock_client = ActionClient(self, DockBus, '/bus/dock')
        self.escort_client = ActionClient(self, EscortPassenger, '/robot/escort')

        self.create_timer(0.1, self.tick)

    def event(self, entity_id, name, reason='', correlation_id=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id = self.run_id
        msg.entity_id = entity_id
        msg.event = name
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events_pub.publish(msg)

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.run_id = msg.run_id
            self.arbiter = CrossingArbiter(
                clear_sec=self.get_parameter('crossing_clear_sec').value,
                lease_sec=self.get_parameter('crossing_lease_sec').value)
        ack_reset(self, self.events_pub, msg)

    def on_queue(self, msg):
        self.queue_state = msg
        self.queue_receipt = time.monotonic()

    def on_vehicles(self, msg):
        self.occupants = {v.vehicle_id for v in msg.vehicles
                          if v.kind in AGENT_RADIUS and v.valid and
                          footprint_in_c(v.pose.x, v.pose.y, AGENT_RADIUS[v.kind])}
        self.vehicles_receipt = time.monotonic()

    def on_demand(self, msg):
        now = time.monotonic()
        q = self.queue_state
        queue_valid = q is not None and q.valid and now - self.queue_receipt <= STALE_SEC
        bus, receipt = self.bus_state
        bus_available = bus is not None and now - receipt <= 2.0 and bus.state == 'IDLE'
        # Reactive mode ignores the forecast and dispatches on the observed queue only, so the
        # two policies can be compared on identical seeds (assignment 9.4, 15.3).
        predictive = self.get_parameter('dispatch_mode').value == 'predictive'
        reason = self.policy.update(now, predictive and msg.valid, msg.predicted_count,
                                    queue_valid, q.count if queue_valid else 0, bus_available,
                                    self.dock_goal is not None)
        if reason == 'FALLBACK' and not predictive:
            reason = 'REACTIVE'
        if reason:
            self.dispatch(reason, msg.predicted_count if reason == 'PREDICTED' else q.count)

    def dispatch(self, reason, value):
        if not self.dock_client.server_is_ready():
            self.event('bus1', 'DISPATCH_FAILED', 'dock action server unavailable')
            return
        goal = DockBus.Goal(bus_id='bus1', bay_id='B', max_duration_sec=240.0)
        self.dock_goal = 'SENT'
        future = self.dock_client.send_goal_async(goal)

        def accepted(f):
            handle = f.result()
            goal_id = ''.join('%02x' % b for b in bytes(handle.goal_id.uuid))
            if not handle.accepted:
                self.dock_goal = None
                self.event('bus1', 'DISPATCH_REJECTED', reason, goal_id)
                return
            self.dock_goal = handle
            self.event('bus1', 'DISPATCH_REQUESTED', '%s demand=%.1f' % (reason, value),
                       goal_id)
            handle.get_result_async().add_done_callback(
                lambda r: self.on_dock_result(goal_id, r))

        future.add_done_callback(accepted)

    def on_dock_result(self, goal_id, future):
        res = future.result()
        self.dock_goal = None
        self.event('bus1', 'DISPATCH_RESULT', 'status=%d %s' % (res.status, res.result.reason),
                   goal_id)

    def on_bus_arrival(self, msg):
        self.bus_state = (msg, time.monotonic())

    def on_parking(self, msg):
        pass

    def on_edges(self, msg):
        pass

    def observe(self):
        now = time.monotonic()
        q = self.queue_state
        queue_ok = q is not None and q.valid and now - self.queue_receipt <= STALE_SEC
        vehicles_ok = now - self.vehicles_receipt <= STALE_SEC
        self.arbiter.observe(now, queue_ok and q.crossing_occupied,
                             queue_ok and vehicles_ok, self.occupants)
        return now

    def on_crossing_request(self, request, response):
        start = time.monotonic()
        now = self.observe()
        previous_owner = self.arbiter.owner
        granted, token, reason = self.arbiter.request(
            request.requester_id, request.operation, request.token, now)
        response.granted, response.token, response.reason = granted, token, reason
        latency = 'latency_ms=%.2f' % ((time.monotonic() - start) * 1000.0)
        if request.operation == 'ACQUIRE' and granted and previous_owner != request.requester_id:
            self.event(request.requester_id, 'CROSSING_GRANTED', latency, token)
        elif request.operation == 'RELEASE' and granted:
            self.event(request.requester_id, 'CROSSING_RELEASED', latency, request.token)
        elif not granted and request.operation != 'ACQUIRE':
            self.event(request.requester_id, 'CROSSING_' + request.operation + '_DENIED', reason)
        return response

    def tick(self):
        self.observe()
        a = self.arbiter
        msg = CrossingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.owner_id = a.owner
        msg.token = a.token
        msg.state = a.state
        msg.pedestrian_present = a.pedestrian_present
        msg.valid = a.observation_valid
        self.crossing_pub.publish(msg)
        if a.state != self.last_state:
            self.event('crossing', 'CROSSING_' + a.state, a.owner)
            self.last_state = a.state


def main(args=None):
    rclpy.init(args=args)
    node = HubCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
