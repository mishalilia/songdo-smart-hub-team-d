"""
hub_coordinator — owner M3 (scenarios S2 dispatch, S8).

Crossing mutex (/crossing/request server, /crossing/state publisher), demand-driven
DockBus dispatch with hysteresis and cooldown, escort/parking orchestration.
See assignment 9.3, 15, 18.4.
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from smart_hub.qos import R10, R100, R1T
from smart_hub_interfaces.action import DockBus, EscortPassenger
from smart_hub_interfaces.msg import (
    BusArrival, CrossingState, DemandPrediction, EdgeStateArray, HubEvent, ParkingState,
    QueueState,
)
from smart_hub_interfaces.srv import RequestCrossing


class HubCoordinator(Node):

    def __init__(self):
        super().__init__('hub_coordinator')
        self.declare_parameter('dispatch_high', 6.0)
        self.declare_parameter('dispatch_low', 4.0)
        self.declare_parameter('dispatch_cooldown_sec', 30.0)
        self.declare_parameter('crossing_clear_sec', 2.0)

        self.crossing_pub = self.create_publisher(CrossingState, '/crossing/state', R1T)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)

        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(DemandPrediction, '/hub/demand', self.on_demand, R10)
        self.create_subscription(BusArrival, '/hub/bus_arrival', self.on_bus_arrival, R10)
        self.create_subscription(ParkingState, '/parking/state', self.on_parking, R1T)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)

        self.create_service(RequestCrossing, '/crossing/request', self.on_crossing_request)
        self.dock_client = ActionClient(self, DockBus, '/bus/dock')
        self.escort_client = ActionClient(self, EscortPassenger, '/robot/escort')

        self.create_timer(0.1, self.tick)

    def on_queue(self, msg):
        pass  # TODO(M3): pedestrian_present / validity for the crossing

    def on_demand(self, msg):
        pass  # TODO(M3): 3x >= dispatch_high -> one DockBus goal; FALLBACK when invalid

    def on_bus_arrival(self, msg):
        pass

    def on_parking(self, msg):
        pass

    def on_edges(self, msg):
        pass

    def on_crossing_request(self, request, response):
        # TODO(M3): ACQUIRE / RENEW / RELEASE with token, debounce, pedestrian priority
        response.granted = False
        response.reason = 'NOT_IMPLEMENTED'
        return response

    def tick(self):
        msg = CrossingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.state = 'UNKNOWN'
        msg.valid = False
        self.crossing_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HubCoordinator()
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
