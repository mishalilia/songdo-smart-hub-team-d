"""
bus_manager — owner M4 (scenario S3).

DockBus action server (/bus/dock): IDLE -> APPROACH -> WAIT -> DOCK -> BOARD -> COMPLETE,
SAFE_STOP from any moving state. Publishes /hub/bus_arrival and a candidate velocity;
calls /route/compute and /crossing/request asynchronously. See assignment 10, 18.6.
"""
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from smart_hub.qos import R1, R10, R1T
from smart_hub_interfaces.action import DockBus
from smart_hub_interfaces.msg import BusArrival, CrossingState, SafetyState, VehicleStateArray
from smart_hub_interfaces.srv import ComputeRoute, RequestCrossing
from std_msgs.msg import Bool


class BusManager(Node):

    def __init__(self):
        super().__init__('bus_manager')
        self.declare_parameter('bus_speed_limit', 0.40)
        self.declare_parameter('pose_tolerance_m', 0.20)
        self.declare_parameter('yaw_tolerance_rad', 0.17)
        self.declare_parameter('boarding_dwell_sec', 8.0)
        self.cb = ReentrantCallbackGroup()

        self.arrival_pub = self.create_publisher(BusArrival, '/hub/bus_arrival', R10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/bus/cmd_vel_candidate', R1)
        self.door_req_pub = self.create_publisher(Bool, '/bus/door_request', R1)

        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(Bool, '/bus/door_open', self.on_door, R1)

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
        self.create_timer(0.5, self.publish_arrival)

    def on_vehicles(self, msg):
        pass  # TODO(M4): track bus pose / speed, freshness

    def on_safety(self, msg):
        pass

    def on_crossing(self, msg):
        pass

    def on_door(self, msg):
        pass  # measured door status (from actuator_bridge)

    def on_goal(self, goal_request):
        # TODO(M4): validate ids, busy flag, safety, route; accept atomically.
        self.get_logger().warning('DockBus not implemented yet; rejecting goal')
        return GoalResponse.REJECT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        # TODO(M4): state machine ticks; command zero before reporting canceled.
        goal_handle.abort()
        result = DockBus.Result()
        result.success = False
        result.final_state = 'IDLE'
        result.reason = 'NOT_IMPLEMENTED'
        return result

    def publish_arrival(self):
        msg = BusArrival()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.bus_id = 'bus1'
        msg.state = 'IDLE'
        msg.eta_sec = -1.0
        msg.eta_valid = False
        self.arrival_pub.publish(msg)


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
