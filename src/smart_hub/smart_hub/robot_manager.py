"""
robot_manager — owner M7 (scenario S7).

Action servers /robot/escort (EscortPassenger) and /robot/navigate_to (NavigateTo);
/robot/confirm_ready (std_srvs/Trigger). IDLE -> TO_PICKUP -> WAIT_READY -> ESCORT -> ARRIVED,
BLOCKED / WAIT_CROSSING pause motion. See assignment 14, 18.5.
"""
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from smart_hub.qos import R1, R10, R1T
from smart_hub_interfaces.action import EscortPassenger, NavigateTo
from smart_hub_interfaces.msg import CrossingState, SafetyState
from smart_hub_interfaces.srv import ComputeRoute, RequestCrossing
from std_srvs.srv import Trigger


class RobotManager(Node):

    def __init__(self):
        super().__init__('robot_manager')
        self.declare_parameter('robot_speed_limit', 0.25)
        self.declare_parameter('arrival_tolerance_m', 0.20)
        self.declare_parameter('ready_timeout_sec', 20.0)
        self.cb = ReentrantCallbackGroup()

        self.cmd_pub = self.create_publisher(TwistStamped, '/robot/cmd_vel_candidate', R1)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)

        self.route_client = self.create_client(
            ComputeRoute, '/route/compute', callback_group=self.cb)
        self.crossing_client = self.create_client(
            RequestCrossing, '/crossing/request', callback_group=self.cb)
        self.create_service(Trigger, '/robot/confirm_ready', self.on_confirm_ready,
                            callback_group=self.cb)

        self.escort_server = ActionServer(
            self, EscortPassenger, '/robot/escort',
            execute_callback=self.execute_escort,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )
        self.navigate_server = ActionServer(
            self, NavigateTo, '/robot/navigate_to',
            execute_callback=self.execute_navigate,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self.cb,
        )

    def on_odom(self, msg):
        pass  # TODO(M7): pose + freshness

    def on_safety(self, msg):
        pass

    def on_crossing(self, msg):
        pass

    def on_confirm_ready(self, request, response):
        # TODO(M7): accept only in WAIT_READY
        response.success = False
        response.message = 'NOT_WAITING_FOR_READY'
        return response

    def on_goal(self, goal_request):
        self.get_logger().warning('Robot missions not implemented yet; rejecting goal')
        return GoalResponse.REJECT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_escort(self, goal_handle):
        goal_handle.abort()
        result = EscortPassenger.Result()
        result.reason = 'NOT_IMPLEMENTED'
        return result

    def execute_navigate(self, goal_handle):
        goal_handle.abort()
        result = NavigateTo.Result()
        result.reason = 'NOT_IMPLEMENTED'
        return result


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
