"""
scenario_runner — owner M8.

Deterministic 2D simulator at 20 Hz: seeded scenario events, private ground truth,
noisy observations for detectors. Serves /scenario/reset (test orchestration only).
See assignment 3.2 and 7.1.
"""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from smart_hub.qos import R1, R10, SENSOR
from smart_hub_interfaces.msg import SlotObservationArray, VehicleStateArray
from smart_hub_interfaces.srv import ResetScenario


STEP_SEC = 0.05


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('scenario_runner')
        self.declare_parameter('scenario', 'S1')
        self.declare_parameter('seed', 42)
        self.declare_parameter('scenario_dir', '')

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', SENSOR)
        self.vehicles_pub = self.create_publisher(VehicleStateArray, '/hub/vehicles', R10)
        self.slots_pub = self.create_publisher(SlotObservationArray, '/parking/observed', R10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', R10)
        # TODO(M8): private ground-truth topic for metrics_logger only.

        self.create_subscription(Twist, '/robot/cmd_vel', self.on_robot_cmd, R1)
        self.create_subscription(Twist, '/bus/cmd_vel', self.on_bus_cmd, R1)
        self.create_service(ResetScenario, '/scenario/reset', self.on_reset)

        self.create_timer(STEP_SEC, self.step)

    def on_robot_cmd(self, msg):
        pass  # TODO(M8): store latest safe velocity for the robot

    def on_bus_cmd(self, msg):
        pass  # TODO(M8): store latest safe velocity for the bus

    def step(self):
        # TODO(M8): integrate kinematics, swept collision check, inject scheduled events,
        #           publish observations (noisy) separately from truth.
        pass

    def on_reset(self, request, response):
        # TODO(M8): cancel goals, zero motion, reload seeded world, collect node acks.
        response.success = False
        response.reason = 'NOT_IMPLEMENTED'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
