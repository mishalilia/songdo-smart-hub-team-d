"""
actuator_bridge — owner M9.

The only hardware-specific layer. In sim mode it forwards nothing (simulator reads the
final commands); in hardware mode it drives motors / servo / LEDs. Independently expires
final commands (a crashed supervisor must not leave a latched nonzero command).
Publishes measured door / barrier status. See assignment 16.2, 22.
"""
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from smart_hub.qos import R1
from std_msgs.msg import Bool


class ActuatorBridge(Node):

    def __init__(self):
        super().__init__('actuator_bridge')
        self.declare_parameter('mode', 'sim')  # sim | hardware (explicit opt-in)
        self.declare_parameter('command_timeout_sec', 0.5)

        self.last_cmd = {'robot': None, 'bus': None}
        self.create_subscription(Twist, '/robot/cmd_vel', lambda m: self.on_cmd('robot', m), R1)
        self.create_subscription(Twist, '/bus/cmd_vel', lambda m: self.on_cmd('bus', m), R1)
        self.create_subscription(Bool, '/bus/door_request', self.on_door_request, R1)
        self.create_subscription(Bool, '/parking/barrier_request', self.on_barrier_request, R1)

        self.door_pub = self.create_publisher(Bool, '/bus/door_open', R1)
        self.barrier_pub = self.create_publisher(Bool, '/parking/barrier_open', R1)

        self.create_timer(0.05, self.tick)

    def on_cmd(self, agent, msg):
        self.last_cmd[agent] = (msg, time.monotonic())

    def on_door_request(self, msg):
        pass  # TODO(M9): open only when bus measured stopped; publish measured status

    def on_barrier_request(self, msg):
        pass  # TODO(M9): valid owner + clear entrance sensor; never close on occupied lane

    def tick(self):
        # TODO(M9): drive hardware; on expiry command zero independently of the supervisor
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
