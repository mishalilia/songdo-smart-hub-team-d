"""
passenger_detector — owner M1 (scenario S1).

Detect + track people, count confirmed tracks whose foot-point lies in Q, flag pedestrians
in C. Publishes /hub/queue at 5 Hz; stale feed (>1 s) => valid=false. See assignment 8.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from smart_hub.qos import R10, SENSOR
from smart_hub_interfaces.msg import QueueState


class PassengerDetector(Node):

    def __init__(self):
        super().__init__('passenger_detector')
        self.declare_parameter('queue_confidence_min', 0.5)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('crossing_clear_sec', 2.0)

        self.queue_pub = self.create_publisher(QueueState, '/hub/queue', R10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, SENSOR)
        self.create_timer(0.2, self.tick)

        self.last_image_stamp = None

    def on_image(self, msg):
        self.last_image_stamp = msg.header.stamp
        # TODO(M1): detection -> tracking (0.4 s debounce, 0.5 s occlusion) -> homography

    def tick(self):
        msg = QueueState()
        msg.header.stamp = self.get_clock().now().to_msg()  # TODO(M1): use observation stamp
        msg.header.frame_id = 'map'
        msg.zone_id = 'Q'
        msg.valid = False  # UNKNOWN until real detections exist
        self.queue_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PassengerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
