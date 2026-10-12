"""
demand_predictor — owner M2 (scenario S2).

Keeps 60 s of 1 Hz queue history, publishes persistence baseline and trained-model
forecast 30 s ahead on /hub/demand at 1 Hz. valid=false during warm-up/stale input.
See assignment 9.
"""
import rclpy
from rclpy.node import Node
from smart_hub.qos import R10
from smart_hub_interfaces.msg import DemandPrediction, QueueState


class DemandPredictor(Node):

    def __init__(self):
        super().__init__('demand_predictor')
        self.declare_parameter('prediction_horizon_sec', 30)
        self.declare_parameter('history_window_sec', 60)
        self.declare_parameter('model_path', '')

        self.demand_pub = self.create_publisher(DemandPrediction, '/hub/demand', R10)
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_timer(1.0, self.tick)

        self.history = []

    def on_queue(self, msg):
        pass  # TODO(M2): keep latest valid count per second

    def tick(self):
        msg = DemandPrediction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.horizon_sec = self.get_parameter('prediction_horizon_sec').value
        msg.model_id = 'persistence'
        msg.valid = False  # TODO(M2): features, baseline, ridge model
        self.demand_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemandPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
