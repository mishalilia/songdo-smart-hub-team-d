"""
metrics_logger — owner M5.

Records run ID, scenario, seed, commit/config hash, events, latencies, violations and
model errors to CSV/JSON under the output directory. The only node allowed to read
private ground truth. See assignment 21.
"""
import rclpy
from rclpy.node import Node
from smart_hub.qos import R100, R1T
from smart_hub_interfaces.msg import HubEvent, SafetyState


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('output_dir', 'evidence/runs')
        self.declare_parameter('run_id', '')

        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)

    def on_event(self, msg):
        self.get_logger().info(
            f'[{msg.scenario_id}/{msg.run_id}] {msg.entity_id}: {msg.event} ({msg.reason}) '
            f'corr={msg.correlation_id}')
        # TODO(M5): append to run CSV

    def on_safety(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
