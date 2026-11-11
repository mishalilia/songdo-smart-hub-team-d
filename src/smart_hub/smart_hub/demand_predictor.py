# Author: Hsu Shwe Sin Htun (12240198)
"""
demand_predictor (scenario S2).

Samples the latest valid /hub/queue count at 1 Hz, keeps 60 s of history and publishes the
persistence baseline and the ridge forecast 30 s ahead on /hub/demand at 1 Hz (assignment
9.1-9.3). valid=false during warm-up (31 samples) or stale input; a feed gap restarts the
warm-up. Falls back to persistence (model_id 'persistence') when no model file is found.
Model: demand_model.RidgeModel, trained by `ros2 run smart_hub train_demand_model`.
"""
from collections import deque
import os
import time

import rclpy
from rclpy.node import Node
from smart_hub.demand_model import features, HISTORY_SEC, RidgeModel
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100
from smart_hub_interfaces.msg import DemandPrediction, HubEvent, QueueState

LATENCY_EVENT_PERIOD = 10


class DemandPredictor(Node):

    def __init__(self):
        super().__init__('demand_predictor')
        self.declare_parameter('prediction_horizon_sec', 30)
        self.declare_parameter('history_window_sec', 60)
        self.declare_parameter('model_path', '')
        self.declare_parameter('stale_timeout_sec', 1.0)
        check_parameters(self)

        self.model = self.load_model()
        self.history = deque(maxlen=self.get_parameter('history_window_sec').value)
        self.latest = None                 # (count, receipt)
        self.valid = None
        self.ticks = 0

        self.demand_pub = self.create_publisher(DemandPrediction, '/hub/demand', R10)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_timer(1.0, self.tick)

    def load_model(self):
        path = self.get_parameter('model_path').value
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory('smart_hub'), 'models',
                                    'demand_ridge.json')
            except LookupError:
                path = ''
        if path and os.path.isfile(path):
            model = RidgeModel.load(path)
            self.get_logger().info('Loaded demand model %s' % model.model_id)
            return model
        self.get_logger().warning('No demand model found; using persistence baseline')
        return None

    def event(self, name, reason=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.entity_id = 'demand_predictor'
        msg.event = name
        msg.reason = reason
        self.events_pub.publish(msg)

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.history.clear()
        ack_reset(self, self.events_pub, msg)

    def on_queue(self, msg):
        if msg.valid:
            self.latest = (msg.count, time.monotonic())

    def tick(self):
        self.ticks += 1
        fresh = self.latest is not None and \
            time.monotonic() - self.latest[1] <= self.get_parameter('stale_timeout_sec').value
        if fresh:
            self.history.append(self.latest[0])
        else:
            self.history.clear()           # a gap breaks the feature windows: re-warm
        msg = DemandPrediction()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.horizon_sec = self.get_parameter('prediction_horizon_sec').value
        msg.model_id = self.model.model_id if self.model else 'persistence'
        x = features(self.history) if fresh else None
        if x is not None:
            start = time.perf_counter()
            msg.baseline_count = float(x[0])
            msg.predicted_count = self.model.predict(x) if self.model else float(x[0])
            latency_ms = (time.perf_counter() - start) * 1000.0
            msg.valid = True
            if self.ticks % LATENCY_EVENT_PERIOD == 0:
                self.event('DEMAND_INFERENCE', 'latency_ms=%.4f' % latency_ms)
        else:
            msg.valid = False
        if msg.valid != self.valid:
            self.event('PREDICTION_' + ('VALID' if msg.valid else 'UNAVAILABLE'),
                       'history=%d/%d' % (len(self.history), HISTORY_SEC))
            self.valid = msg.valid
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
