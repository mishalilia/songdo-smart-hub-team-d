# Author: Hsu Shwe Sin Htun (12240198)
"""
demand_predictor (scenario S2).

Samples /hub/queue at 1 Hz and publishes the persistence baseline and the ridge forecast
30 s ahead on /hub/demand at 1 Hz (assignment 9.1-9.3). A forecast is valid only with a
time-contiguous history_window_sec (60 s) history of validated observations
(demand_model.QueueHistory): the source zone, explicit validity, local receipt age and
the acquisition stamp (age, order, replay, future) are all checked; an invalid message
erases the history at once and any gap restarts the warm-up.
The forecast header stamp is the acquisition stamp of the newest supporting queue
observation, not the publication time, so consumers can gate its real age (6, 6.1).
Falls back to persistence (model_id 'persistence') when no model file is found.
Model: demand_model.RidgeModel, trained by `ros2 run smart_hub train_demand_model`.
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from smart_hub.demand_model import features, QueueHistory, RidgeModel
from smart_hub.freshness import stamp_sec
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100
from smart_hub_interfaces.msg import DemandPrediction, HubEvent, QueueState

LATENCY_EVENT_PERIOD = 10
SOURCE_ZONE = 'Q'


class DemandPredictor(Node):

    def __init__(self):
        super().__init__('demand_predictor')
        self.declare_parameter('prediction_horizon_sec', 30)
        self.declare_parameter('history_window_sec', 60)
        self.declare_parameter('model_path', '')
        self.declare_parameter('stale_timeout_sec', 1.0)
        check_parameters(self)

        self.model = self.load_model()
        self.history = QueueHistory(
            window_sec=self.get_parameter('history_window_sec').value,
            stale_sec=self.get_parameter('stale_timeout_sec').value, zone_id=SOURCE_ZONE)
        self.valid = None
        self.rejected = ''                 # last input rejection reason (events on change)
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

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def event(self, name, reason=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.entity_id = 'demand_predictor'
        msg.event = name
        msg.reason = reason
        self.events_pub.publish(msg)

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.history.reset()
        ack_reset(self, self.events_pub, msg)

    def on_queue(self, msg):
        stamp = stamp_sec(msg.header.stamp)
        problem = self.history.observe(msg.valid, msg.count, stamp, msg.zone_id, self.now(),
                                       time.monotonic())
        if problem == 'REPLAY':
            return                         # same observation republished: nothing new
        if problem != self.rejected:       # transitions only, never one event per message
            if problem:
                self.event('QUEUE_INPUT_REJECTED', '%s zone=%s stamp=%.3f' % (
                    problem, msg.zone_id, stamp))
            self.rejected = problem

    def tick(self):
        self.ticks += 1
        now = self.get_clock().now()
        fresh = self.history.sample(now.nanoseconds / 1e9, time.monotonic())
        msg = DemandPrediction()
        msg.header.frame_id = 'map'
        msg.horizon_sec = self.get_parameter('prediction_horizon_sec').value
        msg.model_id = self.model.model_id if self.model else 'persistence'
        x = features(self.history.counts()) if fresh and self.history.ready() else None
        if x is not None:
            start = time.perf_counter()
            msg.baseline_count = float(x[0])
            msg.predicted_count = self.model.predict(x) if self.model else float(x[0])
            latency_ms = (time.perf_counter() - start) * 1000.0
            msg.valid = True
            # Observation time of the newest supporting queue sample, not "now".
            msg.header.stamp = Time(
                nanoseconds=int(self.history.acquisition_stamp() * 1e9)).to_msg()
            if self.ticks % LATENCY_EVENT_PERIOD == 0:
                self.event('DEMAND_INFERENCE', 'latency_ms=%.4f' % latency_ms)
        else:
            msg.valid = False
            msg.header.stamp = now.to_msg()        # no supporting observation exists
        if msg.valid != self.valid:
            self.event('PREDICTION_' + ('VALID' if msg.valid else 'UNAVAILABLE'),
                       'history=%.0f/%d s input=%s' % (
                           self.history.span_sec(), self.history.window_sec,
                           self.rejected or 'OK'))
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
