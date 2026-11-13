# Author: Andrey Li (12250249)
"""
passenger_detector (scenario S1).

Synthetic-track baseline: tracks noisy foot-points from /sim/pedestrians (map frame),
counts confirmed tracks in Q and flags pedestrians in C. Publishes /hub/queue at 5 Hz with
the observation timestamp; no fresh observation for stale_timeout_sec => valid=false.
Camera path (input_mode:=camera): frames from image_topic go through camera_detector
(camera_detector:=yolox CNN by default, or the hog baseline; foot points, image-to-map
homography) on a worker thread, newest frame first, and the resulting map points feed the
same tracker; see evaluate_s1_video.
A frame replaced before the detector took it is counted as dropped, one older than
stale_timeout_sec when taken as expired; CAMERA_FRAMES reports both every 10 s together
with the acquisition-to-tracker-update latency p95 / p99.
"""
import threading
import time

from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from smart_hub.hub_events import ack_reset
from smart_hub.metrics import percentile
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100, SENSOR
from smart_hub.tracker import ZoneTracker
from smart_hub_interfaces.msg import HubEvent, QueueState


class FrameLedger:
    """Camera frame accounting: every received frame is processed, dropped or expired."""

    def __init__(self, keep=2000):
        self.received = self.processed = self.dropped = self.expired = 0
        self.waiting = False
        self.latency_ms = []
        self.keep = keep

    def offer(self):
        """Count a received frame; an unprocessed older one is replaced by it."""
        self.received += 1
        self.dropped += self.waiting
        self.waiting = True

    def take(self, age_sec, max_age_sec):
        """Return True if the waiting frame is still fresh enough to be processed."""
        self.waiting = False
        if age_sec > max_age_sec:
            self.expired += 1
            return False
        return True

    def done(self, latency_ms):
        self.processed += 1
        self.latency_ms.append(latency_ms)
        del self.latency_ms[:-self.keep]

    def report(self):
        text = 'received=%d processed=%d dropped=%d expired=%d' % (
            self.received, self.processed, self.dropped, self.expired)
        if self.latency_ms:
            text += ' acquisition_to_update_ms_p95=%.1f p99=%.1f n=%d' % (
                percentile(self.latency_ms, 95), percentile(self.latency_ms, 99),
                len(self.latency_ms))
        return text


class PassengerDetector(Node):

    def __init__(self):
        super().__init__('passenger_detector')
        self.declare_parameter('queue_confidence_min', 0.5)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('crossing_clear_sec', 2.0)
        self.declare_parameter('input_mode', 'sim')          # sim | camera
        self.declare_parameter('image_topic', '/camera/video')
        self.declare_parameter('camera_detector', 'yolox')    # see camera_detector.DETECTORS
        self.declare_parameter('camera_model_path', '')       # '' = installed YOLOX model
        check_parameters(self)

        self.tracker = ZoneTracker(
            crossing_clear_sec=self.get_parameter('crossing_clear_sec').value)
        self.queue_pub = self.create_publisher(QueueState, '/hub/queue', R10)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.lock = threading.Lock()
        self.frame = None                # newest camera frame waiting for detection
        self.frames = FrameLedger()
        if self.get_parameter('input_mode').value == 'camera':
            from smart_hub.camera_detector import CameraCounter
            self.counter = CameraCounter(
                detector=self.get_parameter('camera_detector').value,
                model_path=self.get_parameter('camera_model_path').value)
            self.create_subscription(Image, self.get_parameter('image_topic').value,
                                     self.on_image, SENSOR)
            threading.Thread(target=self.detect_loop, daemon=True).start()
            self.create_timer(10.0, self.report_frames)
        else:
            self.create_subscription(PoseArray, '/sim/pedestrians', self.on_pedestrians, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R10)
        self.create_timer(0.2, self.tick)

        self.last_stamp = None          # observation stamp of the newest frame
        self.last_receipt = -1e9        # monotonic receipt time of the newest frame
        self.last_state = None

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.tracker.reset()
            self.last_state = None             # report the current state in the new run
        ack_reset(self, self.events_pub, msg)

    def on_pedestrians(self, msg):
        now = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        self.tracker.update([(p.position.x, p.position.y) for p in msg.poses], now)
        self.last_stamp = msg.header.stamp
        self.last_receipt = time.monotonic()

    def on_image(self, msg):
        channels = 3 if msg.encoding in ('bgr8', 'rgb8') else 1
        frame = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, channels)
        with self.lock:
            self.frame = (frame, msg.header.stamp)     # older unprocessed frames are dropped
            self.frames.offer()

    def frame_age(self, stamp):
        """Age of a frame in seconds: ROS time now minus its acquisition stamp."""
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9

    def detect_loop(self):
        while rclpy.ok():
            with self.lock:
                item, self.frame = self.frame, None
                # Old work is expired, not detected late (the queue goes invalid instead).
                if item is not None and not self.frames.take(
                        self.frame_age(item[1]),
                        self.get_parameter('stale_timeout_sec').value):
                    item = None
            if item is None:
                time.sleep(0.01)
                continue
            frame, stamp = item
            points = self.counter.map_points(frame)
            with self.lock:
                self.tracker.update(points, Time.from_msg(stamp).nanoseconds / 1e9)
                self.last_stamp = stamp
                self.last_receipt = time.monotonic()
                self.frames.done(self.frame_age(stamp) * 1000.0)

    def report_frames(self):
        with self.lock:
            text = self.frames.report()
        self.event('CAMERA_FRAMES', text)

    def tick(self):
        with self.lock:                    # the camera worker updates the tracker too
            self._tick()

    def _tick(self):
        msg = QueueState()
        msg.header.frame_id = 'map'
        msg.zone_id = 'Q'
        fresh = time.monotonic() - self.last_receipt <= \
            self.get_parameter('stale_timeout_sec').value
        if self.last_stamp is None or not fresh:
            # Never relabel an old frame as a new observation.
            msg.header.stamp = self.last_stamp or self.get_clock().now().to_msg()
            msg.valid = False
            msg.confidence = 0.0
        else:
            msg.header.stamp = self.last_stamp
            obs_t = Time.from_msg(self.last_stamp).nanoseconds / 1e9
            msg.count = self.tracker.queue_count()
            msg.confidence = 1.0
            msg.valid = msg.confidence >= self.get_parameter('queue_confidence_min').value
            msg.crossing_occupied = self.tracker.crossing_occupied(obs_t)
        self.queue_pub.publish(msg)
        state = (msg.valid, msg.count, msg.crossing_occupied)
        if state != self.last_state:          # transitions only (6.1)
            self.event('QUEUE_' + ('COUNT' if msg.valid else 'UNKNOWN'),
                       'count=%d crossing=%s' % (msg.count, msg.crossing_occupied))
            self.last_state = state

    def event(self, name, reason=''):
        ev = HubEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.entity_id = 'passenger_detector'
        ev.event = name
        ev.reason = reason
        self.events_pub.publish(ev)


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
