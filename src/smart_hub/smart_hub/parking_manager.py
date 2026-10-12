"""
parking_manager — owner M5 (scenario S4).

Wraps parking_engine: atomic, idempotent reserve/release services, TTL expiry,
sensor reconciliation. Publishes /parking/state at 2 Hz + on change. See assignment 11.
"""
import time

from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.node import Node
from smart_hub.hub_map import PARKING_SLOTS
from smart_hub.parking_engine import ParkingEngine
from smart_hub.qos import R10, R1T
from smart_hub_interfaces.msg import ParkingSlot, ParkingState, SlotObservationArray
from smart_hub_interfaces.srv import ReleaseParking, ReserveParking


class ParkingManager(Node):

    def __init__(self):
        super().__init__('parking_manager')
        self.declare_parameter('reservation_ttl_sec', 30.0)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('debounce_sec', 0.5)

        self.engine = ParkingEngine(
            PARKING_SLOTS, freshness_sec=self.get_parameter('stale_timeout_sec').value)

        self.state_pub = self.create_publisher(ParkingState, '/parking/state', R1T)
        self.create_subscription(SlotObservationArray, '/parking/observed', self.on_observed, R10)
        self.create_service(ReserveParking, '/parking/reserve_slot', self.on_reserve)
        self.create_service(ReleaseParking, '/parking/release_slot', self.on_release)
        self.create_timer(0.5, self.publish_state)

    def on_observed(self, msg):
        now = time.monotonic()
        for obs in msg.slots:
            if obs.slot_id in self.engine.slots:
                # TODO(M5): debounce 0.5 s, confidence threshold, source-stamp age
                self.engine.observe(obs.slot_id, obs.occupied, obs.valid, now)

    def on_reserve(self, request, response):
        result = self.engine.reserve(
            request.request_id, request.vehicle_id, request.preferred_slot,
            request.ttl_sec, time.monotonic())
        response.success = result.success
        response.reservation_id = result.reservation_id
        response.slot_id = result.slot_id
        response.reason = result.reason
        # TODO(M5): convert monotonic expiry to ROS time for expires_at; publish HubEvent
        self.publish_state()
        return response

    def on_release(self, request, response):
        response.success = self.engine.release(request.reservation_id, request.vehicle_id)
        response.reason = 'OK' if response.success else 'TOKEN_OR_VEHICLE_MISMATCH'
        self.publish_state()
        return response

    def publish_state(self):
        now = time.monotonic()
        self.engine.expire(now)
        msg = ParkingState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for slot_id, slot in self.engine.slots.items():
            item = ParkingSlot()
            item.header = msg.header
            item.slot_id = slot_id
            item.state = self.engine.state(slot_id, now)
            item.vehicle_id = slot.vehicle
            item.reservation_id = slot.reservation
            item.expires_at = TimeMsg()
            msg.slots.append(item)
        msg.available_count = self.engine.available_count(now)
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
