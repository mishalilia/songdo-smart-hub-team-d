# Author: Nikita Pak (12250247)
"""
parking_manager (scenario S4).

Wraps parking_engine (assignment 11): debounced, freshness-checked slot sensing; atomic,
idempotent /parking/reserve_slot and /parking/release_slot; TTL expiry that never frees an
occupied slot. Publishes /parking/state at 2 Hz and on every change (expires_at in ROS
time) and reservation transitions on /hub/events with the token as correlation id and the
service handling latency.
"""
import time

from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from smart_hub.hub_events import ack_reset
from smart_hub.hub_map import PARKING_SLOTS
from smart_hub.params import check_parameters
from smart_hub.parking_engine import ParkingEngine
from smart_hub.qos import R1, R10, R100, R1T
from smart_hub_interfaces.msg import HubEvent, ParkingSlot, ParkingState, SlotObservationArray
from smart_hub_interfaces.srv import ReleaseParking, ReserveParking
from std_msgs.msg import Bool, String


class ParkingManager(Node):

    def __init__(self):
        super().__init__('parking_manager')
        self.declare_parameter('reservation_ttl_sec', 30.0)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('debounce_sec', 0.5)
        self.declare_parameter('confidence_min', 0.5)
        self.declare_parameter('barrier_open_sec', 5.0)
        check_parameters(self)

        self.engine = self.new_engine()
        self.last_states = None             # last states reported as SLOT_* events
        self.run_id = ''
        self.barrier_open = False
        self.barrier_close_at = None
        self.granted_vehicle = ''
        self.granted_seen = -1e9
        self.denied_at = {}

        self.state_pub = self.create_publisher(ParkingState, '/parking/state', R1T)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(SlotObservationArray, '/parking/observed', self.on_observed, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.barrier_pub = self.create_publisher(Bool, '/parking/barrier_request', R1)
        self.create_subscription(String, '/parking/entrance', self.on_entrance, R10)
        self.create_subscription(Bool, '/parking/barrier_open', self.on_barrier, R1)
        self.create_service(ReserveParking, '/parking/reserve_slot', self.on_reserve)
        self.create_service(ReleaseParking, '/parking/release_slot', self.on_release)
        self.create_timer(0.5, self.publish_state)
        self.create_timer(0.1, self.check_changes)
        self.create_timer(0.1, self.close_barrier)

    def new_engine(self):
        return ParkingEngine(PARKING_SLOTS,
                             freshness_sec=self.get_parameter('stale_timeout_sec').value,
                             debounce_sec=self.get_parameter('debounce_sec').value)

    def event(self, entity_id, name, reason='', correlation_id=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.run_id = self.run_id
        msg.entity_id = entity_id
        msg.event = name
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events_pub.publish(msg)

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.run_id = msg.run_id
            self.engine = self.new_engine()     # administrative resources cleared on reset
        ack_reset(self, self.events_pub, msg)

    def on_observed(self, msg):
        now = time.monotonic()
        for obs in msg.slots:
            if obs.slot_id not in self.engine.slots:
                continue
            valid = obs.valid and obs.confidence >= self.get_parameter('confidence_min').value
            self.engine.observe(obs.slot_id, obs.occupied, valid, now)

    def on_entrance(self, msg):
        """Open the barrier only for the owner of a valid reservation (11.3)."""
        now = time.monotonic()
        vehicle = msg.data
        slot = next((k for k, s in self.engine.slots.items()
                     if s.vehicle == vehicle and self.engine.state(k, now) == 'RESERVED'), '')
        if slot:
            if self.granted_vehicle != vehicle:
                self.event(vehicle, 'BARRIER_GRANTED', slot,
                           self.engine.slots[slot].reservation)
            self.granted_vehicle, self.granted_seen = vehicle, now
            self.barrier_pub.publish(Bool(data=True))
            self.barrier_close_at = now + self.get_parameter('barrier_open_sec').value
            return
        if now - self.denied_at.get(vehicle, -1e9) > 5.0:
            self.denied_at[vehicle] = now
            self.event(vehicle, 'BARRIER_DENIED', 'NO_VALID_RESERVATION')
        if self.barrier_open or self.barrier_close_at is not None:
            self.barrier_close_at = now               # no tailgating: close right away

    def on_barrier(self, msg):
        self.barrier_open = bool(msg.data)

    def close_barrier(self):
        """Request closing after the open period; the device refuses on an occupied lane."""
        now = time.monotonic()
        if self.granted_vehicle and now - self.granted_seen > 1.0:
            # The owner has left the entrance sensor (driven through): close now.
            self.granted_vehicle = ''
            self.barrier_close_at = min(self.barrier_close_at or now, now)
        if self.barrier_close_at is None or now < self.barrier_close_at:
            return
        if self.barrier_open:
            self.barrier_pub.publish(Bool(data=False))
            self.barrier_close_at = now + 0.2          # retry until closed (occupied lane)
        else:
            self.barrier_close_at = None

    def expires_at(self, expiry_monotonic):
        """Convert a monotonic expiry into ROS time for ParkingSlot / ReserveParking."""
        if not expiry_monotonic:
            return TimeMsg()
        remaining = max(0.0, expiry_monotonic - time.monotonic())
        return (self.get_clock().now() + Duration(seconds=remaining)).to_msg()

    def on_reserve(self, request, response):
        start = time.monotonic()
        duplicate = self.engine.is_duplicate(request.request_id)
        result = self.engine.reserve(request.request_id, request.vehicle_id,
                                     request.preferred_slot, request.ttl_sec, start)
        response.success = result.success
        response.reservation_id = result.reservation_id
        response.slot_id = result.slot_id
        response.expires_at = self.expires_at(result.expiry)
        response.reason = result.reason
        latency = 'latency_ms=%.2f' % ((time.monotonic() - start) * 1000.0)
        if duplicate:
            self.event(request.vehicle_id, 'RESERVE_DUPLICATE', '%s %s' % (
                result.reason, latency), result.reservation_id or request.request_id)
        elif result.success:
            self.event(request.vehicle_id, 'SLOT_RESERVED', '%s ttl=%.0fs %s' % (
                result.slot_id, request.ttl_sec, latency), result.reservation_id)
        else:
            self.event(request.vehicle_id, 'RESERVATION_REJECTED',
                       '%s %s' % (result.reason, latency), request.request_id)
        self.publish_state()
        return response

    def on_release(self, request, response):
        start = time.monotonic()
        response.success, response.reason = self.engine.release(
            request.request_id, request.reservation_id, request.vehicle_id)
        self.event(request.vehicle_id,
                   'SLOT_RELEASED' if response.success else 'RELEASE_REJECTED',
                   '%s latency_ms=%.2f' % (response.reason, (time.monotonic() - start) * 1000.0),
                   request.reservation_id)
        self.publish_state()
        return response

    def check_changes(self):
        """Publish immediately when a slot state changes (debounce, staleness, expiry)."""
        now = time.monotonic()
        for slot_id, token, vehicle in self.engine.expire(now):
            self.event(vehicle, 'RESERVATION_EXPIRED', slot_id, token)
        states = tuple(self.engine.state(s, now) for s in self.engine.slots)
        if states != self.last_states:
            for slot_id, old, new in zip(self.engine.slots, self.last_states or
                                         ('',) * len(states), states):
                if old != new:   # debounced sensing, staleness, reservation or expiry
                    self.event(slot_id, 'SLOT_' + new, 'was ' + (old or 'none'))
            self.last_states = states
            self.publish_state()

    def publish_state(self):
        now = time.monotonic()
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
            item.expires_at = self.expires_at(slot.expiry)
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
