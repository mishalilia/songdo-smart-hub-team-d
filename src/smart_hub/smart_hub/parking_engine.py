"""
Thread-safe, idempotent parking reservation logic (assignment 11, 18.2). Owner: M5.

Based on the handbook's companion parking engine (appendix B). The ROS node wraps this
class; time is always passed in (monotonic seconds) so tests stay deterministic.
TODO(M5): 0.5 s observation debounce, keep reservation metadata when a slot turns
          OCCUPIED, nearest-slot selection, request-id cache for release, expires_at stamps.
"""
from dataclasses import dataclass
import math
from threading import RLock
from uuid import uuid4

FREE, RESERVED, OCCUPIED, UNKNOWN = 'FREE', 'RESERVED', 'OCCUPIED', 'UNKNOWN'
TTL_MIN, TTL_MAX = 5.0, 120.0


@dataclass
class Slot:
    observed: str = UNKNOWN     # FREE / OCCUPIED / UNKNOWN (raw sensing)
    observed_at: float = -math.inf
    reservation: str = ''
    vehicle: str = ''
    expiry: float = 0.0


@dataclass(frozen=True)
class ReserveResult:
    success: bool
    reservation_id: str = ''
    slot_id: str = ''
    expiry: float = 0.0
    reason: str = ''


class ParkingEngine:

    def __init__(self, slots=('P1', 'P2', 'P3'), freshness_sec=1.0):
        self.slots = {k: Slot() for k in slots}
        self.freshness_sec = freshness_sec
        self._cache = {}
        self._lock = RLock()

    def observe(self, slot_id, occupied, valid, now):
        with self._lock:
            s = self.slots[slot_id]
            s.observed = UNKNOWN if not valid else (OCCUPIED if occupied else FREE)
            s.observed_at = now

    def state(self, slot_id, now):
        s = self.slots[slot_id]
        if now - s.observed_at > self.freshness_sec or s.observed == UNKNOWN:
            return UNKNOWN
        if s.observed == OCCUPIED:
            return OCCUPIED
        if s.reservation and s.expiry > now:
            return RESERVED
        return FREE

    def available_count(self, now):
        with self._lock:
            return sum(1 for k in self.slots if self.state(k, now) == FREE)

    def reserve(self, request_id, vehicle_id, preferred_slot, ttl_sec, now):
        payload = (vehicle_id, preferred_slot, float(ttl_sec))
        with self._lock:
            if request_id in self._cache:
                old_payload, result = self._cache[request_id]
                if old_payload != payload:
                    return ReserveResult(False, reason='KEY_REUSE')
                return result
            if not request_id or not vehicle_id or not TTL_MIN <= ttl_sec <= TTL_MAX:
                return ReserveResult(False, reason='INVALID_REQUEST')
            self.expire(now)
            if any(s.reservation and s.vehicle == vehicle_id for s in self.slots.values()):
                result = ReserveResult(False, reason='VEHICLE_ALREADY_RESERVED')
            else:
                free = [k for k in self.slots if self.state(k, now) == FREE]
                if preferred_slot:
                    chosen = preferred_slot if preferred_slot in free else ''
                else:
                    chosen = free[0] if free else ''
                if not chosen:
                    result = ReserveResult(False, reason='NO_SLOT')
                else:
                    s = self.slots[chosen]
                    s.reservation, s.vehicle, s.expiry = uuid4().hex, vehicle_id, now + ttl_sec
                    result = ReserveResult(True, s.reservation, chosen, s.expiry, 'OK')
            self._cache[request_id] = (payload, result)
            return result

    def release(self, reservation_id, vehicle_id):
        with self._lock:
            for s in self.slots.values():
                if reservation_id and s.reservation == reservation_id and s.vehicle == vehicle_id:
                    s.reservation, s.vehicle, s.expiry = '', '', 0.0
                    return True
            return False

    def expire(self, now):
        """Drop administrative reservations only; never mark an occupied slot FREE."""
        with self._lock:
            for s in self.slots.values():
                if s.reservation and s.expiry <= now:
                    s.reservation, s.vehicle, s.expiry = '', '', 0.0
