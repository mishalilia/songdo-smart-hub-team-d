# Author: Nikita Pak (12250247)
"""
Thread-safe, idempotent parking reservation logic (assignment 11.2, 18.2).

Started from the handbook's companion engine (appendix B), extended with:
- 0.5 s debounce: an occupancy change is accepted only after stable valid readings;
  an invalid reading makes the slot UNKNOWN at once, a stale one after `freshness_sec`
- nearest feasible slot to the parking entrance when no preference is given
- reservation metadata kept while a reserved slot is physically OCCUPIED
- request-id idempotency for both reserve and release (same id + payload -> same result,
  same id + different payload -> KEY_REUSE)
- expiry releases only the administrative reservation and reports what expired
Time is always passed in (monotonic seconds) so tests stay deterministic.
"""
from dataclasses import dataclass
import math
from threading import RLock
from uuid import uuid4

from smart_hub import hub_map

FREE, RESERVED, OCCUPIED, UNKNOWN = 'FREE', 'RESERVED', 'OCCUPIED', 'UNKNOWN'
TTL_MIN, TTL_MAX = 5.0, 120.0
ENTRANCE = 'U'                  # parking access edge U-P3-P2-P1


@dataclass
class Slot:
    observed: str = UNKNOWN     # debounced sensing: FREE / OCCUPIED / UNKNOWN
    observed_at: float = -math.inf
    candidate: str = UNKNOWN    # latest raw reading waiting for the debounce
    candidate_since: float = -math.inf
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

    def __init__(self, slots=('P1', 'P2', 'P3'), freshness_sec=1.0, debounce_sec=0.5):
        self.slots = {k: Slot() for k in slots}
        self.freshness_sec = freshness_sec
        self.debounce_sec = debounce_sec
        self._reserve_cache = {}
        self._release_cache = {}
        self._lock = RLock()

    def observe(self, slot_id, occupied, valid, now):
        """Return the new debounced state if it changed, else None."""
        with self._lock:
            s = self.slots[slot_id]
            s.observed_at = now
            raw = UNKNOWN if not valid else (OCCUPIED if occupied else FREE)
            if raw != s.candidate:
                s.candidate, s.candidate_since = raw, now
            accept = raw == UNKNOWN or now - s.candidate_since >= self.debounce_sec - 1e-9
            if accept and raw != s.observed:
                s.observed = raw
                return raw
            return None

    def state(self, slot_id, now):
        s = self.slots[slot_id]
        if now - s.observed_at > self.freshness_sec or s.observed == UNKNOWN:
            return UNKNOWN
        if s.observed == OCCUPIED:
            return OCCUPIED            # reservation metadata, if any, is kept
        if s.reservation and s.expiry > now:
            return RESERVED
        return FREE

    def available_count(self, now):
        with self._lock:
            return sum(1 for k in self.slots if self.state(k, now) == FREE)

    @staticmethod
    def _nearest(candidates):
        entrance = hub_map.NODES[ENTRANCE]
        return min(candidates,
                   key=lambda k: math.dist(hub_map.NODES.get(k, entrance), entrance))

    def is_duplicate(self, request_id):
        with self._lock:
            return request_id in self._reserve_cache

    def reserve(self, request_id, vehicle_id, preferred_slot, ttl_sec, now):
        payload = (vehicle_id, preferred_slot, float(ttl_sec))
        with self._lock:
            if request_id in self._reserve_cache:
                old_payload, result = self._reserve_cache[request_id]
                if old_payload != payload:
                    return ReserveResult(False, reason='KEY_REUSE')
                return result
            if not request_id or not vehicle_id or not TTL_MIN <= ttl_sec <= TTL_MAX:
                return ReserveResult(False, reason='INVALID_REQUEST')
            self.expire(now)
            if preferred_slot and preferred_slot not in self.slots:
                result = ReserveResult(False, reason='UNKNOWN_SLOT')
            elif any(s.reservation and s.vehicle == vehicle_id for s in self.slots.values()):
                result = ReserveResult(False, reason='VEHICLE_ALREADY_RESERVED')
            else:
                free = [k for k in self.slots if self.state(k, now) == FREE]
                if preferred_slot:
                    # Strict preference policy: the preferred slot or an explicit rejection.
                    chosen = preferred_slot if preferred_slot in free else ''
                else:
                    chosen = self._nearest(free) if free else ''
                if not chosen:
                    result = ReserveResult(
                        False, reason='NO_SLOT' if not free else 'PREFERRED_UNAVAILABLE')
                else:
                    s = self.slots[chosen]
                    s.reservation, s.vehicle, s.expiry = uuid4().hex, vehicle_id, now + ttl_sec
                    result = ReserveResult(True, s.reservation, chosen, s.expiry, 'OK')
            self._reserve_cache[request_id] = (payload, result)
            return result

    def release(self, request_id, reservation_id, vehicle_id):
        """Return (success, reason); idempotent per request id."""
        payload = (reservation_id, vehicle_id)
        with self._lock:
            if request_id and request_id in self._release_cache:
                old_payload, result = self._release_cache[request_id]
                return result if old_payload == payload else (False, 'KEY_REUSE')
            result = (False, 'TOKEN_OR_VEHICLE_MISMATCH')
            for s in self.slots.values():
                if reservation_id and s.reservation == reservation_id and \
                        s.vehicle == vehicle_id:
                    s.reservation, s.vehicle, s.expiry = '', '', 0.0
                    result = (True, 'OK')
                    break
            if request_id:
                self._release_cache[request_id] = (payload, result)
            return result

    def expire(self, now):
        """Drop expired administrative reservations; never marks an occupied slot FREE."""
        expired = []
        with self._lock:
            for slot_id, s in self.slots.items():
                if s.reservation and s.expiry <= now:
                    expired.append((slot_id, s.reservation, s.vehicle))
                    s.reservation, s.vehicle, s.expiry = '', '', 0.0
        return expired
