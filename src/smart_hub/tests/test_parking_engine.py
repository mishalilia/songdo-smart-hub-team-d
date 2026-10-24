# Author: Nikita Pak (12250247)
"""Parking engine unit tests (T07 / T08)."""
import threading

from smart_hub.parking_engine import FREE, OCCUPIED, ParkingEngine, RESERVED, UNKNOWN


def make_engine(now=10.0):
    """P1 occupied, P2 / P3 free, readings stable for the 0.5 s debounce."""
    e = ParkingEngine()
    for t in (now - 0.6, now):
        e.observe('P1', occupied=True, valid=True, now=t)
        e.observe('P2', occupied=False, valid=True, now=t)
        e.observe('P3', occupied=False, valid=True, now=t)
    return e


def test_debounce_and_unknown_until_stable():
    e = ParkingEngine()
    assert e.state('P1', now=0.0) == UNKNOWN
    e.observe('P1', occupied=False, valid=True, now=1.0)
    assert e.state('P1', now=1.0) == UNKNOWN          # not yet stable
    assert e.observe('P1', occupied=False, valid=True, now=1.5) == FREE
    e.observe('P1', occupied=True, valid=True, now=1.7)
    assert e.state('P1', now=1.7) == FREE              # a short flicker is ignored
    e.observe('P1', occupied=False, valid=False, now=1.8)
    assert e.state('P1', now=1.8) == UNKNOWN           # invalid reading: UNKNOWN at once


def test_concurrent_preferred_slot_is_exclusive():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', 'P2', 30, now=10.0)
    b = e.reserve('c2-001', 'C2', 'P2', 30, now=10.0)
    assert a.success and a.slot_id == 'P2'
    assert not b.success and b.reason == 'PREFERRED_UNAVAILABLE'   # strict preference


def test_twenty_threaded_request_pairs_never_double_allocate():
    for trial in range(20):
        e = make_engine()
        results = []
        threads = [threading.Thread(target=lambda c=c: results.append(
            e.reserve('%s-%d' % (c, trial), c, '', 30, now=10.0))) for c in ('C1', 'C2')]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        slots = [r.slot_id for r in results if r.success]
        assert len(slots) == len(set(slots)) == 2       # P3 and P2, never the same


def test_nearest_slot_when_no_preference():
    e = make_engine()
    assert e.reserve('r1', 'C1', '', 30, now=10.0).slot_id == 'P3'   # next to entrance U


def test_duplicate_request_is_idempotent():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', 'P2', 30, now=10.0)
    assert e.reserve('c1-001', 'C1', 'P2', 30, now=10.5) == a
    assert e.reserve('c1-001', 'C1', 'P3', 30, now=10.5).reason == 'KEY_REUSE'
    assert e.state('P3', now=10.5) == FREE                           # no second slot


def test_release_needs_token_and_vehicle_and_is_idempotent():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', '', 30, now=10.0)
    assert e.release('r1', 'bogus', 'C1') == (False, 'TOKEN_OR_VEHICLE_MISMATCH')
    assert not e.release('r2', a.reservation_id, 'C2')[0]
    assert e.release('r3', a.reservation_id, 'C1') == (True, 'OK')
    assert e.release('r3', a.reservation_id, 'C1') == (True, 'OK')  # same request id
    assert e.release('r3', 'other', 'C1')[1] == 'KEY_REUSE'


def test_unknown_and_stale_never_free():
    e = make_engine()
    assert not e.reserve('r1', 'car', 'P2', 30, now=12.0).success  # stale after 1 s
    full = make_engine()
    full.reserve('a', 'C1', '', 30, now=10.0)
    full.reserve('b', 'C2', '', 30, now=10.0)
    assert full.reserve('c', 'C3', '', 30, now=10.0).reason == 'NO_SLOT'


def test_expiry_keeps_occupied_slot_and_reports_token():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', 'P2', 5, now=10.0)
    assert e.state('P2', now=10.0) == RESERVED
    e.observe('P2', occupied=True, valid=True, now=10.2)
    e.observe('P2', occupied=True, valid=True, now=10.7)
    assert e.state('P2', now=10.7) == OCCUPIED and e.slots['P2'].reservation == a.reservation_id
    assert e.expire(now=15.5) == [('P2', a.reservation_id, 'C1')]
    e.observe('P2', occupied=True, valid=True, now=15.6)
    assert e.state('P2', now=15.6) == OCCUPIED                     # never falsely FREE


def test_invalid_requests_rejected():
    e = make_engine()
    assert e.reserve('x', 'C1', '', 1, now=10.0).reason == 'INVALID_REQUEST'
    assert e.reserve('y', 'C1', 'P9', 30, now=10.0).reason == 'UNKNOWN_SLOT'
