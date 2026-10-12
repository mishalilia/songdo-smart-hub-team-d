"""Parking engine unit tests (T07/T08 seeds). Owner: M5."""
from smart_hub.parking_engine import OCCUPIED, ParkingEngine, UNKNOWN


def make_engine(now=10.0):
    e = ParkingEngine()
    e.observe('P1', occupied=True, valid=True, now=now)
    e.observe('P2', occupied=False, valid=True, now=now)
    e.observe('P3', occupied=False, valid=True, now=now)
    return e


def test_concurrent_preferred_slot_is_exclusive():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', 'P2', 30, now=10.0)
    b = e.reserve('c2-001', 'C2', 'P2', 30, now=10.0)
    assert a.success and a.slot_id == 'P2'
    assert not b.success  # strict-preference policy


def test_duplicate_request_is_idempotent():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', 'P2', 30, now=10.0)
    assert e.reserve('c1-001', 'C1', 'P2', 30, now=10.5) == a
    assert e.reserve('c1-001', 'C1', 'P3', 30, now=10.5).reason == 'KEY_REUSE'


def test_wrong_token_release_rejected():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', '', 30, now=10.0)
    assert not e.release('bogus', 'C1')
    assert not e.release(a.reservation_id, 'C2')
    assert e.release(a.reservation_id, 'C1')


def test_unknown_and_stale_never_free():
    e = ParkingEngine()
    assert e.state('P1', now=0.0) == UNKNOWN
    e.observe('P2', occupied=False, valid=True, now=10.0)
    assert not e.reserve('r1', 'car', 'P2', 30, now=12.0).success  # stale after 1 s


def test_expiry_does_not_free_occupied_slot():
    e = make_engine()
    a = e.reserve('c1-001', 'C1', 'P2', 5, now=10.0)
    e.observe('P2', occupied=True, valid=True, now=10.5)
    e.expire(now=16.0)
    assert e.state(a.slot_id, now=10.6) == OCCUPIED


def test_invalid_ttl_rejected():
    e = make_engine()
    assert e.reserve('x', 'C1', '', 1, now=10.0).reason == 'INVALID_REQUEST'
