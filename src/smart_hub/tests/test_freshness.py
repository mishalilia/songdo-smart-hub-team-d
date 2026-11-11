# Author: Mikhail Li (12236295)
"""Freshness gate for retained state: validity, receipt age and observation-stamp age."""
from types import SimpleNamespace

from smart_hub.freshness import Retained, stamp_problem, stamp_sec


def message(stamp, valid=True):
    sec = int(stamp)
    header = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp - sec) * 1e9)))
    return SimpleNamespace(header=header, valid=valid)


def test_stamp_problem_reasons():
    assert stamp_problem(99.5, 100.0, 1.0) == ''
    assert stamp_problem(0.0, 100.0, 1.0) == 'NO_STAMP'
    assert stamp_problem(98.0, 100.0, 1.0) == 'STALE_STAMP'
    assert stamp_problem(100.1, 100.0, 1.0) == ''              # inside the declared tolerance
    assert stamp_problem(100.5, 100.0, 1.0) == 'FUTURE_STAMP'
    assert stamp_problem(99.5, 100.0, 1.0, last_stamp=99.8) == 'OUT_OF_ORDER'
    assert stamp_problem(99.5, 100.0, 1.0, last_stamp=99.5) == 'REPLAY'


def test_retained_sample_is_not_fresh_by_itself():
    r = Retained(max_age_sec=1.0)
    assert r.problem(100.0, 10.0) == 'MISSING' and r.get(100.0, 10.0) is None
    msg = message(99.9)
    r.put(msg, 10.0)
    assert r.get(100.0, 10.1) is msg
    assert abs(stamp_sec(msg.header.stamp) - 99.9) < 1e-6
    assert r.problem(100.0, 11.5) == 'STALE_RECEIPT'       # latched long ago
    r.put(message(95.0), 12.0)                              # fresh receipt, old observation
    assert r.problem(100.0, 12.1) == 'STALE_STAMP'
    r.put(message(103.0), 12.0)
    assert r.problem(100.0, 12.1) == 'FUTURE_STAMP'
    r.put(message(99.9, valid=False), 12.0)
    assert r.problem(100.0, 12.1) == 'INVALID'
    r.clear()
    assert r.problem(100.0, 12.1) == 'MISSING'
