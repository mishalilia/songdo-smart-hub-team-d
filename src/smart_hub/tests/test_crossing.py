# Author: Komronbek Tohirov (12235649)
"""Crossing mutex (7.1, 18.4) and demand dispatch (9.3, T04) tests."""
from smart_hub.crossing_arbiter import CrossingArbiter, FREE, HOLD, OWNED
from smart_hub.dispatch_policy import DispatchPolicy


def test_crossing_single_owner_and_pedestrian_priority():
    arb = CrossingArbiter(clear_sec=2.0, lease_sec=2.0)
    arb.observe(0.0, False, True, set())
    assert arb.request('robot1', 'ACQUIRE', '', 1.0)[2] == 'CLEAR_DEBOUNCE'
    arb.observe(2.1, False, True, set())
    granted, token, _ = arb.request('robot1', 'ACQUIRE', '', 2.1)
    assert granted and arb.state == OWNED
    assert arb.request('bus1', 'ACQUIRE', '', 2.2)[2].startswith('OCCUPIED')
    assert not arb.request('bus1', 'RELEASE', 'bogus', 2.2)[0]
    arb.observe(2.3, False, True, {'robot1'})
    assert arb.request('robot1', 'RELEASE', token, 2.3)[2] == 'NOT_CLEAR'
    arb.observe(2.5, False, True, set())
    assert arb.request('robot1', 'RELEASE', token, 2.5)[0]
    assert arb.state == FREE
    arb.observe(5.0, True, True, set())
    assert arb.request('bus1', 'ACQUIRE', '', 5.0)[2] == 'PEDESTRIAN_PRESENT'


def test_crossing_lost_owner_holds_until_clear():
    arb = CrossingArbiter(clear_sec=1.0, lease_sec=1.0)
    arb.observe(0.0, False, True, set())
    arb.observe(1.0, False, True, set())
    assert arb.request('robot1', 'ACQUIRE', '', 1.0)[0]
    arb.observe(3.0, False, True, {'robot1'})       # lease expired while inside C
    assert arb.state == HOLD
    assert not arb.request('bus1', 'ACQUIRE', '', 3.0)[0]
    arb.observe(3.5, False, True, set())
    arb.observe(4.6, False, True, set())
    assert arb.state == FREE


def test_crossing_unknown_denies():
    arb = CrossingArbiter()
    arb.observe(0.0, False, False, set())
    assert arb.request('robot1', 'ACQUIRE', '', 5.0)[2] == 'CROSSING_UNKNOWN'


def test_dispatch_needs_three_high_predictions_and_one_outstanding():
    p = DispatchPolicy(high=6, low=4, cooldown_sec=30)
    calls = [p.update(t, True, 7.0, True, 7, True, False) for t in range(3)]
    assert calls == [None, None, 'PREDICTED']
    assert p.update(4, True, 9.0, True, 9, True, True) is None        # outstanding
    assert p.update(40, True, 9.0, True, 9, True, False) is None      # still escalated
    for t in range(41, 46):
        p.update(t, True, 2.0, True, 2, True, False)                   # 5 below dispatch_low
    assert [p.update(t, True, 7.0, True, 7, True, False) for t in range(46, 49)][-1] == \
        'PREDICTED'


def test_dispatch_fallback_and_invalid_inputs():
    p = DispatchPolicy()
    assert p.update(0, False, 0.0, False, 0, True, False) is None     # nothing valid
    assert p.update(1, False, 0.0, True, 8, True, False) == 'FALLBACK'
    q = DispatchPolicy()
    assert [q.update(t, True, 7.0, True, 7, False, False) for t in range(5)] == [None] * 5
