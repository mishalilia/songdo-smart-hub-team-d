# Author: Komronbek Tohirov (12235649)
"""Crossing mutex (7.1, 18.4) and demand dispatch (9.3, T04) tests."""
from smart_hub.crossing_arbiter import CrossingArbiter, FREE, HOLD, OWNED
from smart_hub.demand_model import RidgeModel
from smart_hub.dispatch_policy import DispatchPolicy, forecast_margin, forecast_problem


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


def test_served_mission_rearms_dispatch_only_when_enabled():
    for rearm, expected in ((False, None), (True, 'PREDICTED')):
        p = DispatchPolicy(high=6, low=4, cooldown_sec=30, rearm_after_service=rearm)
        assert [p.update(t, True, 8.0, True, 8, True, False) for t in range(3)][-1] == \
            'PREDICTED'
        assert p.served() is rearm
        calls = [p.update(t, True, 8.0, True, 8, True, False) for t in range(40, 43)]
        assert calls[:2] == [None, None]                # three new high predictions first
        assert calls[-1] == expected
        assert p.update(43, True, 8.0, True, 8, True, False) is None


def test_observed_floor_never_lets_the_forecast_delay_a_dispatch():
    late = DispatchPolicy(high=6, low=4, cooldown_sec=30)
    floor = DispatchPolicy(high=6, low=4, cooldown_sec=30, observed_floor=True)
    # forecast still below 6 while seven people already wait
    assert late.update(0, True, 4.5, True, 7, True, False) is None
    assert floor.update(0, True, 4.5, True, 7, True, False) == 'OBSERVED'
    # a stale or invalid queue gives no floor
    other = DispatchPolicy(high=6, low=4, cooldown_sec=30, observed_floor=True)
    assert other.update(0, True, 4.5, False, 7, True, False) is None
    # three high forecasts still dispatch before the queue reaches 6
    early = DispatchPolicy(high=6, low=4, cooldown_sec=30, observed_floor=True)
    assert [early.update(t, True, 7.0, True, 3, True, False) for t in range(3)][-1] == \
        'PREDICTED'


def test_forecast_margin_dispatches_on_the_upper_bound_only():
    point = DispatchPolicy(high=6, low=4, cooldown_sec=30, observed_floor=True)
    upper = DispatchPolicy(high=6, low=4, cooldown_sec=30, observed_floor=True)
    # forecast 4.8 with five waiting: the point forecast waits, forecast + 1.66 crosses 6
    assert [point.update(t, True, 4.8, True, 5, True, False) for t in range(3)] == [None] * 3
    assert [upper.update(t, True, 4.8, True, 5, True, False, 1.66)
            for t in range(3)][-1] == 'PREDICTED'
    # an invalid forecast ignores the margin and falls back to the observed queue
    fb = DispatchPolicy(high=6, low=4, cooldown_sec=30)
    assert fb.update(0, False, 4.8, True, 5, True, False, 1.66) is None
    # escalation still clears on the point forecast (five values below 4)
    clear = DispatchPolicy(high=6, low=4, cooldown_sec=0, consecutive=1)
    assert clear.update(0, True, 5.0, True, 0, True, False, 1.66) == 'PREDICTED'
    for t in range(1, 6):
        clear.update(t, True, 3.0, True, 0, True, False, 1.66)
    assert not clear.escalated


def test_forecast_margin_comes_from_the_installed_models_validation_residuals(tmp_path):
    model = RidgeModel([1.0], 0.0, [0.0], [1.0], 0.01, 'ridge-test', {'0.8': 1.66})
    path = str(tmp_path / 'model.json')
    model.save(path)
    assert forecast_margin(path, 0.8) == ('ridge-test', 1.66)
    assert forecast_margin(path, 0.0) == ('', 0.0)            # disabled
    assert forecast_margin(path, 0.95) == ('', 0.0)           # quantile not calibrated
    assert forecast_margin(str(tmp_path / 'missing.json'), 0.8) == ('', 0.0)


def test_dispatch_fallback_and_invalid_inputs():
    p = DispatchPolicy()
    assert p.update(0, False, 0.0, False, 0, True, False) is None     # nothing valid
    assert p.update(1, False, 0.0, True, 8, True, False) == 'FALLBACK'
    q = DispatchPolicy()
    assert [q.update(t, True, 7.0, True, 7, False, False) for t in range(5)] == [None] * 5


def test_forecast_is_gated_on_source_and_acquisition_age():
    now = 500.0
    assert forecast_problem(True, 30, 'ridge', now - 1.0, now) == ''
    assert forecast_problem(False, 30, 'ridge', now - 1.0, now) == 'INVALID'
    assert forecast_problem(True, 60, 'ridge', now - 1.0, now) == 'WRONG_SOURCE'
    assert forecast_problem(True, 30, '', now - 1.0, now) == 'WRONG_SOURCE'
    assert forecast_problem(True, 30, 'ridge', now - 3.5, now) == 'STALE_STAMP'
    assert forecast_problem(True, 30, 'ridge', now + 1.0, now) == 'FUTURE_STAMP'
    assert forecast_problem(True, 30, 'ridge', 0.0, now) == 'NO_STAMP'
    assert forecast_problem(True, 30, 'ridge', now - 1.0, now, last_stamp=now - 0.5) == \
        'OUT_OF_ORDER'
    # A rejected forecast never dispatches by itself; the fresh queue is the fallback.
    p = DispatchPolicy()
    stale = forecast_problem(True, 30, 'ridge', now - 10.0, now)
    assert [p.update(t, not stale, 9.0, False, 0, True, False) for t in range(5)] == [None] * 5
    assert p.update(6, not stale, 9.0, True, 8, True, False) == 'FALLBACK'
