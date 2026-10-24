# Author: Vladislav Li (12250245)
"""S5 traffic estimation tests (T09 / T10)."""
import math

from smart_hub.traffic_logic import TrafficEstimator


def test_traffic_density_blocked_and_stale():
    est = TrafficEstimator(headway_m=1.0)
    for i in range(3):
        est.add('c%d' % i, 'car', 6.0 + i, 2.0, 0.1, 'Cs-J', True, 10.0, 10.0)
    est.add('O', 'obstacle', 7.0, 2.0, 0.0, 'Cs-J', True, 10.0, 10.0)
    by_id = {e.edge_id: e for e in est.estimate(10.0)}
    assert math.isclose(by_id['Cs-J'].density_ratio, 3 / 4)
    assert by_id['Cs-J'].blocked and by_id['Cs-J'].valid
    assert not by_id['A-N'].blocked
    stale = {e.edge_id: e for e in est.estimate(12.0)}
    assert not stale['Cs-J'].valid and not stale['A-N'].valid


def test_traffic_rejects_impossible_samples():
    est = TrafficEstimator()
    assert not est.add('c', 'car', 1.0, 7.0, 0.3, 'Cs-J', True, 0.0, 0.0)   # outside corridor
    assert not est.add('c', 'car', 6.0, 2.0, 0.3, 'X-Y', True, 0.0, 0.0)     # bad edge id
    assert not est.add('c', 'car', 6.0, 2.0, -1.0, 'Cs-J', True, 0.0, 0.0)   # negative speed
