"""Route engine unit tests (T11/T12 seeds). Owner: M6."""
import math

from smart_hub.hub_map import edge_id
from smart_hub.route_engine import plan


def test_baseline_car_route():
    path, cost = plan('A', 'E', 'car')
    assert path == ['A', 'Cs', 'J', 'E']
    assert math.isclose(cost, 10.0 / 0.35)


def test_detour_when_cs_j_blocked():
    path, _ = plan('A', 'E', 'car', blocked={edge_id('Cs', 'J')})
    assert path == ['A', 'N', 'Cn', 'B', 'U', 'E']


def test_no_route_when_both_corridors_blocked():
    path, cost = plan('A', 'E', 'car', blocked={edge_id('Cs', 'J'), edge_id('Cn', 'B')})
    assert path == []
    assert math.isinf(cost)


def test_density_changes_choice():
    lower = {edge_id('A', 'Cs'): 1.0, edge_id('Cs', 'J'): 1.0, edge_id('J', 'E'): 1.0}
    path, _ = plan('A', 'E', 'car', density=lower)
    assert path == ['A', 'N', 'Cn', 'B', 'U', 'E']


def test_bus_cannot_use_parking_edges():
    path, _ = plan('U', 'P1', 'bus')
    assert path == []


def test_robot_reference_mission():
    path, _ = plan('R', 'D', 'robot')
    assert path == ['R', 'W', 'Cn', 'Cs', 'J', 'D']
