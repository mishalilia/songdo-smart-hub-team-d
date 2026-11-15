# Author: Vladislav Li (12250245)
"""Route engine unit tests (T11/T12 seeds)."""
import math

from smart_hub.hub_map import allowed_edges, edge_id
from smart_hub.route_engine import (
    bounded_speed, cost_inputs, crossing_wait_edges, DEFAULT_SPEED_MPS, edge_cost,
    MIN_SPEED_MPS, path_cost, plan,
)


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


class State:

    def __init__(self, density=0.0, speed=0.0, blocked=False, valid=True):
        self.density_ratio, self.mean_speed_mps = density, speed
        self.blocked, self.valid = blocked, valid


def car_states(**changed):
    states = {edge_id(a, b): State() for a, b in allowed_edges('car')}
    states.update({k.replace('_', '-'): v for k, v in changed.items()})
    return states


def test_cost_terms_one_by_one():
    # c_e = L / max(v, 0.05) * (1 + 2 rho) + w, each term varied alone on a 4 m edge.
    assert math.isclose(edge_cost(4.0), 4.0 / 0.35)
    assert math.isclose(edge_cost(4.0, speed=0.20), 20.0)
    assert math.isclose(edge_cost(4.0, speed=0.20, density=0.5), 40.0)
    assert math.isclose(edge_cost(4.0, speed=0.20, density=0.5, wait=3.0), 43.0)
    assert math.isclose(edge_cost(4.0, speed=0.0), 80.0)          # floor 0.05 m/s
    assert math.isclose(edge_cost(4.0, speed=0.01), 80.0)


def test_observed_speed_is_bounded_both_ways():
    assert bounded_speed(0.0) == MIN_SPEED_MPS
    assert bounded_speed(0.20) == 0.20
    assert bounded_speed(0.90) == DEFAULT_SPEED_MPS       # traffic never beats nominal
    blocked, density, speed, wait = cost_inputs(car_states(
        Cs_J=State(density=0.5, speed=0.90), A_Cs=State(density=0.25, speed=0.01)))
    assert speed == {'Cs-J': DEFAULT_SPEED_MPS, 'A-Cs': MIN_SPEED_MPS}
    assert density['Cs-J'] == 0.5 and not blocked and not wait


def test_speed_without_traffic_is_nominal():
    # An empty edge reports speed 0; it must cost the nominal time, not the floor.
    inputs = cost_inputs(car_states())
    assert inputs[2] == {}
    path, cost = plan('A', 'E', 'car', *inputs)
    assert path == ['A', 'Cs', 'J', 'E'] and math.isclose(cost, 10.0 / 0.35)


def test_blocked_invalid_and_missing_edges_are_removed():
    states = car_states(Cs_J=State(blocked=True), B_Cn=State(valid=False))
    del states['E-J']
    blocked = cost_inputs(states, excluded=['A-N'])[0]
    assert blocked == {'Cs-J', 'B-Cn', 'E-J', 'A-N'}
    assert plan('A', 'E', 'car', blocked)[0] == []


def test_crossing_wait_is_charged_once_per_passage():
    wait = crossing_wait_edges('robot', 4.0)
    # Cs and Cn lie inside C: edges with exactly one end inside carry half the wait.
    assert wait == {'Cn-W': 2.0, 'Cs-J': 2.0, 'B-Cn': 2.0}
    free = plan('R', 'D', 'robot')[1]
    path, cost = plan('R', 'D', 'robot', wait=wait)
    assert path == ['R', 'W', 'Cn', 'Cs', 'J', 'D'] and math.isclose(cost, free + 4.0)
    detour = ['R', 'W', 'Cn', 'B', 'U', 'E', 'J', 'D']
    assert math.isclose(path_cost(detour, wait=wait), path_cost(detour) + 4.0)
    inputs = cost_inputs(car_states(), 'car', crossing_wait_sec=4.0)
    assert math.isclose(plan('A', 'E', 'car', *inputs)[1], 10.0 / 0.35 + 4.0)


def test_path_cost_matches_plan_and_rejects_unusable_edges():
    density = {'Cs-J': 0.5, 'A-Cs': 0.25}
    speed = {'Cs-J': 0.20}
    path, cost = plan('A', 'E', 'car', (), density, speed)
    assert math.isclose(path_cost(path, (), density, speed), cost)
    assert math.isinf(path_cost(['A', 'Cs', 'J'], blocked={'Cs-J'}))
    assert math.isinf(path_cost(['A', 'Cs', 'J'], speed={'Cs-J': math.nan}))
    assert math.isinf(path_cost(['A', 'Cs', 'J'], density={'Cs-J': 1.5}))
    assert plan('A', 'J', 'car', speed={'Cs-J': math.nan, 'E-J': math.inf})[0] == []


def test_snapshot_age_uses_receipt_and_observation_stamp():
    from smart_hub.route_planner import snapshot_problem
    assert snapshot_problem(0.2, 100.0, 100.3, 1.5) == ''
    assert snapshot_problem(1.6, 100.0, 100.3, 1.5) == 'STALE_RECEIPT'
    # Freshly received but observed long ago (replayed or delayed): still stale.
    assert snapshot_problem(0.1, 98.0, 100.0, 1.5) == 'STALE_STAMP'
    assert snapshot_problem(0.1, 101.0, 100.0, 1.5) == 'FUTURE_STAMP'
    assert snapshot_problem(0.1, 0.0, 100.0, 1.5) == 'NO_STAMP'


def test_expected_crossing_wait_needs_known_free_crossing():
    from smart_hub.route_planner import expected_crossing_wait

    class Crossing:

        def __init__(self, state='FREE', owner='', pedestrian=False):
            self.state, self.owner_id, self.pedestrian_present = state, owner, pedestrian

    assert expected_crossing_wait(Crossing(), 'car', 4.0) == 0.0
    assert expected_crossing_wait(None, 'car', 4.0) == 4.0              # stale / invalid
    assert expected_crossing_wait(Crossing(pedestrian=True), 'car', 4.0) == 4.0
    assert expected_crossing_wait(Crossing('OWNED', 'bus1'), 'robot', 4.0) == 4.0
    assert expected_crossing_wait(Crossing('OWNED', 'robot1'), 'robot', 4.0) == 0.0
    assert expected_crossing_wait(Crossing('HOLD'), 'car', 4.0) == 4.0
