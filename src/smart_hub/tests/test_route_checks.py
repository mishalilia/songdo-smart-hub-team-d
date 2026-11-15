# Author: Mikhail Li (12236295)
"""Route / replan assertions of the test matrix and the robot's blocked-edge helpers."""
import math

from smart_hub.metrics import StopMonitor
from smart_hub.robot_manager import blocked_ahead, trim_turn_back
from smart_hub.test_matrix import (
    blocked_edges_at, check_escort_ok, free_flow_cost, path_problems, replan_chain,
    route_reply,
)


class Edge:

    def __init__(self, edge_id, blocked=False, valid=True):
        self.edge_id, self.blocked, self.valid = edge_id, blocked, valid


def event(t, entity, name, reason='', correlation=''):
    return {'t': t, 't_sec': '%.3f' % t, 'entity_id': entity, 'event': name,
            'reason': reason, 'correlation_id': correlation}


def escort_events(obstacle_t=30.0, new_route='ESCORT Cn-B-U-E-J-D cost=37.1s gen=3',
                  stop_ms=100):
    return [
        event(2.1, 'robot1', 'ROUTE', 'TO_PICKUP R cost=0.0s gen=1', 'demo-01'),
        event(6.2, 'robot1', 'ROUTE', 'ESCORT R-W-Cn-Cs-J-D cost=31.4s gen=2', 'demo-01'),
        event(obstacle_t, 'scenario_runner', 'INJECT', 'place_obstacle'),
        event(obstacle_t + 0.05, 'Cs-J', 'EDGE_BLOCKED', 'density=0.00 speed=0.00 n=0'),
        event(obstacle_t + 0.10, 'robot1', 'REROUTE',
              'edge Cs-J blocked old=Cn-Cs-J-D from=Cs gen=2', 'demo-01'),
        event(obstacle_t + 0.10 + stop_ms / 1000.0, 'robot1', 'ROUTE_STOP_MEASURED',
              'cause=REROUTE latency_ms=%d' % stop_ms, 'ground_truth'),
        event(obstacle_t + 0.45, 'robot1', 'ROUTE', new_route, 'demo-01'),
        event(90.0, 'robot1', 'ARRIVED', 'error_m=0.031', 'demo-01'),
        event(90.1, 'robot1', 'GOAL_SUCCEED', 'ARRIVED', 'demo-01'),
    ]


SAFE = {'counts': {'collisions': 0}, 'crossing': {
    'strict_occupancy': {'by_class': {'controlled_controlled': {'episodes': 0},
                                      'controlled_car': {'episodes': 0}}},
    'unauthorized_entries': {'no_grant': 0, 'pedestrian_present': 0},
    'token_overlaps': 0, 'pedestrian_response_ms': {'n': 0},
    'pedestrian_response_unresolved': 0}}


def test_path_problems_checks_continuity_permission_and_blockage():
    assert path_problems(['A', 'Cs', 'J', 'E'], 'car', 'E', start='A') == []
    assert path_problems(['A', 'J', 'E'], 'car', 'E')                 # A-J is not an edge
    assert path_problems(['U', 'P3', 'P2', 'P1'], 'bus', 'P1')        # car-only edges
    assert path_problems(['A', 'Cs', 'J', 'E'], 'car', 'E', blocked={'Cs-J'})
    assert path_problems(['A', 'Cs', 'J'], 'car', 'E')                # wrong goal
    assert path_problems(['A', 'Cs', 'A', 'Cs', 'J', 'E'], 'car', 'E')


def test_free_flow_cost_is_recomputed_from_map_lengths():
    assert math.isclose(free_flow_cost(['A', 'Cs', 'J', 'E']), 10.0 / 0.35)
    assert math.isclose(free_flow_cost(['A', 'N', 'Cn', 'B', 'U', 'E'], 4.0), 44.0)


def test_blocked_edges_follow_the_latest_label():
    ev = [event(1.0, 'Cs-J', 'EDGE_NORMAL'), event(10.0, 'Cs-J', 'EDGE_BLOCKED'),
          event(30.0, 'Cs-J', 'EDGE_NORMAL'), event(50.0, 'B-Cn', 'EDGE_BLOCKED')]
    assert blocked_edges_at(ev, 14.0) == {'Cs-J'}
    assert blocked_edges_at(ev, 38.0) == set()
    assert blocked_edges_at(ev, 54.0) == {'B-Cn'}


def test_route_reply_joins_the_service_reply_with_the_planner_record():
    ev = [
        event(14.02, 'car', 'ROUTE_COMPUTED',
              'A->E OK path=A-N-Cn-B-U-E cost=40.000s excluded=- edges_age_ms=120 '
              'crossing_wait=0.0s latency_ms=0.21'),
        event(14.03, 'scenario_runner', 'ROUTE_REPLY',
              'A->E agent=car success=true reason=OK path=A-N-Cn-B-U-E waypoints=6 '
              'cost=40.000000 latency_ms=3.10', 'detour'),
        event(54.03, 'scenario_runner', 'ROUTE_REPLY',
              'A->E agent=car success=false reason=NO_ROUTE path=- waypoints=0 '
              'cost=0.000000 latency_ms=2.00', 'no_route'),
    ]
    r = route_reply(ev, 'detour')
    assert r['success'] and r['path'] == ['A', 'N', 'Cn', 'B', 'U', 'E']
    assert r['cost'] == 40.0 and r['wait'] == 0.0 and r['edges_age_ms'] == 120.0
    r = route_reply(ev, 'no_route')
    assert not r['success'] and r['reason'] == 'NO_ROUTE' and r['path'] == []
    assert route_reply(ev, 'baseline') is None


def test_replan_chain_accepts_a_causal_reroute():
    problems, values = replan_chain(escort_events())
    assert problems == []
    assert values['old_path'] == 'R-W-Cn-Cs-J-D' and values['blocked_edge'] == 'Cs-J'
    assert values['new_path'] == 'Cn-B-U-E-J-D'
    assert values['detect_ms'] == 50 and values['react_ms'] == 50
    ok, detail, _ = check_escort_ok(escort_events(), SAFE)
    assert ok and 'blocked Cs-J' in detail


def test_replan_chain_rejects_runs_without_a_real_reroute():
    # Obstacle placed before the route exists: the midterm variant that passed with zero
    # reroutes. The first route already avoids Cs-J, so there is nothing to replan.
    early = [
        event(2.1, 'robot1', 'ROUTE', 'TO_PICKUP R cost=0.0s gen=1'),
        event(3.0, 'scenario_runner', 'INJECT', 'place_obstacle'),
        event(3.1, 'Cs-J', 'EDGE_BLOCKED'),
        event(6.2, 'robot1', 'ROUTE', 'ESCORT R-W-Cn-B-U-E-J-D cost=42.9s gen=2'),
        event(90.0, 'robot1', 'ARRIVED', 'error_m=0.031'),
        event(90.1, 'robot1', 'GOAL_SUCCEED', 'ARRIVED'),
    ]
    assert replan_chain(early)[0]
    assert not check_escort_ok(early, SAFE)[0]
    # New route still uses the blocked edge, or the generation did not advance.
    assert replan_chain(escort_events(new_route='ESCORT Cs-J-D cost=14.3s gen=3'))[0]
    assert replan_chain(escort_events(new_route='ESCORT Cn-B-U-E-J-D cost=37s gen=2'))[0]
    # Stop not measured within the declared budget.
    assert replan_chain(escort_events(stop_ms=900))[0]
    # REROUTE recorded before the blockage was observed is not causal.
    swapped = escort_events()
    swapped[4] = dict(swapped[4], t=29.0)
    assert replan_chain(swapped)[0]


def test_stop_monitor_measures_standstill_and_motion_without_route():
    m = StopMonitor()
    moving, stopped = [('robot1', 'robot', 3.0, 6.0, 0.25)], [('robot1', 'robot', 3.1, 6.0, 0.0)]
    assert m.sample(49.9, moving) == []
    m.demand('robot1', 'REROUTE', 50.05)
    assert m.sample(50.0, stopped) == []                 # sample older than the demand
    assert m.sample(50.1, moving) == []
    found = m.sample(50.2, stopped)
    assert found == [('ROUTE_STOP_MEASURED', 'robot1', 'cause=REROUTE latency_ms=150')]
    assert m.sample(50.3, moving) == []                  # rerouted: free to move again
    m.demand('robot1', 'NO_ROUTE', 50.4)
    assert m.sample(50.5, stopped)[0][0] == 'ROUTE_STOP_MEASURED'
    assert m.sample(50.6, stopped) == []
    assert m.sample(50.7, moving) == [('MOTION_WITHOUT_ROUTE', 'robot1', 'speed=0.250')]
    s = m.summary()
    assert s['latency_ms']['n'] == 2 and s['motion_without_route'] == 1
    assert s['held_samples'] == {'robot1': 1} and s['unresolved'] == 0
    m.release('robot1')
    assert m.sample(60.0, moving) == []
    m.demand('robot1', 'REROUTE', 61.0)
    assert m.summary()['unresolved'] == 1


def test_blocked_ahead_finds_the_first_blocked_remaining_edge():
    route = ['R', 'W', 'Cn', 'Cs', 'J', 'D']
    states = {e: Edge(e) for e in ('R-W', 'Cn-W', 'Cn-Cs', 'Cs-J', 'D-J')}
    assert blocked_ahead(route, 1, states) == ''
    states['Cs-J'] = Edge('Cs-J', blocked=True)
    assert blocked_ahead(route, 1, states) == 'Cs-J'
    assert blocked_ahead(route, 4, states) == ''         # already past it
    states['Cs-J'] = Edge('Cs-J', blocked=True, valid=False)
    assert blocked_ahead(route, 1, states) == ''         # unknown is handled per edge


def test_trim_turn_back_only_drops_an_immediate_return():
    assert trim_turn_back(['Cs', 'Cn', 'B'], 'Cn') == ['Cn', 'B']
    assert trim_turn_back(['W', 'Cn', 'B'], 'R') == ['W', 'Cn', 'B']
    assert trim_turn_back(['W', 'Cn', 'B'], '') == ['W', 'Cn', 'B']
    assert trim_turn_back(['Cs'], 'Cn') == ['Cs']
