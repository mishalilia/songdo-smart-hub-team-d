"""Simulator unit tests (assignment 3.2, scenario S1 timeline). Owner: M8."""
import math
from pathlib import Path

from smart_hub import hub_map
from smart_hub.sim_world import BOARDING_CAPACITY, STEP_SEC, World
import yaml

SCENARIOS = Path(__file__).resolve().parents[1] / 'scenarios'


def load(name):
    with open(SCENARIOS / (name + '.yaml')) as f:
        return yaml.safe_load(f)


def run_until(world, t):
    while world.t < t - 1e-9:
        world.step()


def queue_count(world):
    return sum(1 for p in world.peds.values() if p.zone == 'Q')


def test_all_scenarios_load_and_run():
    for path in sorted(SCENARIOS.glob('*.yaml')):
        world = World(load(path.stem), seed=1)
        run_until(world, min(world.duration, 130.0))
        assert not [e for e in world.log if e.event == 'UNKNOWN_EVENT']


def test_same_seed_is_deterministic():
    a, b = World(load('S1'), seed=7), World(load('S1'), seed=7)
    run_until(a, 12.0)
    run_until(b, 12.0)
    assert a.observed_pedestrians(0.05) == b.observed_pedestrians(0.05)


def test_s1_truth_timeline():
    w = World(load('S1'), seed=42)
    for t, expected in ((4.9, 0), (9.9, 3), (14.9, 5), (19.9, 4)):
        run_until(w, t)
        assert queue_count(w) == expected, t
    run_until(w, 21.0)
    assert any(p.zone == 'C' for p in w.peds.values())
    run_until(w, 24.5)
    assert not any(p.zone == 'C' for p in w.peds.values())
    run_until(w, 30.5)
    assert w.observed_pedestrians(0.05) is None


def test_s1_queue_observations_stay_inside_q():
    w = World(load('S1'), seed=3)
    run_until(w, 12.0)
    for x, y in w.observed_pedestrians(0.05):
        assert hub_map.point_in_rect((x, y), hub_map.QUEUE_Q)


def test_occlusion_hides_one_track_briefly():
    w = World(load('S1'), seed=42)
    run_until(w, 17.1)
    assert len(w.observed_pedestrians(0.0)) == queue_count(w) - 1
    run_until(w, 17.5)
    assert len(w.observed_pedestrians(0.0)) == queue_count(w)


def test_command_timeout_stops_agent():
    w = World({}, seed=0)
    robot = w.agents['robot1']
    x0 = robot.x
    w.command('robot1', 0.2, 0.0)
    for _ in range(5):
        w.step()
    assert math.isclose(robot.x, x0 + 5 * 0.2 * STEP_SEC, abs_tol=1e-6)
    run_until(w, 2.0)
    assert robot.v == 0.0
    assert robot.x < x0 + 0.6


def test_swept_collision_stops_before_obstacle():
    w = World({'events': [{'t': 0.0, 'type': 'place_obstacle', 'id': 'O',
                           'center': [4.0, 6.0], 'radius': 0.35}]}, seed=0)
    robot = w.agents['robot1']
    while w.t < 10.0:
        w.command('robot1', 0.25, 0.0)
        w.step()
    assert math.hypot(robot.x - 4.0, robot.y - 6.0) >= robot.radius + 0.35 - 1e-9
    assert [e for e in w.log if e.event == 'COLLISION']


def _overlaps_obstacle(a):
    return math.hypot(a.x - 7.0, a.y - 2.0) < a.radius + 0.35 - 1e-9


def test_obstacle_reported_on_blocked_edge_and_cars_stop():
    w = World(load('S5'), seed=0)
    run_until(w, 60.1)
    assert w.edge_of(w.agents['O']) == 'Cs-J'
    cars = [a for a in w.agents.values() if a.kind == 'car']
    # A car the obstacle was dropped onto is stuck in place; no other car drives into it.
    trapped = {a.agent_id: (a.x, a.y) for a in cars if _overlaps_obstacle(a)}
    run_until(w, 75.0)
    for a in cars:
        if a.agent_id in trapped:
            assert (a.x, a.y) == trapped[a.agent_id]
        else:
            assert not _overlaps_obstacle(a)


def test_block_edges_and_restore_all():
    w = World({'events': [
        {'t': 0.0, 'type': 'block_edges', 'edges': ['Cs-J', 'B-Cn']},
        {'t': 1.0, 'type': 'restore_all'},
    ]}, seed=0)
    w.step()
    blocked = {w.edge_of(a) for a in w.agents.values() if a.kind == 'obstacle'}
    assert blocked == {'Cs-J', 'B-Cn'}
    run_until(w, 1.1)
    assert not [a for a in w.agents.values() if a.kind == 'obstacle']


def test_parking_observations_and_disconnect():
    w = World(load('S4'), seed=0)
    assert w.observed_slots() == {'P1': True, 'P2': False, 'P3': False}
    run_until(w, 10.1)
    assert 'P3' not in w.observed_slots()


def test_driver_events_are_queued_for_the_node():
    w = World(load('S4'), seed=0)
    run_until(w, 5.1)
    assert [r['type'] for r in w.requests] == ['reserve', 'reserve']
    w.requests.clear()
    w.reservations['C1'] = 'P2'
    run_until(w, 8.1)
    assert w.slots['P2'] is True


def test_boarding_takes_at_most_four_passengers():
    w = World({'events': [{'t': 0.0, 'type': 'spawn_queue_tracks', 'count': 6}]}, seed=0)
    bus = w.agents['bus1']
    bus.x, bus.y = hub_map.NODES['B']
    w.step()
    assert w.device_request('door', True)
    run_until(w, 12.0)
    boarded = [e for e in w.log if e.event == 'BOARDED']
    assert len(boarded) == BOARDING_CAPACITY
    assert queue_count(w) == 2


def test_scripted_route_reaches_goal_and_returns_control():
    w = World({'events': [{'t': 0.0, 'type': 'agent_route', 'agent': 'robot1',
                           'route': ['R', 'W', 'Cn']}]}, seed=0)
    run_until(w, 25.0)
    robot = w.agents['robot1']
    assert math.dist((robot.x, robot.y), hub_map.NODES['Cn']) < 1e-6
    assert robot.controlled and robot.v == 0.0
    assert [e for e in w.log if e.event == 'ROUTE_DONE']
    assert robot.energy_j > 5.0 * 25.0          # driving costs more than idling


def test_scripted_robot_waits_for_pedestrian_ahead():
    w = World({'events': [
        {'t': 0.0, 'type': 'agent_route', 'agent': 'robot1', 'route': ['R', 'W']},
        {'t': 0.0, 'type': 'crossing_pedestrian', 'position': [4.0, 6.0], 'duration_sec': 6.0},
    ]}, seed=0)
    run_until(w, 5.0)
    robot = w.agents['robot1']
    assert robot.x < 4.0 - robot.radius - 0.2 + 1e-6
    run_until(w, 15.0)
    assert math.dist((robot.x, robot.y), hub_map.NODES['W']) < 1e-6
    assert not [e for e in w.log if e.event == 'COLLISION']


def test_power_cut_stops_scripted_motion_without_restart():
    w = World({'events': [{'t': 0.0, 'type': 'agent_route', 'agent': 'robot1',
                           'route': ['R', 'W']}]}, seed=0)
    run_until(w, 2.0)
    w.power_cut()
    x = w.agents['robot1'].x
    run_until(w, 6.0)
    assert w.agents['robot1'].x == x


def test_inject_applies_event_now():
    w = World({}, seed=0)
    run_until(w, 3.0)
    w.inject({'type': 'spawn_queue_tracks', 'count': 2})
    assert queue_count(w) == 2
    w.inject({'type': 'reserve', 'client': 'C1', 'request_id': 'x'})
    assert w.requests and w.requests[0]['t'] == w.t
    w.inject({'type': 'nonsense'})
    assert w.log[-1].event == 'UNKNOWN_EVENT'


def test_simulated_devices_and_interlocks():
    w = World({}, seed=0)
    assert w.device_request('barrier', True)
    run_until(w, 1.5)
    assert w.barrier.is_open
    bus = w.agents['bus1']
    bus.v = 0.3
    assert not w.device_request('door', True)      # never open while moving
    bus.v = 0.0
    assert w.device_request('door', True)
    w.set_slot_states({'P1': 'OCCUPIED'})
    run_until(w, 2.0)
    assert w.leds['P1'].colour == 'RED'
    assert {d.name for d in w.devices()} >= {'barrier', 'bus_door', 'camera', 'led/P1'}
