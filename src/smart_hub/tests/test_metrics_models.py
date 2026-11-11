# Author: Mikhail Li (12236295)
"""Unit tests: IoT device models, energy model, metric helpers."""
import math

from smart_hub import hub_map
from smart_hub.iot_devices import Barrier, Door, Led
from smart_hub.metrics import (
    CrossingMonitor, parse_latency_ms, percentile, QueueLedger, relaxed_crossing_conflict,
    Series,
)
from smart_hub.sim_world import STEP_SEC, World


def test_energy_idle_and_moving():
    idle, moving = World({}, 0), World({}, 0)
    for _ in range(40):
        moving.command('robot1', 0.25, 0.0)
        idle.step()
        moving.step()
    e_idle = {a: e for a, _, e, _ in idle.energy_report()}['robot1']
    e_move = {a: e for a, _, e, _ in moving.energy_report()}['robot1']
    assert math.isclose(e_idle, 5.0 * 40 * STEP_SEC)
    assert e_move > e_idle


def test_barrier_never_closes_on_occupied_passage():
    b = Barrier()
    b.request(True)
    for _ in range(25):
        b.step(0.05)
    assert b.is_open and b.state == 'OPEN'
    b.passage_occupied = True
    assert not b.request(False)


def test_door_reported_open_while_opening():
    d = Door()
    assert not d.request(True, bus_stopped=False)
    assert d.request(True, bus_stopped=True)
    d.step(0.05)
    assert d.is_open


def test_led_colours_and_blink():
    led = Led('P1')
    led.slot_state = 'OCCUPIED'
    led.step(0.05, 0.0)
    assert led.colour == 'RED'
    led.slot_state = 'UNKNOWN'
    colours = set()
    for t in (0.0, 0.6):
        led.step(0.05, t)
        colours.add(led.colour)
    assert colours == {'AMBER', 'OFF'}


def test_metric_helpers():
    assert percentile([1, 2, 3, 4], 50) == 2
    assert percentile(range(1, 101), 95) == 95
    s = Series()
    for v in (10, 20, 30):
        s.add(v)
    assert s.summary()['mean'] == 20
    assert parse_latency_ms('ESTOP latency_ms=0.42') == 0.42
    assert parse_latency_ms('no number') is None


CX, CY = 5.0, 3.0


def robot(x=CX, y=CY, v=0.2):
    return ('robot1', 'robot', x, y, v)


def test_strict_occupancy_counts_every_pair_without_grace():
    assert hub_map.point_in_rect((CX, CY), hub_map.CROSSING_C)
    m = CrossingMonitor()
    m.sample(0.0, [robot()], [], 'robot1')
    assert m.summary()['strict_occupancy']['episodes'] == 0      # the owner alone
    # A stationary robot next to a pedestrian and two cars together are both occupancy;
    # the midterm definition counted neither.
    m.sample(0.1, [robot(v=0.0)], [(CX, 2.5)], 'robot1')
    m.sample(0.2, [robot(v=0.0)], [(CX, 2.5)], 'robot1')
    m.sample(0.3, [robot(v=0.0)], [], 'robot1')
    cars = [('c1', 'car', CX, 2.0, 0.3), ('c2', 'car', CX, 4.0, 0.3)]
    m.sample(0.4, cars, [], '')
    m.sample(0.5, [], [], '')
    occupancy = m.summary()['strict_occupancy']
    assert occupancy['episodes'] == 2
    assert occupancy['by_class']['controlled_pedestrian'] == {'episodes': 1, 'seconds': 0.2}
    assert occupancy['by_class']['car_car'] == {'episodes': 1, 'seconds': 0.1}
    assert occupancy['by_class']['controlled_controlled']['episodes'] == 0
    assert not relaxed_crossing_conflict([robot(v=0.0)], [(CX, 2.5)])
    assert not relaxed_crossing_conflict(cars, [])


def test_occupancy_trace_keeps_every_change():
    m = CrossingMonitor()
    rows = [m.sample(0.1 * i, agents, peds, owner, 'tok')[0] for i, (agents, peds, owner) in
            enumerate([([], [], ''), ([robot()], [], 'robot1'), ([robot()], [], 'robot1'),
                       ([robot()], [(CX, 2.5)], 'robot1'), ([robot()], [], None)])]
    assert [r is not None for r in rows] == [True, True, False, True, True]
    assert rows[3][1:] == ['robot1', 1, 'robot1', 'tok', 'controlled_pedestrian']
    assert rows[4][3] == 'UNKNOWN'


def test_unauthorized_entries_by_kind():
    outside = robot(y=6.0)
    m = CrossingMonitor()
    m.sample(0.0, [outside], [], '')
    _, found = m.sample(0.1, [robot()], [], '')                  # nobody owns C
    assert found[0][:2] == ('UNAUTHORIZED_ENTRY', 'robot1') and 'no_grant' in found[0][2]
    m.sample(0.2, [outside], [], 'robot1')
    m.sample(0.3, [robot()], [], 'robot1')                       # owner enters: fine
    m.sample(0.4, [outside], [(CX, 2.5)], 'robot1')
    m.sample(0.5, [robot()], [(CX, 2.5)], 'robot1')              # pedestrian already inside
    m.sample(0.6, [outside], [], None)
    m.sample(0.7, [robot()], [], None)                           # stale / invalid state
    car_out, car_in = ('c1', 'car', 3.0, 2.0, 0.3), ('c1', 'car', CX, 2.0, 0.3)
    m.sample(0.8, [car_out], [], 'bus1')
    m.sample(0.9, [car_in], [], 'bus1')
    m.sample(1.0, [car_out], [], '')
    m.sample(1.1, [car_in], [], '')                              # free crossing: allowed
    assert m.summary()['unauthorized_entries'] == {
        'no_grant': 2, 'pedestrian_present': 1, 'car_against_owner': 1}
    spawned = CrossingMonitor()
    spawned.sample(0.0, [robot()], [], '')                       # first seen inside C
    assert sum(spawned.summary()['unauthorized_entries'].values()) == 0


def test_pedestrian_response_latency_and_unresolved():
    m = CrossingMonitor()
    m.sample(0.0, [robot()], [], 'robot1')
    m.sample(0.1, [robot()], [(CX, 2.5)], 'robot1')              # pedestrian steps in
    m.sample(0.2, [robot(v=0.1)], [(CX, 2.5)], 'robot1')
    _, found = m.sample(0.4, [robot(v=0.0)], [(CX, 2.5)], 'robot1')
    assert ('PEDESTRIAN_RESPONSE', 'robot1', 'latency_ms=300') in found
    assert math.isclose(m.summary()['pedestrian_response_ms']['max'], 300.0)
    m.sample(0.5, [robot(v=0.0)], [], 'robot1')
    m.sample(0.6, [robot()], [(CX, 2.5)], 'robot1')
    m.sample(0.7, [robot()], [], 'robot1')                       # never stopped
    assert m.summary()['pedestrian_response_unresolved'] == 1


def test_token_overlap_follows_grant_and_release_events():
    m = CrossingMonitor()
    m.on_event('CROSSING_GRANTED', 'a')
    m.on_event('CROSSING_RELEASED', 'a')
    m.on_event('CROSSING_GRANTED', 'b')
    m.on_event('CROSSING_FREE', '')                              # lease expired, C cleared
    m.on_event('CROSSING_GRANTED', 'c')
    assert m.summary()['token_overlaps'] == 0
    m.on_event('CROSSING_GRANTED', 'd')                          # 'c' was never released
    assert m.summary()['token_overlaps'] == 1


def test_queue_ledger_reports_censored_and_abandoned_waits():
    q = QueueLedger()
    for i, t in enumerate((0.0, 10.0, 20.0, 30.0)):
        q.enter('ped%d' % i, t)
    q.board('ped0', 40.0, 40.0)
    q.abandon('ped1', 25.0)
    q.balked += 1
    s = q.summary(100.0)
    assert (s['arrived'], s['boarded'], s['abandoned'], s['waiting_at_end']) == (4, 1, 1, 2)
    assert s['completion_rate'] == 0.25 and s['clearance_rate'] == 0.5 and s['balked'] == 1
    assert s['wait_boarded_sec']['mean'] == 40.0
    assert s['wait_censored_sec'] == {'n': 2, 'mean': 75.0, 'p50': 70.0, 'p95': 80.0,
                                      'max': 80.0}
    # Boarded-only waits hide the two passengers who never got a bus.
    assert s['wait_all_lower_bound_sec']['n'] == 4
    assert s['wait_all_lower_bound_sec']['mean'] == (40.0 + 15.0 + 80.0 + 70.0) / 4


def test_world_logs_every_passenger_entry():
    w = World({'events': [{'t': 0.0, 'type': 'spawn_queue_tracks', 'count': 3}]}, 0)
    w.step()
    assert [e.event for e in w.log].count('QUEUE_ENTERED') == 3
