# Author: Mikhail Li (12236295)
"""Unit tests: IoT device models, energy model, metric helpers."""
import math

from smart_hub import hub_map
from smart_hub.iot_devices import Barrier, Door, Led
from smart_hub.metrics import crossing_conflict, parse_latency_ms, percentile, Series
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


def test_crossing_conflict_definition():
    cx, cy = 5.0, 3.0
    assert hub_map.point_in_rect((cx, cy), hub_map.CROSSING_C)
    assert not crossing_conflict([('robot1', 'robot', cx, cy, 0.2)], [])
    assert crossing_conflict([('robot1', 'robot', cx, cy, 0.2), ('bus1', 'bus', cx, 2.0, 0.0)],
                             [])
    assert crossing_conflict([('robot1', 'robot', cx, cy, 0.2)], [(cx, 2.5)])
    assert not crossing_conflict([('robot1', 'robot', cx, cy, 0.0)], [(cx, 2.5)])
    assert not crossing_conflict([('c1', 'car', cx, cy, 0.3), ('c2', 'car', cx, 2.0, 0.3)], [])
