# Author: Mikhail Li (12236295)
"""
Scalability benchmark of the simulator core (single process, no ROS graph).

Measures how per-cycle compute grows with load: world step (20 Hz budget 50 ms),
observation generation for the published topics (vehicles with edge assignment, noisy
pedestrians, slots) and route planning (on request, target p95 < 250 ms).
Requested load = N cars on the vehicle ring and 4N pedestrians. The map is small, so the
simulator places fewer agents than requested once the ring and the free area are full; every
row reports requested and placed counts, and the load of a row is its placed count.

world_step_compute_ratio = step period / mean world-step compute time. It is the headroom
of the world step alone; it is not the speed of the whole ROS graph and says nothing about
executor load, sensor backlog or message latency, which this benchmark does not measure.

  ros2 run smart_hub scalability_benchmark --steps 400 --out evidence/scalability.csv
"""
import argparse
import csv
import os
import time

from smart_hub import hub_map
from smart_hub.metrics import percentile
from smart_hub.route_engine import plan
from smart_hub.sim_world import STEP_SEC, World

# The last two requests exceed what the map can hold and show the placement limit.
LOADS = (0, 5, 10, 15, 20, 25, 50, 100)


def timed(fn):
    start = time.perf_counter()
    fn()
    return (time.perf_counter() - start) * 1000.0


def run_load(cars, steps, seed):
    world = World({'events': [
        {'t': 0.0, 'type': 'spawn_random_cars', 'count': cars},
        {'t': 0.0, 'type': 'spawn_crowd', 'count': 4 * cars},
    ]}, seed)
    world.step()                                   # spawn the load before timing
    sim_ms, obs_ms, route_ms = [], [], []

    def observe():
        world.observed_vehicles(0.02)
        world.observed_pedestrians(0.05)
        world.observed_slots()

    for i in range(steps):
        sim_ms.append(timed(world.step))
        if i % 2 == 0:                             # 10 Hz topics
            obs_ms.append(timed(observe))
        if i % 10 == 0:
            blocked = {world.edge_of(a) for a in world.agents.values() if a.kind == 'obstacle'}
            route_ms.append(timed(lambda: plan('A', 'E', 'car', blocked)))
    mean = sum(sim_ms) / len(sim_ms)
    cars_placed = sum(1 for a in world.agents.values() if a.kind == 'car')
    return {
        'cars_requested': cars, 'cars_placed': cars_placed,
        'pedestrians_requested': 4 * cars, 'pedestrians_placed': len(world.peds),
        'placement_complete': int(cars_placed == cars and len(world.peds) == 4 * cars),
        'steps': steps,
        'world_step_mean_ms': mean, 'world_step_p95_ms': percentile(sim_ms, 95),
        'world_step_p99_ms': percentile(sim_ms, 99), 'world_step_max_ms': max(sim_ms),
        'world_step_compute_ratio': STEP_SEC * 1000.0 / max(1e-9, mean),
        'observations_p95_ms': percentile(obs_ms, 95),
        'observations_p99_ms': percentile(obs_ms, 99),
        'route_p95_ms': percentile(route_ms, 95), 'route_p99_ms': percentile(route_ms, 99),
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--steps', type=int, default=400)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--loads', type=int, nargs='*', default=list(LOADS))
    parser.add_argument('--out', default=os.path.join('evidence', 'scalability.csv'))
    opts, _ = parser.parse_known_args(args)

    rows = [run_load(n, opts.steps, opts.seed) for n in opts.loads]
    if opts.out:
        os.makedirs(os.path.dirname(opts.out) or '.', exist_ok=True)
        with open(opts.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    cols = list(rows[0])
    print('  '.join('%s' % c for c in cols))
    for r in rows:
        print('  '.join('%.3f' % r[c] if isinstance(r[c], float) else '%d' % r[c]
                        for c in cols))
    print('budget: world step < %.0f ms (20 Hz), route p95 < 250 ms; nodes in map: %d' % (
        STEP_SEC * 1000.0, len(hub_map.NODES)))
    full = [r for r in rows if r['placement_complete']]
    for r in rows:
        if not r['placement_complete']:
            print('placement incomplete: %d of %d cars, %d of %d pedestrians placed' % (
                r['cars_placed'], r['cars_requested'], r['pedestrians_placed'],
                r['pedestrians_requested']))
    print('largest fully placed load: %d cars, %d pedestrians' % (
        max((r['cars_placed'] for r in full), default=0),
        max((r['pedestrians_placed'] for r in full), default=0)))
    if opts.out:
        print('written', opts.out)


if __name__ == '__main__':
    main()
