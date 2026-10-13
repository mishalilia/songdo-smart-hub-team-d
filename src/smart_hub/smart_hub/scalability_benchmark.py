"""
Scalability benchmark — owner M8.

Measures how per-cycle compute of the simulator grows with load: world step (20 Hz budget
50 ms), observation generation for the published topics (vehicles with edge assignment,
noisy pedestrians, slots) and route planning (on request, target p95 < 250 ms).
Load = N cars on the vehicle graph and 4N pedestrians. Writes a CSV and prints a table.

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

LOADS = (0, 5, 10, 25, 50, 100)


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
    return {
        'cars_requested': cars,
        'cars_placed': sum(1 for a in world.agents.values() if a.kind == 'car'),
        'pedestrians': len(world.peds),
        'sim_step_mean_ms': mean, 'sim_step_p95_ms': percentile(sim_ms, 95),
        'realtime_factor': STEP_SEC * 1000.0 / max(1e-9, mean),
        'observations_p95_ms': percentile(obs_ms, 95),
        'route_p95_ms': percentile(route_ms, 95),
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
    print('  '.join('%16s' % c for c in cols))
    for r in rows:
        print('  '.join('%16.3f' % r[c] if isinstance(r[c], float) else '%16d' % r[c]
                        for c in cols))
    print('budget: sim step < %.0f ms (20 Hz), route p95 < 250 ms; nodes in map: %d' % (
        STEP_SEC * 1000.0, len(hub_map.NODES)))
    if opts.out:
        print('written', opts.out)


if __name__ == '__main__':
    main()
