# Author: Vladislav Li (12250245)
"""
T11 / T12: Dijkstra vs. an independent exhaustive simple-path oracle (13.3).

The oracle shares no code with the planner: it reads config/map.yaml itself, builds its
own adjacency and restates the cost formula of assignment 13.2, so an error in
route_engine.edge_cost or hub_map cannot cancel out in the comparison.
"""
import math
from pathlib import Path
import random

from smart_hub.route_engine import plan
import yaml

MAP = yaml.safe_load((Path(__file__).resolve().parent.parent / 'config' / 'map.yaml')
                     .read_text())
NOMINAL_MPS = 0.35
FLOOR_MPS = 0.05


def oracle_edges(agent):
    edges = {'car': MAP['vehicle_edges'] + MAP['parking_edges'],
             'bus': MAP['vehicle_edges'], 'robot': MAP['robot_edges']}[agent]
    return [(a, b, '-'.join(sorted((a, b)))) for a, b in edges]


def oracle_cost(a, b, speed, density, wait):
    (ax, ay), (bx, by) = MAP['nodes'][a], MAP['nodes'][b]
    length = ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
    v = speed if speed > FLOOR_MPS else FLOOR_MPS
    return length / v * (1.0 + 2.0 * density) + wait


def oracle(start, goal, agent, blocked, density, speed, wait):
    """Enumerate every simple path; return (minimum cost, {path: cost})."""
    adj = {}
    for a, b, e in oracle_edges(agent):
        if e in blocked:
            continue
        c = oracle_cost(a, b, speed.get(e, NOMINAL_MPS), density.get(e, 0.0),
                        wait.get(e, 0.0))
        adj.setdefault(a, []).append((b, c))
        adj.setdefault(b, []).append((a, c))
    costs = {}

    def dfs(node, cost, path):
        if node == goal:
            costs[path] = cost
            return
        for nxt, c in adj.get(node, []):
            if nxt not in path:
                dfs(nxt, cost + c, path + (nxt,))

    dfs(start, 0.0, (start,))
    return (min(costs.values()) if costs else math.inf), costs


def test_planner_matches_oracle_on_20_seeded_edge_states():
    rng = random.Random(42)
    compared = no_route = 0
    for _ in range(20):
        for start, goal, agent in (('A', 'E', 'car'), ('A', 'P1', 'car'), ('N', 'J', 'bus'),
                                   ('R', 'D', 'robot')):
            edges = [e for _, _, e in oracle_edges(agent)]
            blocked = {e for e in edges if rng.random() < 0.2}
            density = {e: rng.uniform(0.0, 1.0) for e in edges}
            speed = {e: rng.uniform(0.01, 0.4) for e in edges}      # below the floor too
            wait = {e: rng.uniform(0.0, 5.0) for e in edges if rng.random() < 0.3}
            path, cost = plan(start, goal, agent, blocked, density, speed, wait)
            expected, costs = oracle(start, goal, agent, blocked, density, speed, wait)
            if math.isinf(expected):
                assert path == [] and math.isinf(cost)       # explicit NO_ROUTE
                no_route += 1
                continue
            compared += 1
            assert math.isclose(cost, expected, rel_tol=1e-6)
            # The returned path is one the oracle enumerated (continuous, permitted, no
            # blocked edge, simple) and its own cost is the reported minimum.
            assert math.isclose(costs[tuple(path)], expected, rel_tol=1e-6)
            assert path[0] == start and path[-1] == goal
    assert compared >= 20 and no_route >= 1


def test_oracle_reference_values_from_the_assignment():
    # 13.3: lower path A-Cs-J-E is 10 m, upper detour A-N-Cn-B-U-E is 14 m.
    best, costs = oracle('A', 'E', 'car', set(), {}, {}, {})
    assert math.isclose(costs[('A', 'Cs', 'J', 'E')], 10.0 / NOMINAL_MPS)
    assert math.isclose(costs[('A', 'N', 'Cn', 'B', 'U', 'E')], 14.0 / NOMINAL_MPS)
    assert math.isclose(best, 10.0 / NOMINAL_MPS)
    assert math.isinf(oracle('A', 'E', 'car', {'Cs-J', 'B-Cn'}, {}, {}, {})[0])
