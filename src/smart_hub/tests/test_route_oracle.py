# Author: Vladislav Li (12250245)
"""T11 / T12: Dijkstra vs. an independent exhaustive simple-path oracle (13.3)."""
import math
import random

from smart_hub import hub_map
from smart_hub.route_engine import DEFAULT_SPEED_MPS, edge_cost, plan


def oracle(start, goal, agent, blocked, density, speed):
    """Enumerate every simple path and return the minimum total cost."""
    adj = {}
    for a, b in hub_map.allowed_edges(agent):
        e = hub_map.edge_id(a, b)
        if e in blocked:
            continue
        c = edge_cost(hub_map.edge_length(a, b), speed.get(e, DEFAULT_SPEED_MPS),
                      density.get(e, 0.0))
        adj.setdefault(a, []).append((b, c))
        adj.setdefault(b, []).append((a, c))
    best = math.inf

    def dfs(node, cost, seen):
        nonlocal best
        if node == goal:
            best = min(best, cost)
            return
        for nxt, c in adj.get(node, []):
            if nxt not in seen:
                dfs(nxt, cost + c, seen | {nxt})

    dfs(start, 0.0, {start})
    return best


def test_planner_matches_oracle_on_20_seeded_edge_states():
    rng = random.Random(42)
    edges = [hub_map.edge_id(a, b) for a, b in hub_map.VEHICLE_EDGES + hub_map.PARKING_EDGES]
    for _ in range(20):
        blocked = {e for e in edges if rng.random() < 0.2}
        density = {e: rng.uniform(0.0, 1.0) for e in edges}
        speed = {e: rng.uniform(0.1, 0.4) for e in edges}
        for start, goal, agent in (('A', 'E', 'car'), ('A', 'P1', 'car'), ('N', 'J', 'bus')):
            path, cost = plan(start, goal, agent, blocked, density, speed)
            expected = oracle(start, goal, agent, blocked, density, speed)
            if math.isinf(expected):
                assert path == [] and math.isinf(cost)       # explicit NO_ROUTE
                continue
            assert math.isclose(cost, expected, rel_tol=1e-6)
            allowed = {hub_map.edge_id(a, b) for a, b in hub_map.allowed_edges(agent)}
            for a, b in zip(path, path[1:]):
                e = hub_map.edge_id(a, b)
                assert e in allowed and e not in blocked      # continuity, no blocked edge
            assert path[0] == start and path[-1] == goal
