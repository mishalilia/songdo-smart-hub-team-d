# Author: Vladislav Li (12250245)
"""
Congestion-aware Dijkstra over map M1 (assignment 13.2, 18.3).

Based on the handbook's companion route engine (appendix A).
Related pieces live elsewhere: route_planner removes blocked / invalid edges and fails on
stale edge states, robot_manager applies the replan rule (15 % improvement, 2 s period),
and tests/test_route_oracle.py checks Dijkstra against exhaustive path enumeration (T11).
"""
import heapq
import math

from smart_hub.hub_map import allowed_edges, edge_id, edge_length, NODES


DEFAULT_SPEED_MPS = 0.35
MIN_SPEED_MPS = 0.05


def edge_cost(length, speed=DEFAULT_SPEED_MPS, density=0.0, wait=0.0):
    """c_e = L_e / max(v_e, 0.05) * (1 + 2 * rho_e) + w_e, in seconds."""
    return length / max(speed, MIN_SPEED_MPS) * (1.0 + 2.0 * density) + wait


def plan(start, goal, agent_type='car', blocked=(), density=None, speed=None, wait=None):
    """
    Return (node_ids, cost_sec). Empty list and inf cost means NO_ROUTE.

    Blocked, forbidden and invalid edges are removed, not penalised.
    """
    if start not in NODES or goal not in NODES:
        raise ValueError('Unknown node')
    density, speed, wait = density or {}, speed or {}, wait or {}
    blocked = set(blocked)

    graph = {node: [] for node in NODES}
    for a, b in allowed_edges(agent_type):
        e = edge_id(a, b)
        if e in blocked:
            continue
        rho = density.get(e, 0.0)
        v = speed.get(e, DEFAULT_SPEED_MPS)
        w = wait.get(e, 0.0)
        if not (0.0 <= rho <= 1.0 and math.isfinite(v) and v > 0.0 and w >= 0.0):
            continue  # invalid speed must never become zero travel time
        c = edge_cost(edge_length(a, b), v, rho, w)
        graph[a].append((b, c))
        graph[b].append((a, c))

    queue = [(0.0, start, (start,))]
    best = {start: 0.0}
    while queue:
        cost, u, path = heapq.heappop(queue)
        if cost > best[u] + 1e-10:
            continue
        if u == goal:
            return list(path), cost
        for v, w in graph[u]:
            alt = cost + w
            if alt < best.get(v, math.inf):
                best[v] = alt
                heapq.heappush(queue, (alt, v, path + (v,)))
    return [], math.inf
