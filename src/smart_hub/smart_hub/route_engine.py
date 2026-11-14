# Author: Vladislav Li (12250245)
"""
Congestion-aware Dijkstra over map M1 (assignment 13.2, 18.3).

Based on the handbook's companion route engine (appendix A).

Cost terms (13.2), each built here so that the planner and its clients price a path the
same way:
  L_e    Euclidean edge length from the map;
  v_e    nominal speed DEFAULT_SPEED_MPS on an edge without observed traffic, otherwise the
         windowed speed from traffic_monitor bounded to [MIN_SPEED_MPS, DEFAULT_SPEED_MPS];
  rho_e  density ratio in [0, 1];
  w_e    expected crossing wait: half of the expected wait on each edge that has exactly
         one end inside crossing C, so one passage through C is charged the wait once.
Blocked, invalid, unknown and forbidden edges are removed, never penalised. route_planner
feeds live edge states and fails on a stale snapshot, robot_manager applies the replan
rule (15 % improvement, 2 s period), and tests/test_route_oracle.py checks Dijkstra
against an independent exhaustive path enumeration (T11).
"""
import heapq
import math

from smart_hub.hub_map import (
    allowed_edges, CROSSING_C, edge_id, edge_length, NODES, point_in_rect,
)


DEFAULT_SPEED_MPS = 0.35
MIN_SPEED_MPS = 0.05


def edge_cost(length, speed=DEFAULT_SPEED_MPS, density=0.0, wait=0.0):
    """c_e = L_e / max(v_e, 0.05) * (1 + 2 * rho_e) + w_e, in seconds."""
    return length / max(speed, MIN_SPEED_MPS) * (1.0 + 2.0 * density) + wait


def bounded_speed(observed):
    """Observed edge speed bounded to [MIN_SPEED_MPS, DEFAULT_SPEED_MPS]."""
    return min(max(observed, MIN_SPEED_MPS), DEFAULT_SPEED_MPS)


def crossing_wait_edges(agent_type, wait_sec):
    """Return {edge_id: w_e}: half of `wait_sec` on every edge entering or leaving C."""
    out = {}
    for a, b in allowed_edges(agent_type):
        if point_in_rect(NODES[a], CROSSING_C) != point_in_rect(NODES[b], CROSSING_C):
            out[edge_id(a, b)] = 0.5 * wait_sec
    return out


def cost_inputs(states, agent_type='car', crossing_wait_sec=0.0, excluded=()):
    """
    Turn an edge-state snapshot into plan() inputs: (blocked, density, speed, wait).

    `states` maps edge ids to objects with blocked, valid, density_ratio and
    mean_speed_mps. An allowed edge that is blocked, invalid or missing from the snapshot
    is removed (unknown is never "free").
    """
    blocked = set(excluded)
    density, speed = {}, {}
    for a, b in allowed_edges(agent_type):
        e = edge_id(a, b)
        s = states.get(e)
        if s is None or s.blocked or not s.valid:
            blocked.add(e)
            continue
        density[e] = s.density_ratio
        if s.density_ratio > 0.0:
            speed[e] = bounded_speed(s.mean_speed_mps)
    wait = crossing_wait_edges(agent_type, crossing_wait_sec) if crossing_wait_sec > 0.0 \
        else {}
    return blocked, density, speed, wait


def usable(rho, v, w):
    """Reject values that would turn into a zero, negative or undefined travel time."""
    return 0.0 <= rho <= 1.0 and math.isfinite(v) and v > 0.0 and \
        math.isfinite(w) and w >= 0.0


def path_cost(nodes, blocked=(), density=None, speed=None, wait=None):
    """Cost of a given node path with the planner's formula; inf if an edge is unusable."""
    density, speed, wait = density or {}, speed or {}, wait or {}
    cost = 0.0
    for a, b in zip(nodes, nodes[1:]):
        e = edge_id(a, b)
        rho, v, w = density.get(e, 0.0), speed.get(e, DEFAULT_SPEED_MPS), wait.get(e, 0.0)
        if e in blocked or not usable(rho, v, w):
            return math.inf
        cost += edge_cost(edge_length(a, b), v, rho, w)
    return cost


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
        if not usable(rho, v, w):
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
