# Author: Mikhail Li (12236295)
"""
Map M1 loaded from config/map.yaml (assignment sections 2-3).

Origin is the south-west corner, +x east, +y north, metres / radians.
"""
import math
from pathlib import Path

import yaml


def _map_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('smart_hub')) / 'config' / 'map.yaml'
    except Exception:
        return Path(__file__).resolve().parent.parent / 'config' / 'map.yaml'


def _rect(r):
    return (r['xmin'], r['xmax'], r['ymin'], r['ymax'])


with open(_map_path()) as _f:
    _M = yaml.safe_load(_f)

NODES = {k: tuple(v) for k, v in _M['nodes'].items()}
VEHICLE_EDGES = [tuple(e) for e in _M['vehicle_edges']]
PARKING_EDGES = [tuple(e) for e in _M['parking_edges']]
ROBOT_EDGES = [tuple(e) for e in _M['robot_edges']]

# Axis-aligned regions: (xmin, xmax, ymin, ymax)
CROSSING_C = _rect(_M['regions']['crossing_C'])
QUEUE_Q = _rect(_M['regions']['queue_Q'])
MAP_BOUNDS = _rect(_M['bounds'])

OBSTACLE_O = {'center': tuple(_M['obstacle_O']['center']), 'radius': _M['obstacle_O']['radius']}

BUS_BAY = _M['bus_bay']
PARKING_SLOTS = tuple(_M['parking_slots'])


def edge_id(a, b):
    """Return the undirected edge id: ('J', 'Cs') -> 'Cs-J'."""
    return '-'.join(sorted((a, b)))


def edge_length(a, b):
    return math.dist(NODES[a], NODES[b])


def allowed_edges(agent_type):
    if agent_type == 'robot':
        return list(ROBOT_EDGES)
    if agent_type == 'bus':
        return list(VEHICLE_EDGES)
    if agent_type == 'car':
        return VEHICLE_EDGES + PARKING_EDGES
    raise ValueError(f'Unsupported agent type: {agent_type!r}')


def point_in_rect(p, rect):
    x, y = p
    xmin, xmax, ymin, ymax = rect
    return xmin <= x <= xmax and ymin <= y <= ymax


def segment_intersects_rect(p, q, rect, inflate=0.0):
    """Liang-Barsky clip: True if segment p-q touches rect (optionally inflated)."""
    xmin, xmax, ymin, ymax = rect
    xmin, xmax, ymin, ymax = xmin - inflate, xmax + inflate, ymin - inflate, ymax + inflate
    (x0, y0), (x1, y1) = p, q
    dx, dy = x1 - x0, y1 - y0
    t0, t1 = 0.0, 1.0
    for pk, qk in ((-dx, x0 - xmin), (dx, xmax - x0), (-dy, y0 - ymin), (dy, ymax - y0)):
        if pk == 0:
            if qk < 0:
                return False
            continue
        t = qk / pk
        if pk < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def crossing_edges(inflate=0.0):
    """
    Edges that require crossing ownership before entry.

    Includes W-Cn in addition to the assignment's example list, because Cn lies inside C.
    """
    all_edges = set(VEHICLE_EDGES) | set(ROBOT_EDGES)
    return sorted(
        edge_id(a, b) for a, b in all_edges
        if segment_intersects_rect(NODES[a], NODES[b], CROSSING_C, inflate)
    )
