# Author: Vladislav Li (12250245)
"""
Per-edge traffic estimation (assignment 12.2).

Sliding window of validated vehicle samples -> per-edge robust mean speed, vehicle count,
density rho = min(1, n_e / K_e) with K_e = max(1, floor(L_e / headway)), geometric blockage
from obstacles, and validity (stale or missing observations => invalid, never "empty").
"""
from dataclasses import dataclass
import math
from statistics import median

from smart_hub import hub_map

CORRIDOR_M = 0.5
VEHICLE_KINDS = ('car', 'bus')


@dataclass
class EdgeEstimate:
    edge_id: str
    mean_speed_mps: float
    density_ratio: float
    blocked: bool
    valid: bool
    vehicle_count: int


def all_edges():
    edges = {}
    for a, b in hub_map.VEHICLE_EDGES + hub_map.PARKING_EDGES + hub_map.ROBOT_EDGES:
        edges[hub_map.edge_id(a, b)] = (a, b)
    return edges


def point_segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class TrafficEstimator:

    def __init__(self, window_sec=5.0, headway_m=1.0, stale_sec=1.0, swept_radius_m=0.30):
        self.window_sec = window_sec
        self.headway_m = headway_m
        self.stale_sec = stale_sec
        self.swept_radius_m = swept_radius_m
        self.edges = all_edges()
        self.samples = []          # (t, vehicle_id, edge_id, speed)
        self.latest = {}           # vehicle_id -> (obs_t, edge_id)
        self.obstacles = {}        # id -> (x, y, r, obs_t)
        self.last_feed = -math.inf
        self.rejected = 0

    def capacity(self, edge_id):
        a, b = self.edges[edge_id]
        return max(1, math.floor(hub_map.edge_length(a, b) / self.headway_m))

    def add(self, vehicle_id, kind, x, y, speed, edge_id, valid, obs_t, now):
        """Add one observation; returns False if it was rejected as impossible."""
        self.last_feed = now
        if kind == 'obstacle':
            self.obstacles[vehicle_id] = (x, y, 0.35, obs_t)
            return True
        if kind not in VEHICLE_KINDS:
            return True   # robots are not road traffic
        if not edge_id:
            self.latest.pop(vehicle_id, None)   # standing at a node: left its edge
            return True
        if not valid or obs_t > now + 0.1 or not math.isfinite(speed) or speed < 0:
            self.rejected += 1
            return False
        if edge_id not in self.edges:
            self.rejected += 1
            return False
        a, b = self.edges[edge_id]
        if point_segment_distance((x, y), hub_map.NODES[a], hub_map.NODES[b]) > CORRIDOR_M:
            self.rejected += 1
            return False
        if self.latest.get(vehicle_id, (None,))[0] == obs_t:
            return True  # repeated (frozen) observation: no new information
        self.latest[vehicle_id] = (obs_t, edge_id)
        self.samples.append((obs_t, vehicle_id, edge_id, speed))
        return True

    def estimate(self, now):
        self.samples = [s for s in self.samples if now - s[0] <= self.window_sec]
        self.obstacles = {k: v for k, v in self.obstacles.items()
                          if now - v[3] <= self.stale_sec}
        feed_fresh = now - self.last_feed <= self.stale_sec
        stale_edges = {e for vid, (t, e) in self.latest.items() if now - t > self.stale_sec
                       and now - t <= self.window_sec}
        out = []
        for eid, (a, b) in sorted(self.edges.items()):
            pa, pb = hub_map.NODES[a], hub_map.NODES[b]
            blocked = any(point_segment_distance((ox, oy), pa, pb) < r + self.swept_radius_m
                          for ox, oy, r, _ in self.obstacles.values())
            current = {vid for vid, (t, e) in self.latest.items()
                       if e == eid and now - t <= self.stale_sec}
            speeds = [s for t, vid, e, s in self.samples if e == eid]
            mean_speed = median(speeds) if speeds else 0.0
            out.append(EdgeEstimate(
                edge_id=eid,
                mean_speed_mps=mean_speed,
                density_ratio=min(1.0, len(current) / self.capacity(eid)),
                blocked=blocked,
                valid=feed_fresh and eid not in stale_edges,
                vehicle_count=len(current),
            ))
        return out

    def forget_vehicle(self, vehicle_id):
        self.latest.pop(vehicle_id, None)

    def reset(self):
        self.samples.clear()
        self.latest.clear()
        self.obstacles.clear()
