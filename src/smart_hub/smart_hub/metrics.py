# Author: Mikhail Li (12236295)
"""
Metric helpers for metrics_logger (assignment 21.2).

Nearest-rank percentiles, running series summaries, latency parsing from event reasons and
crossing-conflict detection from ground truth.
"""
import math
import re

from smart_hub import hub_map

_LATENCY = re.compile(r'latency_ms=([0-9.]+)')
_WAIT = re.compile(r'wait_sec=([0-9.]+)')


def percentile(values, q):
    """Return the nearest-rank percentile, q in 0..100, of a non-empty sequence."""
    ordered = sorted(values)
    rank = max(1, math.ceil(q / 100.0 * len(ordered)))
    return ordered[rank - 1]


class Series:

    def __init__(self, keep=10000):
        self.values = []
        self.keep = keep

    def add(self, value):
        self.values.append(float(value))
        del self.values[:-self.keep]

    def summary(self):
        v = self.values
        if not v:
            return {'n': 0}
        return {'n': len(v), 'mean': sum(v) / len(v), 'p50': percentile(v, 50),
                'p95': percentile(v, 95), 'max': max(v)}


def parse_latency_ms(reason):
    m = _LATENCY.search(reason or '')
    return float(m.group(1)) if m else None


def parse_wait_sec(reason):
    m = _WAIT.search(reason or '')
    return float(m.group(1)) if m else None


def in_crossing(x, y, radius=0.0):
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    return xmin - radius < x < xmax + radius and ymin - radius < y < ymax + radius


def crossing_conflict(agents, pedestrians):
    """
    Return True for simultaneous incompatible occupancy of C.

    `agents`: [(id, kind, x, y, speed)] (radius by kind); `pedestrians`: [(x, y)].
    Incompatible (assignment 21.2, physical occupancy): a hub-controlled agent (robot or
    bus) inside C together with another agent, or moving while a pedestrian is inside C.
    Scripted background cars are not arbitrated, so car-car overlaps are not counted.
    """
    radius = {'robot': 0.20, 'bus': 0.30, 'car': 0.30}
    inside = [(k, s) for _, k, x, y, s in agents
              if k in radius and in_crossing(x, y, radius[k])]
    controlled = [(k, s) for k, s in inside if k in ('robot', 'bus')]
    if not controlled:
        return False
    if len(inside) >= 2:
        return True
    peds = any(in_crossing(x, y) for x, y in pedestrians)
    return peds and any(speed > 0.02 for _, speed in controlled)
