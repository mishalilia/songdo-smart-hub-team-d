# Author: Andrey Li (12250249)
"""
Pedestrian tracking and zone counting (assignment 8.2, 18.1).

Synthetic baseline: associates map-frame foot-points over time (nearest neighbour with a
gate), confirms a track after it has been seen for `confirm_sec`, keeps it through short
occlusions (`occlusion_sec`), and counts confirmed tracks inside Q and C. Crossing
occupancy is held until C has been clear for `crossing_clear_sec`.
"""
from dataclasses import dataclass
import math

from smart_hub import hub_map


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    first_seen: float
    last_seen: float
    confirmed: bool = False
    in_queue: bool = False


class ZoneTracker:

    def __init__(self, confirm_sec=0.4, occlusion_sec=0.5, gate_m=0.5, crossing_clear_sec=2.0,
                 zone_hysteresis_m=0.15):
        self.zone_hysteresis_m = zone_hysteresis_m
        self.confirm_sec = confirm_sec
        self.occlusion_sec = occlusion_sec
        self.gate_m = gate_m
        self.crossing_clear_sec = crossing_clear_sec
        self.tracks = {}
        self._next_id = 1
        self._crossing_last_seen = -math.inf

    def update(self, points, now):
        """Associate one frame of (x, y) foot-points observed at time `now`."""
        unmatched = set(self.tracks)
        for x, y in points:
            best, best_d = None, self.gate_m
            for tid in unmatched:
                t = self.tracks[tid]
                d = math.hypot(t.x - x, t.y - y)
                if d < best_d:
                    best, best_d = tid, d
            if best is None:
                self.tracks[self._next_id] = Track(self._next_id, x, y, now, now)
                self._next_id += 1
                continue
            unmatched.discard(best)
            t = self.tracks[best]
            t.x, t.y, t.last_seen = x, y, now
            if now - t.first_seen >= self.confirm_sec - 1e-9:
                t.confirmed = True
        for tid in list(self.tracks):
            t = self.tracks[tid]
            if now - t.last_seen > self.occlusion_sec + 1e-9:
                del self.tracks[tid]
        if any(t.confirmed and hub_map.point_in_rect((t.x, t.y), hub_map.CROSSING_C)
               for t in self.tracks.values()):
            self._crossing_last_seen = now

    def queue_count(self):
        """
        Count confirmed tracks in Q with hysteresis.

        A track joins Q when its foot-point is inside Q and leaves only when it is more than
        `zone_hysteresis_m` outside, so measurement noise at the boundary cannot flicker
        the count.
        """
        xmin, xmax, ymin, ymax = hub_map.QUEUE_Q
        m = self.zone_hysteresis_m
        for t in self.tracks.values():
            if hub_map.point_in_rect((t.x, t.y), hub_map.QUEUE_Q):
                t.in_queue = True
            elif not hub_map.point_in_rect((t.x, t.y), (xmin - m, xmax + m, ymin - m, ymax + m)):
                t.in_queue = False
        return sum(1 for t in self.tracks.values() if t.confirmed and t.in_queue)

    def crossing_occupied(self, now):
        return now - self._crossing_last_seen < self.crossing_clear_sec

    def reset(self):
        self.tracks.clear()
        self._crossing_last_seen = -math.inf
