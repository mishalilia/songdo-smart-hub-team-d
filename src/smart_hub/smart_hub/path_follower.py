# Author: Timur Klechshev (12235650)
"""
Waypoint tracking for the assistance robot (assignment 14.2 step 3).

Heading-error controller with bounded speed: no forward motion while the heading error is
large, angular rate capped, slow approach to the final waypoint.
"""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def follow(x, y, yaw, tx, ty, v_max, w_max, final=False, k_w=2.0,
           max_heading_error=0.6, approach_gain=0.6, min_speed=0.05):
    """Return (v, w, distance) to drive from pose (x, y, yaw) towards (tx, ty)."""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return 0.0, 0.0, 0.0
    err = wrap(math.atan2(dy, dx) - yaw)
    w = max(-w_max, min(w_max, k_w * err))
    if abs(err) > max_heading_error:
        return 0.0, w, dist
    v = v_max * math.cos(err)
    if final:
        v = min(v, max(min_speed, approach_gain * dist))
    return v, w, dist


def route_length(nodes, positions):
    return sum(math.dist(positions[a], positions[b]) for a, b in zip(nodes, nodes[1:]))
