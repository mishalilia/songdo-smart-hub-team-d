# Author: Timur Klechshev (12235650)
"""Robot waypoint follower tests (14.2)."""
import math

from smart_hub.path_follower import follow


def test_follower_turns_in_place_for_large_heading_error():
    v, w, _ = follow(0.0, 0.0, 0.0, -1.0, 0.0, 0.25, 0.8)
    assert v == 0.0 and abs(w) == 0.8
    v, w, _ = follow(0.0, 0.0, 0.0, 1.0, 0.0, 0.25, 0.8)
    assert math.isclose(v, 0.25) and w == 0.0
