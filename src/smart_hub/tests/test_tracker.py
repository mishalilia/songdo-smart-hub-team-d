# Author: Andrey Li (12250249)
"""S1 tracking tests (T01 / T02): debounce, occlusion, crossing hold."""
from smart_hub.tracker import ZoneTracker


def test_tracker_debounce_and_short_occlusion():
    tr = ZoneTracker()
    p = [(3.0, 5.5)]
    tr.update(p, 0.0)
    assert tr.queue_count() == 0               # not confirmed before 0.4 s
    for t in (0.1, 0.2, 0.3, 0.4):
        tr.update(p, t)
    assert tr.queue_count() == 1
    tr.update([], 0.5)
    tr.update([], 0.7)                         # 0.3 s occlusion
    tr.update(p, 0.8)
    assert tr.queue_count() == 1 and len(tr.tracks) == 1


def test_tracker_crossing_hold():
    tr = ZoneTracker(crossing_clear_sec=2.0)
    for t in (0.0, 0.2, 0.4, 0.6):
        tr.update([(5.0, 3.0)], t)
    assert tr.crossing_occupied(0.6)
    tr.update([], 1.0)              # still a kept (occluded) track at 1.0 s
    tr.update([], 1.2)              # deleted after 0.5 s unseen
    assert tr.crossing_occupied(2.9)
    assert not tr.crossing_occupied(3.1)


def test_queue_membership_hysteresis():
    tr = ZoneTracker(zone_hysteresis_m=0.15)
    for t in (0.0, 0.2, 0.4, 0.6):
        tr.update([(2.05, 6.0)], t)                 # just inside Q (x from 2.0)
    assert tr.queue_count() == 1
    tr.update([(1.95, 6.0)], 0.8)                   # noise 5 cm outside: still counted
    assert tr.queue_count() == 1
    tr.update([(1.7, 6.0)], 1.0)                    # clearly outside: leaves the queue
    assert tr.queue_count() == 0
