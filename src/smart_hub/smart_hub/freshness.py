# Author: Mikhail Li (12236295)
"""
Freshness gate for retained ROS state (assignment 5, 6.1, 16.1).

Reliable or transient-local delivery is not a freshness guarantee: a consumer may use a
sample only if it is explicitly valid, was received locally within `max_age_sec`
(monotonic clock) and its observation stamp is neither older than `max_age_sec` nor in
the future beyond FUTURE_TOLERANCE_SEC (ROS clock). Unknown state is never permission.
"""
FUTURE_TOLERANCE_SEC = 0.2


def stamp_sec(stamp):
    """Return a builtin_interfaces/Time as float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_problem(stamp, now, max_age_sec, last_stamp=None,
                  future_tolerance_sec=FUTURE_TOLERANCE_SEC):
    """
    Return '' for a usable observation stamp, else the rejection reason.

    `stamp`, `now` and `last_stamp` are seconds on the same (ROS) clock; `last_stamp` is
    the previously accepted stamp of the same source (None skips the ordering checks).
    """
    if stamp <= 0.0:
        return 'NO_STAMP'
    if stamp > now + future_tolerance_sec:
        return 'FUTURE_STAMP'
    if last_stamp is not None and stamp < last_stamp:
        return 'OUT_OF_ORDER'
    if last_stamp is not None and stamp == last_stamp:
        return 'REPLAY'
    if now - stamp > max_age_sec:
        return 'STALE_STAMP'
    return ''


class Retained:
    """Latest message of one topic; get() returns it only while valid and fresh."""

    def __init__(self, max_age_sec, check_valid=True):
        self.max_age_sec = max_age_sec
        self.check_valid = check_valid
        self.msg = None
        self.receipt = -1e9

    def put(self, msg, receipt):
        """Store a message with its local monotonic receipt time."""
        self.msg, self.receipt = msg, receipt

    def clear(self):
        self.msg, self.receipt = None, -1e9

    def problem(self, now, monotonic):
        """Return '' when usable; `now` is ROS time, `monotonic` the local clock (s)."""
        if self.msg is None:
            return 'MISSING'
        if self.check_valid and not self.msg.valid:
            return 'INVALID'
        if monotonic - self.receipt > self.max_age_sec:
            return 'STALE_RECEIPT'
        return stamp_problem(stamp_sec(self.msg.header.stamp), now, self.max_age_sec)

    def get(self, now, monotonic):
        return None if self.problem(now, monotonic) else self.msg
