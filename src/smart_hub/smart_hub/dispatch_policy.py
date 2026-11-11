# Author: Komronbek Tohirov (12235649)
"""
Demand-driven bus dispatch policy (assignment 9.3).

Request one DockBus mission when the forecast is >= dispatch_high for three consecutive
valid predictions and the bus is available; with an unavailable forecast fall back to the
fresh observed queue (reason FALLBACK). One outstanding goal at a time, a request cooldown,
and hysteresis: escalation clears only after five consecutive samples below dispatch_low.
"""


class DispatchPolicy:

    def __init__(self, high=6.0, low=4.0, cooldown_sec=30.0, consecutive=3, clear_after=5):
        self.high, self.low = high, low
        self.cooldown_sec = cooldown_sec
        self.consecutive = consecutive
        self.clear_after = clear_after
        self.high_streak = 0
        self.low_streak = 0
        self.escalated = False
        self.last_request = -1e9

    def update(self, now, prediction_valid, predicted, queue_valid, queue_count,
               bus_available, outstanding):
        """Return 'PREDICTED' or 'FALLBACK' when a mission should be requested, else None."""
        if prediction_valid:
            value, reason = predicted, 'PREDICTED'
            self.high_streak = self.high_streak + 1 if predicted >= self.high else 0
            trigger = self.high_streak >= self.consecutive
        elif queue_valid:
            value, reason = queue_count, 'FALLBACK'
            self.high_streak = 0
            trigger = queue_count >= self.high
        else:
            self.high_streak = 0
            return None                      # no valid demand information: never dispatch
        self.low_streak = self.low_streak + 1 if value < self.low else 0
        if self.low_streak >= self.clear_after:
            self.escalated = False
        if not trigger or self.escalated or outstanding or not bus_available:
            return None
        if now - self.last_request < self.cooldown_sec:
            return None
        self.escalated = True
        self.last_request = now
        return reason
