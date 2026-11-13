# Author: Komronbek Tohirov (12235649)
"""
Demand-driven bus dispatch policy (assignment 9.3).

Request one DockBus mission when the forecast is >= dispatch_high for three consecutive
valid predictions and the bus is available; with an unavailable forecast fall back to the
fresh observed queue (reason FALLBACK). One outstanding goal at a time, a request cooldown,
and hysteresis: escalation clears only after five consecutive samples below dispatch_low.
With observed_floor (decision-aware use of the forecast), a fresh observed queue >= high also
triggers in predictive mode (reason OBSERVED): the forecast may make a dispatch earlier, never
later than the observation already justifies. A model fitted for MAE under-predicts a rising
queue, so without the floor it delayed the first dispatch by up to 40 s in S2.
With rearm_after_service, a mission that ended with passengers boarded also clears it: the
escalated demand was served, and a queue that is still high has to be confirmed again by
three new consecutive predictions (or the fallback rule) before the next request. Without
it a queue that never falls below dispatch_low keeps the bus at one trip per episode.

A forecast counts as valid only after forecast_problem() accepted it: explicit validity,
the expected source (30 s horizon, named model) and the age and order of its supporting
acquisition stamp. Anything else is "prediction unavailable" and uses the fallback.
"""
from smart_hub.freshness import stamp_problem

FORECAST_HORIZON_SEC = 30
FORECAST_STALE_SEC = 3.0     # 1 Hz forecast + supporting queue sample at most 2 s old


def forecast_problem(valid, horizon_sec, model_id, stamp, now, last_stamp=None,
                     max_age_sec=FORECAST_STALE_SEC):
    """Return '' when a /hub/demand message may drive dispatch, else the reason."""
    if not valid:
        return 'INVALID'
    if horizon_sec != FORECAST_HORIZON_SEC or not model_id:
        return 'WRONG_SOURCE'
    problem = stamp_problem(stamp, now, max_age_sec)
    if not problem and last_stamp is not None and stamp < last_stamp:
        problem = 'OUT_OF_ORDER'
    return problem


class DispatchPolicy:

    def __init__(self, high=6.0, low=4.0, cooldown_sec=30.0, consecutive=3, clear_after=5,
                 rearm_after_service=False, observed_floor=False):
        self.high, self.low = high, low
        self.observed_floor = observed_floor
        self.rearm_after_service = rearm_after_service
        self.cooldown_sec = cooldown_sec
        self.consecutive = consecutive
        self.clear_after = clear_after
        self.high_streak = 0
        self.low_streak = 0
        self.escalated = False
        self.last_request = -1e9

    def served(self):
        """Record a mission that ended with boarding; return True if escalation cleared."""
        if not (self.rearm_after_service and self.escalated):
            return False
        self.escalated = False
        self.high_streak = 0
        return True

    def update(self, now, prediction_valid, predicted, queue_valid, queue_count,
               bus_available, outstanding):
        """Return 'PREDICTED', 'OBSERVED' or 'FALLBACK' to request a mission, else None."""
        if prediction_valid:
            value, reason = predicted, 'PREDICTED'
            self.high_streak = self.high_streak + 1 if predicted >= self.high else 0
            trigger = self.high_streak >= self.consecutive
            if self.observed_floor and queue_valid:
                value = max(predicted, queue_count)
                if not trigger and queue_count >= self.high:
                    trigger, reason = True, 'OBSERVED'
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
