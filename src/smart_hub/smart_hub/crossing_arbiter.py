# Author: Komronbek Tohirov (12235649)
"""
Crossing C mutual exclusion (assignment 7.1, 15.2, 18.4).

One owner and token at a time. Pedestrians have priority: a pedestrian in C or an unknown
crossing state denies new grants. A new grant also needs C observed clear (no pedestrian,
no agent footprint) for `clear_sec`. A lost owner heartbeat (lease expiry) causes a HOLD
until C is observed empty; ownership is never transferred while a footprint occupies C.
Waiting requesters are served FIFO from a bounded queue.
"""
from uuid import uuid4

FREE, OWNED, HOLD, UNKNOWN = 'FREE', 'OWNED', 'HOLD', 'UNKNOWN'


class CrossingArbiter:

    def __init__(self, clear_sec=2.0, lease_sec=2.0, max_queue=8):
        self.clear_sec = clear_sec
        self.lease_sec = lease_sec
        self.max_queue = max_queue
        self.owner = ''
        self.token = ''
        self.acquired_at = 0.0
        self.lease_expiry = 0.0
        self.hold = False
        self.queue = []
        self.last_request = {}          # requester -> time of its latest request
        self.queue_timeout_sec = 3.0
        self.pedestrian_present = False
        self.observation_valid = False
        self.occupants = set()          # agent ids whose footprint is inside C
        self._clear_since = None

    # ------------------------------------------------------------------ observations
    def observe(self, now, pedestrian_present, observation_valid, occupants):
        self.pedestrian_present = bool(pedestrian_present)
        self.observation_valid = bool(observation_valid)
        self.occupants = set(occupants)
        clear = self.observation_valid and not self.pedestrian_present and not self.occupants
        if not clear:
            self._clear_since = None
        elif self._clear_since is None:
            self._clear_since = now
        self._expire(now)

    def clear_for(self, now):
        return 0.0 if self._clear_since is None else now - self._clear_since

    def _expire(self, now):
        # Requesters that stopped retrying leave the queue (bounded, no stale head).
        self.queue = [r for r in self.queue
                      if now - self.last_request.get(r, now) <= self.queue_timeout_sec]
        if self.owner and not self.hold and now > self.lease_expiry:
            self.hold = True            # owner heartbeat lost
        if self.hold and self.clear_for(now) >= self.clear_sec:
            self._free()

    def _free(self):
        self.owner = self.token = ''
        self.hold = False

    @property
    def state(self):
        if not self.observation_valid:
            return UNKNOWN
        if self.hold:
            return HOLD
        return OWNED if self.owner else FREE

    # ------------------------------------------------------------------ requests
    def request(self, requester, operation, token, now):
        """Return (granted, token, reason)."""
        self.last_request[requester] = now
        self._expire(now)
        if operation == 'RENEW':
            if token and token == self.token and not self.hold:
                self.lease_expiry = now + self.lease_sec
                return True, self.token, 'RENEWED'
            return False, '', 'TOKEN_MISMATCH'
        if operation == 'RELEASE':
            if not token or token != self.token:
                return False, '', 'TOKEN_MISMATCH'
            if requester in self.occupants:
                return False, self.token, 'NOT_CLEAR'
            self._free()
            return True, '', 'RELEASED'
        if operation != 'ACQUIRE':
            return False, '', 'INVALID_OPERATION'
        if self.owner == requester and not self.hold:
            self.lease_expiry = now + self.lease_sec
            return True, self.token, 'ALREADY_OWNER'
        if requester not in self.queue:
            if len(self.queue) >= self.max_queue:
                return False, '', 'QUEUE_FULL'
            self.queue.append(requester)
        if not self.observation_valid:
            return False, '', 'CROSSING_UNKNOWN'
        if self.pedestrian_present:
            return False, '', 'PEDESTRIAN_PRESENT'
        if self.owner or self.hold:
            return False, '', 'OCCUPIED_BY_' + (self.owner or 'HOLD')
        if self.queue[0] != requester:
            return False, '', 'QUEUED_BEHIND_' + self.queue[0]
        others = self.occupants - {requester}
        if others:
            return False, '', 'FOOTPRINT_IN_C'
        if self.clear_for(now) < self.clear_sec and requester not in self.occupants:
            return False, '', 'CLEAR_DEBOUNCE'
        self.queue.pop(0)
        self.owner, self.token = requester, uuid4().hex
        self.acquired_at, self.lease_expiry = now, now + self.lease_sec
        return True, self.token, 'GRANTED'

    def forget(self, requester):
        if requester in self.queue:
            self.queue.remove(requester)
