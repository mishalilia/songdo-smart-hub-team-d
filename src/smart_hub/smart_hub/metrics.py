# Author: Mikhail Li (12236295)
"""
Metric helpers for metrics_logger (assignment 21.2).

Nearest-rank percentiles, running series summaries, latency parsing from event reasons, the
strict ground-truth record of crossing C (CrossingMonitor) and the passenger ledger with
right-censored waits (QueueLedger).
"""
from collections import Counter
import math
import re

from smart_hub import hub_map

_LATENCY = re.compile(r'latency_ms=([0-9.]+)')
_WAIT = re.compile(r'wait_sec=([0-9.]+)')

AGENT_RADIUS = {'robot': 0.20, 'bus': 0.30, 'car': 0.30}
CONTROLLED = ('robot', 'bus')
STOPPED_MPS = 0.02
PAIR_CLASSES = ('controlled_controlled', 'controlled_car', 'controlled_pedestrian',
                'car_car', 'car_pedestrian')
ENTRY_KINDS = ('no_grant', 'pedestrian_present', 'car_against_owner')


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


def crossing_occupants(agents, pedestrians):
    """
    Return ({agent id: kind} with a footprint in C, number of pedestrians in C).

    `agents`: [(id, kind, x, y, speed)] ground truth (radius by kind); `pedestrians`:
    [(x, y)] ground-truth centres.
    """
    inside = {a: k for a, k, x, y, _ in agents
              if k in AGENT_RADIUS and in_crossing(x, y, AGENT_RADIUS[k])}
    return inside, sum(1 for x, y in pedestrians if in_crossing(x, y))


def pair_classes(inside, pedestrians):
    """Return the classes of simultaneous physical occupancy present in one sample."""
    controlled = sum(1 for k in inside.values() if k in CONTROLLED)
    cars = len(inside) - controlled
    present = {
        'controlled_controlled': controlled >= 2,
        'controlled_car': controlled >= 1 and cars >= 1,
        'controlled_pedestrian': controlled >= 1 and pedestrians >= 1,
        'car_car': cars >= 2,
        'car_pedestrian': cars >= 1 and pedestrians >= 1,
    }
    return {c for c in PAIR_CLASSES if present[c]}


class CrossingMonitor:
    """
    Strict ground-truth record of crossing C, one sample per truth message.

    Separate measures, none of them filtered by a grace interval or a speed threshold:
      strict occupancy     two or more users (robot, bus, car, pedestrian) physically in C
                           in the same sample, by pair class, as episodes and seconds;
      unauthorized entry   a footprint moving from outside to inside C without the right:
                           `no_grant` (robot / bus that is not the owner in a fresh
                           /crossing/state), `pedestrian_present` (robot / bus entering
                           while a pedestrian was already inside), `car_against_owner`
                           (background car entering a crossing owned by someone else);
      token overlap        a grant for a new token while an earlier token was still live;
      response latency     pedestrian enters C -> each robot / bus inside C measured at
                           standstill (ground-truth speed <= STOPPED_MPS).
    Contact is the simulator's COLLISION event and is counted by the logger. Every change
    of the occupant set, owner or token is returned as a raw trace row.
    """

    MAX_GAP_SEC = 0.5       # a longer gap between truth samples is not counted as occupancy

    def __init__(self):
        self.samples = 0
        self.last_t = None
        self.known = set()          # agent ids present in the previous sample
        self.inside = {}            # agent id -> kind inside C in the previous sample
        self.pedestrians = 0
        self.active = set()
        self.key = None
        self.episodes = Counter()
        self.seconds = Counter()
        self.unauthorized = Counter()
        self.response = Series()
        self.pending = {}           # agent id -> time the pedestrian entered
        self.unresolved = 0         # pedestrian left before the agent was at standstill
        self.exited = 0             # agent left C before it was at standstill
        self.live_token = ''
        self.token_overlaps = 0

    def on_event(self, event, token):
        """Follow the token lifecycle from the coordinator's events."""
        if event == 'CROSSING_GRANTED':
            if self.live_token and token != self.live_token:
                self.token_overlaps += 1
            self.live_token = token
        elif event == 'CROSSING_FREE' or \
                (event == 'CROSSING_RELEASED' and token == self.live_token):
            self.live_token = ''

    def sample(self, t, agents, pedestrians, owner, token=''):
        """
        Take one ground-truth sample at run time `t`.

        `owner` is the owner in a fresh /crossing/state ('' when free, None when the state
        is stale or invalid). Returns (trace row or None, [(event, entity, reason)]).
        """
        inside, peds = crossing_occupants(agents, pedestrians)
        speed = {a: s for a, _, _, _, s in agents}
        found = []
        dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), self.MAX_GAP_SEC)
        for c in self.active:
            self.seconds[c] += dt
        if self.active:
            self.seconds['any'] += dt

        for agent, kind in sorted(inside.items()):
            if agent in self.inside or agent not in self.known:
                continue                      # already inside, or first seen (spawned)
            kinds = []
            if kind in CONTROLLED:
                if owner != agent:
                    kinds.append('no_grant')
                if self.pedestrians:
                    kinds.append('pedestrian_present')
            elif owner and owner != agent:
                kinds.append('car_against_owner')
            for k in kinds:
                self.unauthorized[k] += 1
                found.append(('UNAUTHORIZED_ENTRY', agent, '%s owner=%s' % (
                    k, 'UNKNOWN' if owner is None else owner or 'FREE')))

        if peds and not self.pedestrians:
            for agent, kind in inside.items():
                if kind in CONTROLLED:
                    self.pending.setdefault(agent, t)
        for agent, since in sorted(self.pending.items()):
            if agent not in inside:
                self.exited += 1
            elif speed.get(agent, 0.0) <= STOPPED_MPS:
                self.response.add((t - since) * 1000.0)
                found.append(('PEDESTRIAN_RESPONSE', agent,
                              'latency_ms=%.0f' % ((t - since) * 1000.0)))
            elif not peds:
                self.unresolved += 1
                found.append(('PEDESTRIAN_RESPONSE', agent, 'unresolved'))
            else:
                continue
            del self.pending[agent]

        active = pair_classes(inside, peds)
        for c in sorted(active - self.active):
            self.episodes[c] += 1
        if active and not self.active:
            self.episodes['any'] += 1
            found.append(('STRICT_OCCUPANCY', 'crossing', '%s pedestrians=%d' % (
                ';'.join(sorted(inside)), peds)))

        key = (tuple(sorted(inside)), peds, owner, token)
        row = None
        if key != self.key:
            row = ['%.3f' % t, ';'.join(key[0]), peds,
                   'UNKNOWN' if owner is None else owner, token, ';'.join(sorted(active))]
        self.key, self.active = key, active
        self.known, self.inside, self.pedestrians = set(speed), inside, peds
        self.last_t = t
        self.samples += 1
        return row, found

    def summary(self):
        def cell(c):
            return {'episodes': self.episodes[c], 'seconds': round(self.seconds[c], 2)}
        return {
            'truth_samples': self.samples,
            'strict_occupancy': dict(cell('any'), by_class={c: cell(c) for c in PAIR_CLASSES}),
            'unauthorized_entries': {k: self.unauthorized[k] for k in ENTRY_KINDS},
            'token_overlaps': self.token_overlaps,
            'pedestrian_response_ms': self.response.summary(),
            'pedestrian_response_unresolved': self.unresolved + len(self.pending),
            'pedestrian_response_exited': self.exited,
        }


class QueueLedger:
    """
    Every passenger of the run, not only those who boarded (assignment 21.2).

    A passenger enters the queue, then boards, abandons the queue, or is still waiting when
    the run ends (right-censored: the recorded wait is a lower bound). Arrivals refused
    because the queue zone was full are counted as balked.
    """

    def __init__(self):
        self.entered = {}           # passenger id -> entry time, still waiting
        self.arrived = 0
        self.balked = 0
        self.boarded = Series()
        self.abandoned = Series()

    def enter(self, passenger, t):
        self.entered[passenger] = t
        self.arrived += 1

    def board(self, passenger, t, wait=None):
        since = self.entered.pop(passenger, None)
        if wait is None and since is not None:
            wait = t - since
        if wait is not None:
            self.boarded.add(wait)

    def abandon(self, passenger, t):
        since = self.entered.pop(passenger, None)
        if since is not None:
            self.abandoned.add(t - since)

    def summary(self, t):
        censored = Series()
        for since in self.entered.values():
            censored.add(max(0.0, t - since))
        everyone = Series()
        for v in self.boarded.values + self.abandoned.values + censored.values:
            everyone.add(v)
        arrived = max(self.arrived, len(everyone.values))
        done = len(self.boarded.values)
        return {
            'arrived': arrived, 'boarded': done, 'abandoned': len(self.abandoned.values),
            'waiting_at_end': len(censored.values), 'balked': self.balked,
            'completion_rate': round(done / arrived, 4) if arrived else None,
            'clearance_rate': round(1.0 - len(censored.values) / arrived, 4)
            if arrived else None,
            'wait_boarded_sec': self.boarded.summary(),
            'wait_abandoned_sec': self.abandoned.summary(),
            'wait_censored_sec': censored.summary(),
            'wait_all_lower_bound_sec': everyone.summary(),
        }


def relaxed_crossing_conflict(agents, pedestrians):
    """
    Return True under the midterm definition, kept only as a labelled diagnostic.

    It ignores car-only occupancy and a pedestrian next to a stationary robot / bus, so it
    undercounts; the reported measures come from CrossingMonitor.
    """
    inside = [(k, s) for _, k, x, y, s in agents
              if k in AGENT_RADIUS and in_crossing(x, y, AGENT_RADIUS[k])]
    controlled = [(k, s) for k, s in inside if k in CONTROLLED]
    if not controlled:
        return False
    if len(inside) >= 2:
        return True
    peds = any(in_crossing(x, y) for x, y in pedestrians)
    return peds and any(speed > STOPPED_MPS for _, speed in controlled)
