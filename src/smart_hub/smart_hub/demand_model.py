# Author: Hsu Shwe Sin Htun (12240198)
"""
Thirty-second queue-demand forecast (assignment 9.2).

Features at 1 Hz sample t (queue counts only, so arrivals / departures are documented
proxies: positive / negative count changes over the last 30 s):
  x_t = [q_t, mean(q_{t-9..t}), q_t - q_{t-10}, arrivals_{t-29..t}, departures_{t-29..t}]
Persistence baseline: q_hat_{t+30} = q_t. Model: ridge regression on standardised features
(normalisation fitted on training episodes only), q_hat_{t+30} = max(0, f(x_t)).

A forecast needs a time-contiguous WARMUP_SEC (60 s) history of validated observations
(assignment 9.1, 9.3): QueueHistory gates every /hub/queue message on source, validity,
observation-stamp age and order, and restarts the warm-up on any gap. The offline
windows start after the same 60 s, so evaluation matches the online validity rule.
"""
from collections import deque
import json
import math

import numpy as np
from smart_hub.freshness import FUTURE_TOLERANCE_SEC, stamp_problem

HORIZON_SEC = 30
HISTORY_SEC = 31          # samples needed for the 30 s arrival / departure window
WARMUP_SEC = 60           # contiguous history required before a forecast is valid
FEATURES = ['q', 'mean10', 'trend10', 'arrivals30', 'departures30']


class QueueHistory:
    """
    One-second queue history that is valid only while time-contiguous.

    observe() gates each queue message; sample() runs once per period and appends the
    newest accepted observation or clears the history when there is none. Times: `now`
    and `stamp` are ROS-clock seconds, `monotonic` is the local receipt clock.
    """

    def __init__(self, window_sec=WARMUP_SEC, stale_sec=1.0, period_sec=1.0, zone_id='Q',
                 future_tolerance_sec=FUTURE_TOLERANCE_SEC):
        self.window_sec = window_sec
        self.stale_sec = stale_sec
        self.period_sec = period_sec
        self.zone_id = zone_id
        self.future_tolerance_sec = future_tolerance_sec
        self.samples = deque()      # (sample time, count, acquisition stamp)
        self.latest = None          # (count, acquisition stamp, receipt time)
        self.last_stamp = None      # newest accepted acquisition stamp

    def reset(self):
        self.samples.clear()
        self.latest = None
        self.last_stamp = None

    def observe(self, valid, count, stamp, zone_id, now, monotonic):
        """Gate one queue message; return '' when accepted, else the rejection reason."""
        if zone_id != self.zone_id:
            return 'WRONG_SOURCE'                  # another zone says nothing about Q
        if not valid:
            self.samples.clear()                   # explicit invalidity erases the past
            self.latest = None
            return 'INVALID'
        problem = stamp_problem(stamp, now, self.stale_sec, self.last_stamp,
                                self.future_tolerance_sec)
        if problem:
            return problem                         # not refreshed: the gap rule applies
        self.last_stamp = stamp
        self.latest = (int(count), stamp, monotonic)
        return ''

    def sample(self, now, monotonic):
        """Take the periodic sample; return True when a fresh observation was appended."""
        fresh = self.latest is not None and \
            monotonic - self.latest[2] <= self.stale_sec and \
            now - self.latest[1] <= 2.0 * self.stale_sec     # accepted age + receipt age
        if not fresh:
            self.samples.clear()                   # a gap breaks the feature windows
            return False
        if self.samples and monotonic - self.samples[-1][0] > 1.5 * self.period_sec:
            self.samples.clear()                   # missed samples: not contiguous
        self.samples.append((monotonic, self.latest[0], self.latest[1]))
        while self.span_sec() > self.window_sec + 0.5 * self.period_sec:
            self.samples.popleft()
        return True

    def span_sec(self):
        """Seconds covered by the contiguous history."""
        return self.samples[-1][0] - self.samples[0][0] if self.samples else 0.0

    def ready(self):
        return len(self.samples) >= HISTORY_SEC and \
            self.span_sec() >= self.window_sec - 0.5 * self.period_sec

    def counts(self):
        return [s[1] for s in self.samples]

    def acquisition_stamp(self):
        """Acquisition stamp of the newest supporting observation (None when empty)."""
        return self.samples[-1][2] if self.samples else None


def features(history):
    """Feature vector from 1 Hz counts (oldest first); None until HISTORY_SEC samples."""
    if len(history) < HISTORY_SEC:
        return None
    h = list(history)[-HISTORY_SEC:]
    diffs = [b - a for a, b in zip(h, h[1:])]
    return [float(h[-1]), sum(h[-10:]) / 10.0, float(h[-1] - h[-11]),
            float(sum(d for d in diffs if d > 0)), float(-sum(d for d in diffs if d < 0))]


def windows(series, warmup_sec=WARMUP_SEC):
    """
    (X, y, baseline) for one episode: features at t, target q_{t+30}, persistence q_t.

    The first window is at t = warmup_sec, when the online predictor first has a full
    contiguous history; features still use only samples up to t.
    """
    X, y, base = [], [], []
    for t in range(max(warmup_sec, HISTORY_SEC - 1), len(series) - HORIZON_SEC):
        x = features(series[:t + 1])
        X.append(x)
        y.append(float(series[t + HORIZON_SEC]))
        base.append(float(series[t]))
    return X, y, base


def split_episodes(n, train=0.6, val=0.2):
    """Whole-episode split (before windowing) -> index lists."""
    n_train, n_val = round(n * train), round(n * val)
    idx = list(range(n))
    return idx[:n_train], idx[n_train:n_train + n_val], idx[n_train + n_val:]


class RidgeModel:

    def __init__(self, weights, bias, mean, std, lam, model_id, residual_quantiles=None):
        self.weights = np.asarray(weights, dtype=float)
        self.bias = float(bias)
        self.mean = np.asarray(mean, dtype=float)
        self.std = np.asarray(std, dtype=float)
        self.lam = lam
        self.model_id = model_id
        # Quantiles of (truth - forecast) on the validation episodes, e.g. {'0.8': 1.66}.
        self.residual_quantiles = dict(residual_quantiles or {})

    @classmethod
    def fit(cls, X, y, lam, model_id='ridge'):
        X, y = np.asarray(X, dtype=float), np.asarray(y, dtype=float)
        mean = X.mean(axis=0)
        std = X.std(axis=0)
        std[std < 1e-9] = 1.0
        Z = (X - mean) / std
        y_mean = y.mean()
        A = Z.T @ Z + lam * np.eye(Z.shape[1])
        w = np.linalg.solve(A, Z.T @ (y - y_mean))   # intercept is not regularised
        return cls(w, y_mean, mean, std, lam, model_id)

    def predict(self, x):
        z = (np.asarray(x, dtype=float) - self.mean) / self.std
        return max(0.0, float(z @ self.weights + self.bias))

    def to_json(self):
        return {'model_id': self.model_id, 'type': 'ridge', 'features': FEATURES,
                'horizon_sec': HORIZON_SEC, 'lambda': self.lam,
                'weights': self.weights.tolist(), 'bias': self.bias,
                'mean': self.mean.tolist(), 'std': self.std.tolist(),
                'validation_residual_quantiles': self.residual_quantiles}

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d['weights'], d['bias'], d['mean'], d['std'], d['lambda'], d['model_id'],
                   d.get('validation_residual_quantiles'))

    def save(self, path):
        with open(path, 'w') as f:
            json.dump(self.to_json(), f, indent=2)


RESIDUAL_QUANTILES = (0.5, 0.8, 0.9)


def residual_quantiles(model, X, y, levels=RESIDUAL_QUANTILES):
    """Nearest-rank quantiles of truth - forecast (calibration on validation windows)."""
    r = sorted(float(t) - model.predict(x) for x, t in zip(X, y))
    return {'%g' % p: round(r[int(p * (len(r) - 1))], 4) for p in levels}


def mae(pred, truth):
    return sum(abs(p - t) for p, t in zip(pred, truth)) / len(truth)


def rmse(pred, truth):
    return math.sqrt(sum((p - t) ** 2 for p, t in zip(pred, truth)) / len(truth))


def paired_bootstrap(deltas, resamples=10000, seed=0):
    """
    Mean and 95 % percentile-bootstrap interval of per-episode paired differences.

    Whole episodes are resampled with replacement: overlapping windows (or passengers)
    inside one episode are not independent trials, so the episode is the unit.
    """
    d = np.asarray(deltas, dtype=float)
    if d.size == 0:
        return {'n': 0}
    rng = np.random.default_rng(seed)
    means = d[rng.integers(0, d.size, size=(resamples, d.size))].mean(axis=1)
    return {'n': int(d.size), 'mean': float(d.mean()),
            'ci95_low': float(np.percentile(means, 2.5)),
            'ci95_high': float(np.percentile(means, 97.5))}
