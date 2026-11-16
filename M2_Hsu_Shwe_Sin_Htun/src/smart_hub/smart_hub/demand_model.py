# Author: Hsu Shwe Sin Htun (12240198)
"""
Thirty-second queue-demand forecast (assignment 9.2).

Features at 1 Hz sample t (queue counts only, so arrivals / departures are documented
proxies: positive / negative count changes over the last 30 s):
  x_t = [q_t, mean(q_{t-9..t}), q_t - q_{t-10}, arrivals_{t-29..t}, departures_{t-29..t}]
Persistence baseline: q_hat_{t+30} = q_t. Model: ridge regression on standardised features
(normalisation fitted on training episodes only), q_hat_{t+30} = max(0, f(x_t)).
"""
import json
import math

import numpy as np

HORIZON_SEC = 30
HISTORY_SEC = 31          # samples needed for the 30 s arrival / departure window
FEATURES = ['q', 'mean10', 'trend10', 'arrivals30', 'departures30']


def features(history):
    """Feature vector from 1 Hz counts (oldest first); None until HISTORY_SEC samples."""
    if len(history) < HISTORY_SEC:
        return None
    h = list(history)[-HISTORY_SEC:]
    diffs = [b - a for a, b in zip(h, h[1:])]
    return [float(h[-1]), sum(h[-10:]) / 10.0, float(h[-1] - h[-11]),
            float(sum(d for d in diffs if d > 0)), float(-sum(d for d in diffs if d < 0))]


def windows(series):
    """(X, y, baseline) for one episode: features at t, target q_{t+30}, persistence q_t."""
    X, y, base = [], [], []
    for t in range(HISTORY_SEC - 1, len(series) - HORIZON_SEC):
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

    def __init__(self, weights, bias, mean, std, lam, model_id):
        self.weights = np.asarray(weights, dtype=float)
        self.bias = float(bias)
        self.mean = np.asarray(mean, dtype=float)
        self.std = np.asarray(std, dtype=float)
        self.lam = lam
        self.model_id = model_id

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
                'mean': self.mean.tolist(), 'std': self.std.tolist()}

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(d['weights'], d['bias'], d['mean'], d['std'], d['lambda'], d['model_id'])

    def save(self, path):
        with open(path, 'w') as f:
            json.dump(self.to_json(), f, indent=2)


def mae(pred, truth):
    return sum(abs(p - t) for p, t in zip(pred, truth)) / len(truth)


def rmse(pred, truth):
    return math.sqrt(sum((p - t) ** 2 for p, t in zip(pred, truth)) / len(truth))
