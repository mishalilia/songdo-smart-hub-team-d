# Author: Hsu Shwe Sin Htun (12240198)
"""S2 demand model tests (T03): features, episode split, ridge, persistence."""
import math

from smart_hub.demand_model import (
    features, HISTORY_SEC, mae, RidgeModel, split_episodes, windows,
)


def test_features_need_full_history():
    assert features([1] * (HISTORY_SEC - 1)) is None
    h = [0] * 20 + [1, 2, 3, 3, 2, 2, 4, 4, 4, 3, 5]
    q, mean10, trend, arrivals, departures = features(h)
    assert q == 5 and math.isclose(mean10, sum(h[-10:]) / 10)
    assert trend == h[-1] - h[-11]
    assert arrivals == 7 and departures == 2       # +1+1+1+2+1 +... and -1 -1


def test_windows_use_only_past_for_features_and_future_for_target():
    series = list(range(100))
    X, y, base = windows(series)
    assert len(X) == len(series) - HISTORY_SEC + 1 - 30
    assert X[0][0] == HISTORY_SEC - 1 and y[0] == HISTORY_SEC - 1 + 30 and base[0] == X[0][0]


def test_split_is_by_whole_episode():
    train, val, test = split_episodes(30)
    assert (len(train), len(val), len(test)) == (18, 6, 6)
    assert not set(train) & set(val) and not set(val) & set(test)


def test_ridge_learns_linear_target_and_is_nonnegative():
    X = [[i, i / 2, 1, 0, 0] for i in range(50)]
    y = [2 * i + 1 for i in range(50)]
    model = RidgeModel.fit(X, y, lam=1e-6)
    assert mae([model.predict(x) for x in X], y) < 1e-3
    assert model.predict([-100, -50, 1, 0, 0]) == 0.0


def test_model_round_trip(tmp_path):
    model = RidgeModel.fit([[1, 2, 3, 4, 5], [2, 3, 4, 5, 6], [0, 1, 0, 1, 0]], [1, 2, 0], 1.0)
    path = tmp_path / 'm.json'
    model.save(path)
    loaded = RidgeModel.load(path)
    assert math.isclose(loaded.predict([1, 2, 3, 4, 5]), model.predict([1, 2, 3, 4, 5]))
