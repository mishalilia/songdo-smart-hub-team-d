# Author: Hsu Shwe Sin Htun (12240198)
"""S2 demand model tests (T03): features, episode split, ridge, persistence."""
import math

from smart_hub.demand_model import (
    features, HISTORY_SEC, mae, paired_bootstrap, QueueHistory, RidgeModel, split_episodes,
    WARMUP_SEC, windows,
)


def test_features_need_full_history():
    assert features([1] * (HISTORY_SEC - 1)) is None
    h = [0] * 20 + [1, 2, 3, 3, 2, 2, 4, 4, 4, 3, 5]
    q, mean10, trend, arrivals, departures = features(h)
    assert q == 5 and math.isclose(mean10, sum(h[-10:]) / 10)
    assert trend == h[-1] - h[-11]
    assert arrivals == 7 and departures == 2       # +1+1+1+2+1 +... and -1 -1


def test_windows_use_only_past_for_features_and_future_for_target():
    series = list(range(200))
    X, y, base = windows(series)
    # The first window needs the same 60 s history as the online predictor.
    assert len(X) == len(series) - WARMUP_SEC - 30
    assert X[0][0] == WARMUP_SEC and y[0] == WARMUP_SEC + 30 and base[0] == X[0][0]
    X, _, _ = windows(series, warmup_sec=0)
    assert X[0][0] == HISTORY_SEC - 1


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


def feed(history, seconds, start=0.0, count=3):
    """Feed a 5 Hz valid queue stream and sample at 1 Hz; return the last time."""
    t = start
    for i in range(seconds * 5 + 1):
        t = start + i * 0.2
        assert history.observe(True, count, 1000.0 + t, 'Q', 1000.0 + t + 0.05, t) == ''
        if i % 5 == 0:
            history.sample(1000.0 + t + 0.05, t)
    return t


def test_history_needs_sixty_contiguous_seconds():
    h = QueueHistory()
    feed(h, 59)
    assert len(h.samples) == 60 and not h.ready()     # 60 samples span only 59 s
    t = feed(h, 1, start=60.0)
    assert h.ready() and h.span_sec() >= 60.0 - 1e-6
    assert h.acquisition_stamp() == 1000.0 + t        # supporting acquisition time kept
    feed(h, 30, start=62.0)
    assert h.ready() and h.span_sec() <= 60.5         # old samples leave the window


def test_explicit_invalid_message_erases_history_immediately():
    h = QueueHistory()
    t = feed(h, 61)
    assert h.ready()
    assert h.observe(False, 0, 1000.0 + t + 0.2, 'Q', 1000.0 + t + 0.25, t + 0.2) == 'INVALID'
    assert not h.ready() and not h.samples and h.latest is None
    # The previous valid sample must not be reused by the next 1 Hz tick.
    assert not h.sample(1000.0 + t + 0.3, t + 0.3)


def test_history_rejects_bad_source_and_stamps():
    h = QueueHistory()
    t = feed(h, 61)
    now = 1000.0 + t
    assert h.observe(True, 9, now + 0.2, 'P', now + 0.2, t + 0.2) == 'WRONG_SOURCE'
    assert h.observe(True, 9, now - 0.2, 'Q', now + 0.2, t + 0.2) == 'OUT_OF_ORDER'
    assert h.observe(True, 9, now, 'Q', now + 0.2, t + 0.2) == 'REPLAY'
    assert h.observe(True, 9, now + 5.0, 'Q', now + 0.2, t + 0.2) == 'FUTURE_STAMP'
    assert h.observe(True, 9, now + 0.1, 'Q', now + 3.0, t + 3.0) == 'STALE_STAMP'
    assert h.latest[0] == 3                           # none of them replaced the sample
    # Rejected input does not refresh the receipt: the feed gap restarts the warm-up.
    assert not h.sample(now + 3.0, t + 3.0) and not h.ready()


def test_missed_samples_break_contiguity():
    h = QueueHistory()
    t = feed(h, 61)
    assert h.ready()
    late = t + 2.6                                    # two 1 Hz ticks were skipped
    assert h.observe(True, 3, 1000.0 + late, 'Q', 1000.0 + late, late) == ''
    assert h.sample(1000.0 + late, late)
    assert len(h.samples) == 1 and not h.ready()


def test_paired_bootstrap_resamples_whole_episodes():
    out = paired_bootstrap([-1.0, -0.5, -0.8, -0.2, -0.6, -0.9])
    assert out['n'] == 6 and math.isclose(out['mean'], -4.0 / 6)
    assert out['ci95_low'] <= out['mean'] <= out['ci95_high'] < 0
    assert paired_bootstrap([]) == {'n': 0}
