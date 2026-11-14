# Author: Hsu Shwe Sin Htun (12240198)
"""
Train and evaluate the S2 demand model (assignment 9.2, 9.4).

1. Generate independent six-minute episodes in the simulator (sim_world, no ROS): varied
   arrival rates per minute and boarding events (up to 4 passengers leave).
2. Split whole episodes 60/20/20 into train / validation / test before windowing.
3. Fit normalisation and ridge regression on training episodes, choose lambda on
   validation, report held-out MAE / RMSE against the persistence baseline and the
   inference latency.
4. Uncertainty: overlapping windows of one episode are not independent trials, so the
   errors are also reported per held-out episode, with a 95 % bootstrap interval of the
   paired difference (model MAE - persistence MAE) that resamples whole episodes. Extra
   unseen episodes (--holdout-extra, never used for fitting or selection) widen that
   sample beyond the six test episodes of the 60/20/20 split.

  ros2 run smart_hub train_demand_model --episodes 30 \
      --model-out src/smart_hub/models/demand_ridge.json --evidence-dir evidence/demand
"""
import argparse
import csv
import json
import os
import random
import time

from smart_hub.demand_model import (
    features, HISTORY_SEC, mae, paired_bootstrap, residual_quantiles, RidgeModel, rmse,
    split_episodes, WARMUP_SEC, windows,
)
from smart_hub.metrics import percentile
from smart_hub.sim_world import STEP_SEC, World

EPISODE_SEC = 360
LAMBDAS = (0.01, 0.1, 1.0, 10.0, 100.0, 1000.0, 10000.0)


def episode_events(rng):
    """Arrival-rate schedule (per minute) and boarding events for one episode."""
    events = []
    for minute in range(EPISODE_SEC // 60):
        events.append({'t': minute * 60.0, 'type': 'arrival_rate',
                       'per_min': round(rng.uniform(0.0, 10.0), 2)})
    t = rng.uniform(40.0, 100.0)
    while t < EPISODE_SEC:
        events.append({'t': round(t, 1), 'type': 'remove_queue_tracks', 'count': 4})
        t += rng.uniform(45.0, 120.0)
    return events


def generate_episode(seed):
    """1 Hz queue counts of one simulated episode."""
    rng = random.Random(seed)
    world = World({'duration_sec': EPISODE_SEC, 'events': episode_events(rng)}, seed)
    counts = []
    steps_per_sec = round(1.0 / STEP_SEC)
    for i in range(EPISODE_SEC * steps_per_sec):
        if i % steps_per_sec == 0:
            counts.append(len(world.queue_peds()))
        world.step()
    return counts


def stack(episodes, ids):
    X, y, base = [], [], []
    for i in ids:
        a, b, c = windows(episodes[i])
        X += a
        y += b
        base += c
    return X, y, base


def episode_errors(model, seed, series):
    """Errors of one held-out episode (the independent unit of the evaluation)."""
    X, y, base = windows(series)
    preds = [model.predict(x) for x in X]
    row = {'seed': seed, 'windows': len(y),
           'model_mae': round(mae(preds, y), 4), 'model_rmse': round(rmse(preds, y), 4),
           'persistence_mae': round(mae(base, y), 4),
           'persistence_rmse': round(rmse(base, y), 4)}
    row['delta_mae'] = round(row['model_mae'] - row['persistence_mae'], 4)
    return row


def episode_uncertainty(rows):
    """Episode-level summary: mean MAE and bootstrap CI of the paired MAE difference."""
    if not rows:
        return {'episodes': 0}
    boot = paired_bootstrap([r['delta_mae'] for r in rows])
    return {
        'episodes': len(rows),
        'model_mae_mean': round(sum(r['model_mae'] for r in rows) / len(rows), 4),
        'persistence_mae_mean': round(sum(r['persistence_mae'] for r in rows) / len(rows), 4),
        'delta_mae_mean': round(boot['mean'], 4),
        'delta_mae_ci95': [round(boot['ci95_low'], 4), round(boot['ci95_high'], 4)],
        'episodes_model_better': sum(r['delta_mae'] < 0 for r in rows),
        'resampling': 'whole episodes, 10000 bootstrap resamples, seed 0',
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--episodes', type=int, default=30)
    parser.add_argument('--holdout-extra', type=int, default=24,
                        help='further unseen episodes used only for per-episode errors')
    parser.add_argument('--seed', type=int, default=1000)
    parser.add_argument('--model-out', default=os.path.join(
        'src', 'smart_hub', 'models', 'demand_ridge.json'))
    parser.add_argument('--evidence-dir', default=os.path.join('evidence', 'demand'))
    opts, _ = parser.parse_known_args(args)

    seeds = [opts.seed + i for i in range(opts.episodes)]
    episodes = [generate_episode(s) for s in seeds]
    train_ids, val_ids, test_ids = split_episodes(len(episodes))
    Xtr, ytr, _ = stack(episodes, train_ids)
    Xva, yva, bva = stack(episodes, val_ids)
    Xte, yte, bte = stack(episodes, test_ids)

    # Hyper-parameter on validation only; test episodes stay unseen until the end.
    scores = {}
    for lam in LAMBDAS:
        m = RidgeModel.fit(Xtr, ytr, lam)
        scores[lam] = mae([m.predict(x) for x in Xva], yva)
    best = min(scores, key=scores.get)
    model = RidgeModel.fit(Xtr, ytr, best, model_id='ridge-l%g-e%d' % (best, opts.episodes))
    # Forecast uncertainty for decision-aware dispatch, calibrated on validation only.
    model.residual_quantiles = residual_quantiles(model, Xva, yva)

    latencies = []
    preds = []
    for x in Xte:
        start = time.perf_counter()
        preds.append(model.predict(x))
        latencies.append((time.perf_counter() - start) * 1000.0)
    feature_ms = []
    for ep in (episodes[i] for i in test_ids):
        start = time.perf_counter()
        features(ep[:HISTORY_SEC])
        feature_ms.append((time.perf_counter() - start) * 1000.0)

    report = {
        'model_id': model.model_id,
        'episodes': {'train': [seeds[i] for i in train_ids], 'validation': [
            seeds[i] for i in val_ids], 'test': [seeds[i] for i in test_ids]},
        'windows': {'train': len(ytr), 'validation': len(yva), 'test': len(yte)},
        'validation_mae_by_lambda': {str(k): round(v, 4) for k, v in scores.items()},
        'lambda': best,
        'validation_residual_quantiles': model.residual_quantiles,
        'test': {
            'model_mae': round(mae(preds, yte), 4), 'model_rmse': round(rmse(preds, yte), 4),
            'persistence_mae': round(mae(bte, yte), 4),
            'persistence_rmse': round(rmse(bte, yte), 4),
        },
        'inference_ms': {'p50': percentile(latencies, 50), 'p95': percentile(latencies, 95),
                         'feature_p95': percentile(feature_ms, 95)},
        'features': ['q', 'mean10', 'trend10', 'arrivals30 (proxy)', 'departures30 (proxy)'],
    }
    report['test']['model_no_worse_than_persistence'] = \
        report['test']['model_mae'] <= report['test']['persistence_mae']
    # Independent unit = episode. The pooled numbers above are kept for comparison only.
    per_episode = [episode_errors(model, seeds[i], episodes[i]) for i in test_ids]
    extra_seeds = [opts.seed + opts.episodes + i for i in range(opts.holdout_extra)]
    extra = [episode_errors(model, s, generate_episode(s)) for s in extra_seeds]
    report['test_per_episode'] = per_episode
    report['test_uncertainty'] = episode_uncertainty(per_episode)
    report['holdout_extra_per_episode'] = extra
    report['holdout_uncertainty'] = episode_uncertainty(per_episode + extra)
    report['note'] = (
        'Windows of one episode overlap and are not independent trials: use the '
        'per-episode rows and the episode-level bootstrap interval for uncertainty. '
        'Windows start after the %d s contiguous warm-up required online.' % WARMUP_SEC)

    os.makedirs(os.path.dirname(opts.model_out) or '.', exist_ok=True)
    model.save(opts.model_out)
    os.makedirs(opts.evidence_dir, exist_ok=True)
    with open(os.path.join(opts.evidence_dir, 'evaluation.json'), 'w') as f:
        json.dump(report, f, indent=2)
    with open(os.path.join(opts.evidence_dir, 'episodes.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['seed', 'split', 't_sec', 'queue_count'])
        split_of = {**{i: 'train' for i in train_ids}, **{i: 'validation' for i in val_ids},
                    **{i: 'test' for i in test_ids}}
        for i, ep in enumerate(episodes):
            for t, q in enumerate(ep):
                writer.writerow([seeds[i], split_of[i], t, q])
    print(json.dumps(report['test'], indent=2))
    print(json.dumps(report['holdout_uncertainty'], indent=2))
    print('lambda', best, 'inference p95 ms %.4f' % report['inference_ms']['p95'])
    print('model ->', opts.model_out, '| evidence ->', opts.evidence_dir)


if __name__ == '__main__':
    main()
