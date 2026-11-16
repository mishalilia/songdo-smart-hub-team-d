# Author: Hsu Shwe Sin Htun (12240198)
"""
Train and evaluate the S2 demand model (assignment 9.2, 9.4).

1. Generate independent six-minute episodes in the simulator (sim_world, no ROS): varied
   arrival rates per minute and boarding events (up to 4 passengers leave).
2. Split whole episodes 60/20/20 into train / validation / test before windowing.
3. Fit normalisation and ridge regression on training episodes, choose lambda on
   validation, report held-out MAE / RMSE against the persistence baseline and the
   inference latency.

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
    features, HISTORY_SEC, mae, RidgeModel, rmse, split_episodes, windows,
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


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--episodes', type=int, default=30)
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
    print('lambda', best, 'inference p95 ms %.4f' % report['inference_ms']['p95'])
    print('model ->', opts.model_out, '| evidence ->', opts.evidence_dir)


if __name__ == '__main__':
    main()
