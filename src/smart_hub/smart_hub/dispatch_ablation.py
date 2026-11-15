# Author: Mikhail Li (12236295)
"""
dispatch_ablation.

Is the S2 gain of the uncertainty-aware forecast a forecast effect or only an earlier call?
Forecast + 1.66 >= 6 equals forecast >= 4.34, so the reactive rule is run with lower
observed-queue thresholds too: predictive (installed configuration) against reactive with
dispatch_high 6, 5 and 4 (dispatch_low = dispatch_high - 2, the gap of the 6 / 4 default),
on fresh seeds that no earlier matrix used (evidence/dispatch_ablation/plan.json). All
reactive thresholds are reported; none is selected on these seeds.

Runs through the test-matrix machinery (attempt IDs, single infrastructure re-run, graph
audit, ROS domain pool) and writes <out>/attempts.csv, results.csv and summary.md:

  ros2 run smart_hub dispatch_ablation --parallel 3
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import os
import statistics
import threading
import time

from smart_hub import test_matrix as tm
from smart_hub.demand_model import paired_bootstrap

SEEDS = tuple(range(201, 221))
POLICIES = (('predictive', 'predictive', None), ('reactive-6', 'reactive', 6.0),
            ('reactive-5', 'reactive', 5.0), ('reactive-4', 'reactive', 4.0))
HYSTERESIS = 2.0
DURATION = 360


def build_cases(seeds=SEEDS):
    cases = []
    for label, mode, high in POLICIES:
        for seed in seeds:
            cases.append({'test': 'A04', 'scenario': 'S2', 'variant': '', 'seed': seed,
                          'mode': mode, 'policy': label, 'duration': DURATION,
                          'check': tm.check_s2_dispatch, 'kill': None, 'record': False,
                          'dispatch_high': high,
                          'dispatch_low': None if high is None else high - HYSTERESIS})
    return cases


def compare(rows, a, b, key):
    """Paired delta a - b of `key` per seed; returns (bootstrap, seeds a lower, pairs)."""
    by = {}
    for r in rows:
        if r.get('passed') and r.get(key) not in (None, ''):
            by.setdefault(r['policy'], {})[r['seed']] = r
    seeds = sorted(set(by.get(a, {})) & set(by.get(b, {})))
    deltas = [float(by[a][s][key]) - float(by[b][s][key]) for s in seeds]
    if not deltas:
        return None, 0, []
    return paired_bootstrap(deltas), sum(d < 0 for d in deltas), \
        [(by[a][s], by[b][s]) for s in seeds]


def lead_text(pairs):
    leads = [float(rb['first_dispatch_s']) - float(ra['first_dispatch_s']) for ra, rb in pairs
             if ra.get('first_dispatch_s') not in (None, '') and
             rb.get('first_dispatch_s') not in (None, '')]
    if not leads:
        return '-'
    return '%+.1f (%d earlier, %d later by >= 1 s)' % (
        statistics.median(leads), sum(x >= 1.0 for x in leads), sum(x <= -1.0 for x in leads))


def summary_lines(rows, matrix_id, commit):
    lines = ['# Dispatch threshold ablation (S2)', '',
             'Matrix %s, commit %s. Seeds %d-%d (not used by any earlier matrix or by the '
             'choice of q). Reactive: observed queue >= dispatch_high, dispatch_low = '
             'dispatch_high - %g. Predictive: installed configuration (forecast + validation '
             'residual quantile 0.8, observed floor at 6).' % (
                 matrix_id, commit, SEEDS[0], SEEDS[-1], HYSTERESIS), '',
             '| Policy | Episodes passed | Boarded | All-passenger wait, lower bound, mean (s) '
             '| Boarded wait mean (s) | Completion |', '|---|---|---|---|---|---|']
    for label, _, _ in POLICIES:
        rs = [r for r in rows if r['policy'] == label and r.get('passed')]
        if not rs:
            continue
        lines.append('| %s | %d | %d | %.1f | %.1f | %.3f |' % (
            label, len(rs), sum(int(r['boarded']) for r in rs),
            statistics.mean(float(r['wait_all_lower_bound_mean_s']) for r in rs),
            statistics.mean(float(r['queue_wait_mean_s']) for r in rs),
            statistics.mean(float(r['completion_rate']) for r in rs)))
    lines += ['', 'Paired difference per seed, predictive - reactive (and lower thresholds '
              'against reactive-6); 95 % interval from 10000 bootstrap resamples of whole '
              'episodes. Lead = first dispatch of the second policy minus the first.', '',
              '| Comparison | Wait | Pairs | Identical arrivals | Mean delta (s) | 95 % '
              'interval (s) | First lower | Median lead (s) |',
              '|---|---|---|---|---|---|---|---|']
    comparisons = [('predictive', other) for other, _, _ in POLICIES[1:]] + \
        [(low, 'reactive-6') for low, _, _ in POLICIES[2:]]
    for a, b in comparisons:
        for name, key in (('all passengers', 'wait_all_lower_bound_mean_s'),
                          ('boarded', 'queue_wait_mean_s')):
            boot, lower, pairs = compare(rows, a, b, key)
            if boot is None:
                continue
            same = sum(tm.same_arrivals(ra.get('arrival_times'), rb.get('arrival_times'))
                       for ra, rb in pairs)
            lines.append('| %s vs %s | %s | %d | %d | %+.1f | %+.1f .. %+.1f | %d | %s |' % (
                a, b, name, boot['n'], same, boot['mean'], boot['ci95_low'],
                boot['ci95_high'], lower, lead_text(pairs) if name == 'boarded' else ''))
    return lines


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', default=os.path.join('evidence', 'dispatch_ablation'))
    parser.add_argument('--runs-dir', default=os.path.join('evidence', 'runs'))
    parser.add_argument('--parallel', type=int, default=3)
    parser.add_argument('--domain-base', type=int, default=30)
    parser.add_argument('--ui-port-base', type=int, default=9000)
    parser.add_argument('--tag-base', type=int, default=int(time.time()) % 900 * 100)
    parser.add_argument('--commit', default='')
    opts, _ = parser.parse_known_args(args)
    opts.commit, opts.dirty = tm.source_commit(opts.commit)
    opts.commit_label = opts.commit + ('+dirty' if opts.dirty else '')
    opts.matrix_id = time.strftime('A%Y%m%d-%H%M%S', time.gmtime())
    os.makedirs(os.path.join(opts.out, 'logs'), exist_ok=True)
    opts.domains = tm.DomainPool(opts.domain_base)
    cases = build_cases()
    lock = threading.Lock()
    with ThreadPoolExecutor(max_workers=opts.parallel) as pool:
        futures = [pool.submit(tm.run_case, c, i, opts, lock) for i, c in enumerate(cases)]
        rows = [dict(f.result(), policy=c['policy']) for f, c in zip(futures, cases)]
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(opts.out, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(opts.out, 'summary.md'), 'w') as f:
        f.write('\n'.join(summary_lines(rows, opts.matrix_id, opts.commit_label)) + '\n')
    print('\n'.join(summary_lines(rows, opts.matrix_id, opts.commit_label)))


if __name__ == '__main__':
    main()
