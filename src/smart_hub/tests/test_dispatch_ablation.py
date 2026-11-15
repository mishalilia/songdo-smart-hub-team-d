# Author: Mikhail Li (12236295)
"""Threshold ablation of the S2 dispatch: cases and paired comparison."""
from smart_hub import dispatch_ablation as da


def test_cases_cover_every_policy_on_fresh_seeds_with_the_default_hysteresis():
    cases = da.build_cases()
    assert len(cases) == 4 * 20 and min(c['seed'] for c in cases) == 201
    reactive = {c['dispatch_high']: c['dispatch_low'] for c in cases if c['mode'] == 'reactive'}
    assert reactive == {6.0: 4.0, 5.0: 3.0, 4.0: 2.0}
    assert all(c['dispatch_high'] is None for c in cases if c['mode'] == 'predictive')


def test_summary_pairs_policies_by_seed():
    def row(policy, seed, wait, first):
        return {'policy': policy, 'seed': seed, 'passed': True, 'boarded': 12,
                'wait_all_lower_bound_mean_s': wait, 'queue_wait_mean_s': wait,
                'completion_rate': 0.5, 'first_dispatch_s': first,
                'arrival_times': '10.00;20.00'}
    rows = [row('predictive', s, 100.0, 80.0) for s in (201, 202)] + \
        [row('reactive-6', s, 110.0, 100.0) for s in (201, 202)] + \
        [row('reactive-4', s, 99.0, 75.0) for s in (201, 202)]
    boot, lower, pairs = da.compare(rows, 'predictive', 'reactive-6', 'queue_wait_mean_s')
    assert boot['mean'] == -10.0 and lower == 2 and len(pairs) == 2
    text = '\n'.join(da.summary_lines(rows, 'A1', 'abc'))
    assert '| predictive vs reactive-6 | all passengers | 2 | 2 | -10.0 |' in text
    assert '| predictive vs reactive-4 | boarded | 2 | 2 | +1.0 |' in text
    assert '+20.0 (2 earlier, 0 later by >= 1 s)' in text
