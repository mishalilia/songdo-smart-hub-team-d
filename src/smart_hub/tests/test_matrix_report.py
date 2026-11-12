# Author: Mikhail Li (12236295)
"""Attempt bookkeeping, graph audit check and summary sections of the test matrix."""
import argparse
import csv
import os
import threading

from smart_hub import test_matrix
from smart_hub.test_matrix import (
    dispatch_section, GRAPH_NODES, graph_problems, GRAPH_TOPICS, standstill, write_summary,
)


def audit():
    return {'graph': {'t_sec': 2.0, 'nodes': list(GRAPH_NODES), 'topics': {
        topic: {'publishers': ['%s %s %s' % (node, reliability, durability)],
                'subscribers': ['metrics_logger'], 'rate_hz': hz}
        for topic, (node, reliability, durability, hz) in GRAPH_TOPICS.items()}}}


def case(test='T16', seed=42):
    return {'test': test, 'scenario': 'FAULTS', 'variant': 'estop_waiting', 'seed': seed,
            'mode': 'predictive', 'kill': None, 'duration': 1, 'record': False}


def options(tmp_path):
    runs = tmp_path / 'runs'
    for run_id in ('run-a', 'run-b'):
        (runs / run_id).mkdir(parents=True, exist_ok=True)
        (runs / run_id / 'events.csv').write_text('t_sec,entity_id,event\n')
        (runs / run_id / 'energy.csv').write_text('t_sec\n')
    return argparse.Namespace(out=str(tmp_path), runs_dir=str(runs), matrix_id='M-TEST',
                              commit_label='abc123')


def attempts(tmp_path):
    with open(os.path.join(str(tmp_path), 'attempts.csv')) as f:
        return list(csv.DictReader(f))


def test_graph_audit_matches_the_interface_table():
    assert graph_problems(audit()) == []
    assert graph_problems({}) == ['no graph audit in the run record']


def test_graph_audit_reports_missing_node_wrong_qos_and_rate():
    summary = audit()
    graph = summary['graph']
    graph['nodes'].remove('route_planner')
    graph['topics']['/hub/safety']['publishers'] = ['safety_supervisor RELIABLE VOLATILE']
    graph['topics']['/hub/queue']['rate_hz'] = 2.4
    graph['topics']['/hub/vehicles']['publishers'].append('hub_ui RELIABLE VOLATILE')
    problems = graph_problems(summary)
    assert len(problems) == 4
    assert problems[0] == 'node route_planner not discovered'
    assert any(p.startswith('/hub/safety published by') for p in problems)
    assert any(p.startswith('/hub/vehicles published by') for p in problems)
    assert '/hub/queue at 2.40 Hz, nominal 5 Hz' in problems
    # A node the case kills on purpose is not an audit failure.
    assert len(graph_problems(summary, exempt=('route_planner',))) == 3


def test_on_change_topics_have_no_maximum_rate():
    summary = audit()
    summary['graph']['topics']['/parking/state']['rate_hz'] = 6.0     # 2 Hz plus changes
    assert graph_problems(summary) == []
    summary['graph']['topics']['/parking/state']['rate_hz'] = 1.5
    summary['graph']['topics']['/robot/odom']['rate_hz'] = 26.0
    assert graph_problems(summary) == ['/parking/state at 1.50 Hz, nominal 2 Hz',
                                       '/robot/odom at 26.00 Hz, nominal 20 Hz']


def test_run_stays_up_for_the_observation_window_after_a_late_kill():
    nominal = 100.0 + test_matrix.STARTUP_SEC + 17.0
    assert test_matrix.run_end(100.0, 17.0) == nominal
    assert test_matrix.run_end(100.0, 17.0, kill_wall=110.0) == nominal
    # slow start-up: the kill lands one second before the nominal end
    assert test_matrix.run_end(100.0, 17.0, kill_wall=nominal - 1.0) == \
        nominal - 1.0 + test_matrix.POST_KILL_SEC


def test_standstill_reads_latency_and_observer_delay():
    ev = [{'t': 10.0, 'entity_id': 'robot1', 'event': 'STANDSTILL_MEASURED',
           'reason': 'cause=ESTOP latency_ms=412.50 zero_cmd_ms=3.10 observer_delay_ms=1.25'}]
    assert standstill(ev, 9.0) == (412.5, 1.25)
    assert standstill(ev, 11.0) == (None, None)


def test_infrastructure_failure_is_retried_once_and_both_attempts_are_kept(
        tmp_path, monkeypatch):
    outcomes = iter([
        {'passed': False, 'failure_class': 'infrastructure', 'run_id': 'run-a',
         'details': 'startup gate incomplete, missing=bus_manager'},
        {'passed': True, 'failure_class': '', 'run_id': 'run-b', 'details': 'ok'}])
    monkeypatch.setattr(
        test_matrix, '_run_case',
        lambda c, index, opts, lock, attempt: dict(test_matrix.case_row(c), **next(outcomes)))
    row = test_matrix.run_case(case(), 7, options(tmp_path), threading.Lock())
    assert (row['passed'], row['attempts'], row['first_attempt']) == (
        True, 2, 'infrastructure')
    kept = attempts(tmp_path)
    assert [a['attempt_id'] for a in kept] == ['M-TEST-007-1', 'M-TEST-007-2']
    assert [a['run_id'] for a in kept] == ['run-a', 'run-b']
    assert kept[0]['superseded_by'] == 'M-TEST-007-2' and kept[1]['superseded_by'] == ''
    assert kept[0]['failure_class'] == 'infrastructure' and kept[0]['passed'] == 'False'
    assert {a['harness_version'] for a in kept} == {test_matrix.HARNESS_VERSION}
    assert {a['commit'] for a in kept} == {'abc123'}
    # The records of both attempts of a failed case are archived with the results.
    for attempt_id in ('M-TEST-007-1', 'M-TEST-007-2'):
        assert os.listdir(os.path.join(str(tmp_path), 'records', attempt_id)) == ['events.csv']


def test_application_failure_is_final(tmp_path, monkeypatch):
    calls = []

    def run(c, index, opts, lock, attempt):
        calls.append(attempt)
        return dict(test_matrix.case_row(c), passed=False, failure_class='application',
                    run_id='run-a', details='zero command 250.00 ms')
    monkeypatch.setattr(test_matrix, '_run_case', run)
    row = test_matrix.run_case(case(), 3, options(tmp_path), threading.Lock())
    assert calls == [1]
    assert (row['passed'], row['attempts'], row['first_attempt']) == (
        False, 1, 'application')
    assert attempts(tmp_path)[0]['superseded_by'] == ''


def test_runner_exception_is_an_infrastructure_failure(tmp_path, monkeypatch):
    def run(c, index, opts, lock, attempt):
        raise OSError('ros2 not found')
    monkeypatch.setattr(test_matrix, '_run_case', run)
    row = test_matrix.run_case(case(), 1, options(tmp_path), threading.Lock())
    assert row['failure_class'] == 'infrastructure' and row['attempts'] == 2
    assert len(attempts(tmp_path)) == 2


def s8(mode, seed, wait, lower):
    return {'test': 'T15', 'scenario': 'S8', 'variant': '', 'seed': seed, 'mode': mode,
            'passed': True, 'first_attempt': 'pass', 'attempts': 1, 'details': 'ok',
            'matrix_id': 'M-TEST', 'commit': 'abc123', 'failure_class': '',
            'queue_wait_mean_s': wait, 'queue_wait_p50_s': wait, 'queue_wait_p95_s': 2 * wait,
            'queue_wait_n': 10, 'wait_all_lower_bound_mean_s': lower,
            'wait_all_lower_bound_p95_s': 2 * lower, 'arrived': 12, 'boarded': 10,
            'abandoned': 1, 'waiting_at_end': 1, 'completion_rate': 0.833}


def test_dispatch_section_pairs_episodes_by_seed():
    rows = [s8('predictive', 1, 40.0, 50.0), s8('reactive', 1, 60.0, 65.0),
            s8('predictive', 2, 50.0, 55.0), s8('reactive', 2, 45.0, 60.0),
            s8('reactive', 3, 70.0, 80.0)]          # no predictive partner: not paired
    text = '\n'.join(dispatch_section(rows))
    assert '| predictive | 2 | 20 | 45.0 / 45.0 / 90.0 | 52.5 / 105.0 | 24 | 2 | 2 | 0.833 |' \
        in text
    assert '| reactive | 3 | 30 |' in text
    assert '| boarded only | 2 | -7.5 |' in text and '| 1 of 2 |' in text
    assert '| all passengers (lower bound) | 2 | -10.0 |' in text and '| 2 of 2 |' in text
    assert 'seed 1 -20.0, seed 2 +5.0' in text
    assert dispatch_section([s8('reactive', 1, 60.0, 65.0)]) == []
    s2 = [dict(r, scenario='S2') for r in rows]
    both = '\n'.join(dispatch_section(s2 + rows))
    assert both.index('(S2, rising demand') < both.index('(S8, step surge')


def test_summary_separates_first_attempt_from_eventual_success(tmp_path):
    opts = options(tmp_path)
    lock = threading.Lock()
    base = dict(test_matrix.case_row(case()), harness_version='2.0', matrix_id='M-TEST',
                commit='abc123')
    test_matrix.append_attempt(opts, lock, dict(
        base, attempt_id='M-TEST-000-1', attempt=1, passed=False, run_id='run-a',
        failure_class='infrastructure', superseded_by='M-TEST-000-2',
        details='startup gate incomplete, missing=bus_manager'))
    test_matrix.append_attempt(opts, lock, dict(
        base, attempt_id='M-TEST-000-2', attempt=2, passed=True, run_id='run-b', details='ok',
        stop_latency_ms=3.2))                  # metrics are not attempt columns
    rows = [dict(base, passed=True, run_id='run-b', details='ok', attempts=2,
                 first_attempt='infrastructure', stop_latency_ms=3.2, standstill_ms=410.0,
                 observer_delay_ms=1.5),
            dict(base, seed=43, passed=False, run_id='run-c', details='zero command 250 ms',
                 failure_class='application', attempts=1, first_attempt='application'),
            dict(base, seed=44, passed=True, run_id='run-d', details='ok', attempts=1,
                 first_attempt='pass')]
    write_summary(rows, opts)
    with open(os.path.join(str(tmp_path), 'summary.md')) as f:
        text = f.read()
    assert '- Passed on the first attempt: 1/3.' in text
    assert '(run could not be judged): 1; for application reasons' in text
    assert '(eventual success): 2/3.' in text
    assert '| T16 | FAULTS | estop_waiting | predictive | 3 | 1/3 | 2/3 |' in text
    assert '| 2.0 | M-TEST | abc123 | 2 | 1 | 1 | 0 | 0 | 1 |' in text
    assert '| M-TEST-000-1 | T16 | FAULTS / estop_waiting | 42 | infrastructure | run-a | ' \
        'M-TEST-000-2 | startup gate incomplete, missing=bus_manager |' in text
    assert 'Zero command after E-stop receipt (STOP_LATCHED) | 1 | 3.20 / 3.20 / 3.20 ms' \
        in text
    with open(os.path.join(str(tmp_path), 'results.csv')) as f:
        assert [r['first_attempt'] for r in csv.DictReader(f)] == [
            'infrastructure', 'application', 'pass']
