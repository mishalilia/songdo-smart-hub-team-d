# Author: Mikhail Li (12236295)
"""
Verification matrix runner, T01-T18 (assignment 21).

Launches the full hub for every case (scenario, variant, seed, dispatch mode), optionally
kills a node during the run (T17 / T18), then checks the assignment pass criteria on the
recorded run (evidence/runs/<run_id>/events.csv + summary.json). Writes
<out>/results.csv and <out>/summary.md.

  ros2 run smart_hub run_test_matrix --parallel 5 --out evidence/test_matrix
  ros2 run smart_hub run_test_matrix --cases T13 --repeat 3     # subset
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import glob
import inspect
import json
import os
import re
import signal
import subprocess
import threading
import time

from smart_hub.metrics import parse_latency_ms, percentile

STARTUP_SEC = 8.0          # launch -> scenario clock start (services up)


# ---------------------------------------------------------------- run helpers
def load_events(run_dir):
    with open(os.path.join(run_dir, 'events.csv')) as f:
        return [dict(r, t=float(r['t_sec'])) for r in csv.DictReader(f)]


def first(events, event, entity=None, after=-1.0, reason_has=''):
    for e in events:
        if e['event'] == event and e['t'] > after and reason_has in e['reason'] and \
                (entity is None or e['entity_id'] == entity):
            return e
    return None


def all_of(events, event, entity=None):
    return [e for e in events if e['event'] == event and (entity is None or
                                                          e['entity_id'] == entity)]


def injected(events, kind):
    return first(events, 'INJECT', 'scenario_runner', reason_has=kind)


def number(text, key):
    m = re.search(key + r'=(-?[0-9.]+)', text)
    return float(m.group(1)) if m else None


def safety_ok(summary):
    c = summary['counts']
    return c['collisions'] == 0 and c['crossing_conflicts'] == 0


# ---------------------------------------------------------------- checks
def check_s1(ev, summary):
    counts, stable = [], []
    changes = [e for e in ev if e['event'] in ('QUEUE_COUNT', 'QUEUE_UNKNOWN')]
    for e, nxt in zip(changes, changes[1:] + [None]):
        held = (nxt['t'] if nxt else 1e9) - e['t']
        if e['event'] == 'QUEUE_COUNT' and held >= 1.0:
            c = int(number(e['reason'], 'count'))
            if not stable or stable[-1] != c:
                stable.append(c)
        counts.append(e['reason'])
    freeze = injected(ev, 'freeze_camera')
    unknown = first(ev, 'QUEUE_UNKNOWN', after=freeze['t']) if freeze else None
    delay = unknown['t'] - freeze['t'] if unknown else None
    ok = stable[:4] == [0, 3, 5, 4] and delay is not None and delay <= 1.2
    return ok, 'stable counts %s; invalid %.2fs after freeze' % (
        stable, delay if delay is not None else -1), {'freeze_to_unknown_s': delay}


def check_s2_dispatch(ev, summary):
    outstanding, overlaps = 0, 0
    reasons = []
    for e in ev:
        if e['event'] == 'DISPATCH_REQUESTED':
            overlaps += outstanding > 0
            outstanding += 1
            reasons.append(e['reason'].split()[0])
        elif e['event'] == 'DISPATCH_RESULT':
            outstanding = max(0, outstanding - 1)
    ok = reasons and overlaps == 0 and not all_of(ev, 'DISPATCH_REJECTED') and safety_ok(summary)
    return ok, 'dispatches %s, overlapping %d' % (reasons, overlaps), {'dispatches': len(reasons)}


def check_dock_ok(ev, summary, eta_required=True):
    docked = first(ev, 'DOCKED', 'bus1')
    docking = first(ev, 'DOCKING', 'bus1')
    pos = number(docked['reason'], 'pos_err_m') if docked else None
    yaw = number(docked['reason'], 'yaw_err_rad') if docked else None
    eta = number(docking['reason'], 'eta_err_s') if docking else None
    invalid = number(docking['reason'], 'invalid_eta_s') if docking else None
    door = all_of(ev, 'DOOR_OPEN_MOVING')
    eta_ok = not eta_required or (eta is not None and abs(eta) <= 3.0)
    ok = bool(first(ev, 'GOAL_SUCCEED', 'bus1')) and pos is not None and pos <= 0.20 and \
        abs(yaw) <= 0.17 and not door and eta_ok and safety_ok(summary)
    return ok, 'pos_err %.3f m, yaw_err %.3f rad, ETA err at 5 m %s s (invalid %s s), ' \
        'door-moving %d, boarded %d' % (
            pos if pos is not None else -1, yaw if yaw is not None else -1,
            '%.2f' % eta if eta is not None else '-', invalid, len(door),
            summary['counts']['boarded']), {'pos_err_m': pos, 'yaw_err_rad': yaw,
                                            'eta_err_s': eta, 'eta_invalid_s': invalid}


def check_dock_disturbed(ev, summary):
    # A pedestrian or blocked bay makes the bus wait, so the 5 m ETA is not a target here.
    return check_dock_ok(ev, summary, eta_required=False)


def check_dock_cancel(ev, summary):
    c = first(ev, 'GOAL_CANCELED', 'bus1')
    lat = parse_latency_ms(c['reason']) if c else None
    ok = lat is not None and lat <= 500.0 and safety_ok(summary)
    return ok, 'cancel accepted -> zero %.0f ms' % (lat or -1), {'cancel_ms': lat}


def check_dock_stale(ev, summary):
    a = first(ev, 'GOAL_ABORT', 'bus1')
    ok = bool(a) and 'STALE_BUS_POSE' in a['reason'] and safety_ok(summary)
    return ok, 'abort reason %s' % (a['reason'] if a else 'none'), {}


def check_pairs(ev, summary):
    # Grants as reported to vehicles (not the slots' own state transitions).
    oks = [e for e in ev if e['event'] == 'SLOT_RESERVED' and 'ttl=' in e['reason']]
    by_time = {}
    for e in oks:
        by_time.setdefault(round(e['t']), []).append(e['reason'].split()[0])
    pairs = list(by_time.values())
    doubles = sum(1 for v in pairs if len(v) != len(set(v)))
    lat = [parse_latency_ms(e['reason']) for e in ev if e['event'] in (
        'RESERVE_OK', 'RESERVE_REJECTED') and parse_latency_ms(e['reason']) is not None]
    p95 = percentile(lat, 95) if lat else None
    ok = len(oks) == 40 and doubles == 0 and p95 is not None and p95 < 250.0
    return ok, '%d grants in %d pairs, %d double allocations, service p95 %.1f ms (n=%d)' % (
        len(oks), len(pairs), doubles, p95 or -1, len(lat)), {'service_p95_ms': p95}


def check_s4(ev, summary):
    first_res = first(ev, 'SLOT_RESERVED', 'C1')
    dup = first(ev, 'RESERVE_DUPLICATE', 'C1')
    expired = first(ev, 'RESERVATION_EXPIRED')
    freed_after = [e for e in ev if e['entity_id'] == 'P2' and e['event'] == 'SLOT_FREE' and
                   expired and e['t'] >= expired['t']]
    disc = injected(ev, 'disconnect_slot_sensor')
    p3_events = [e for e in ev if e['entity_id'] == 'P3' and e['event'].startswith('SLOT_')]
    # P3 must be UNKNOWN within 1.5 s of the disconnect (already UNKNOWN also satisfies it).
    p3_state = [e for e in p3_events if disc and e['t'] <= disc['t'] + 1.5]
    p3 = p3_state[-1] if p3_state and p3_state[-1]['event'] == 'SLOT_UNKNOWN' else None
    checks = {
        'C2 rejected': bool(first(ev, 'RESERVATION_REJECTED', 'C2')),
        'duplicate same token': bool(dup and first_res and
                                     dup['correlation_id'] == first_res['correlation_id']),
        'wrong token rejected': bool(first(ev, 'RELEASE_REJECTED', 'C1')),
        'barrier for owner': bool(first(ev, 'BARRIER_GRANTED', 'C1')),
        'car parked P2': bool(first(ev, 'CAR_PARKED', 'C1', reason_has='P2')),
        'barrier denied C3': bool(first(ev, 'BARRIER_DENIED', 'C3')),
        'expiry keeps occupied': bool(expired) and not freed_after,
        'stale sensor UNKNOWN<=1.5s': bool(p3) and not any(
            e['event'] == 'SLOT_FREE' for e in p3_events if e['t'] > disc['t']),
    }
    failed = [k for k, v in checks.items() if not v]
    return not failed, 'all checks passed' if not failed else 'failed: ' + ', '.join(failed), {}


def check_s5(ev, summary):
    def delay(kind, label, edge='Cs-J'):
        inj = injected(ev, kind)
        hit = first(ev, label, edge, after=inj['t'] if inj else 1e9)
        return hit['t'] - inj['t'] if inj and hit else None
    congested = delay('spawn_slow_vehicles', 'EDGE_CONGESTED')
    unknown = delay('freeze_vehicle_source', 'EDGE_UNKNOWN')
    blocked = delay('place_obstacle', 'EDGE_BLOCKED')
    precision, recall = congestion_scores(ev)
    ok = None not in (congested, unknown, blocked) and congested <= 2.0 + 1.5 and \
        unknown <= 1.2 and blocked <= 0.6
    fmt = lambda v: '%.2f' % v if v is not None else 'none'   # noqa: E731
    details = 'CONGESTED %ss, UNKNOWN %ss, BLOCKED %ss after stimulus; ' \
        'congestion precision %s recall %s' % (
            fmt(congested), fmt(unknown), fmt(blocked), fmt(precision), fmt(recall))
    metrics = {'congested_s': congested, 'unknown_s': unknown, 'blocked_s': blocked,
               'precision': precision, 'recall': recall}
    return ok, details, metrics


def congestion_scores(ev, edge='Cs-J', step=0.5):
    """
    Precision / recall of CONGESTED on `edge` against the scenario labels (12.3).

    Truth: congested from the slow-vehicle injection until the obstacle (then BLOCKED).
    Predictions are the published labels, sampled every `step` seconds; UNKNOWN samples
    are excluded (no claim was made).
    """
    slow, obstacle = injected(ev, 'spawn_slow_vehicles'), injected(ev, 'place_obstacle')
    if not slow or not obstacle:
        return None, None
    labels = [(e['t'], e['event'][5:]) for e in ev
              if e['entity_id'] == edge and e['event'].startswith('EDGE_')]
    tp = fp = fn = 0
    t = 1.0
    while t < obstacle['t']:
        current = [lab for when, lab in labels if when <= t]
        pred = current[-1] if current else 'UNKNOWN'
        truth = slow['t'] + 2.0 <= t      # 2 s allowance after the stimulus (12.3)
        if pred != 'UNKNOWN' and (truth or slow['t'] > t):
            if pred == 'CONGESTED' and truth:
                tp += 1
            elif pred == 'CONGESTED':
                fp += 1
            elif truth:
                fn += 1
        t += step
    precision = tp / (tp + fp) if tp + fp else None
    recall = tp / (tp + fn) if tp + fn else None
    return precision, recall


def check_free_flow(ev, summary):
    false = [e for e in ev if e['event'] in ('EDGE_CONGESTED', 'EDGE_BLOCKED')]
    ok = not false and summary['counts']['collisions'] == 0
    return ok, '%d false congestion / blockage labels in free flow' % len(false), {
        'false_alarms': len(false)}


def check_estop(ev, summary):
    latch = first(ev, 'STOP_LATCHED', reason_has='ESTOP')
    lat = parse_latency_ms(latch['reason']) if latch else None
    abort = first(ev, 'GOAL_ABORT', 'robot1', after=latch['t'] - 0.1 if latch else 1e9)
    restart = first(ev, 'GOAL_ACCEPTED', 'robot1', after=latch['t'] if latch else 1e9)
    ok = lat is not None and lat <= 200.0 and bool(abort) and not restart and \
        safety_ok(summary)
    return ok, 'stop latency %.2f ms, mission %s, no restart: %s' % (
        lat or -1, abort['reason'] if abort else 'not aborted', not restart), {
        'stop_latency_ms': lat}


def check_escort_ok(ev, summary):
    arrived = first(ev, 'ARRIVED', 'robot1')
    err = number(arrived['reason'], 'error_m') if arrived else None
    ok = bool(first(ev, 'GOAL_SUCCEED', 'robot1')) and err is not None and err <= 0.20 and \
        safety_ok(summary)
    return ok, 'arrival error %.3f m, replans %d' % (
        err if err is not None else -1, summary['counts']['reroutes']), {'arrival_err_m': err}


def check_goal_outcome(event, reason, extra=None):
    def check(ev, summary):
        hit = first(ev, event, 'robot1', reason_has=reason)
        ok = bool(hit) and safety_ok(summary) and (extra is None or extra(ev))
        return ok, '%s %s' % (event, hit['reason'] if hit else 'missing'), {}
    return check


def check_escort_cancel(ev, summary):
    c = first(ev, 'GOAL_CANCELED', 'robot1')
    lat = parse_latency_ms(c['reason']) if c else None
    ok = lat is not None and lat <= 500.0 and safety_ok(summary)
    return ok, 'cancel accepted -> zero %.0f ms' % (lat or -1), {'cancel_ms': lat}


def check_s8(ev, summary):
    outstanding, overlaps = 0, 0
    for e in ev:
        if e['event'] == 'DISPATCH_REQUESTED':
            overlaps += outstanding > 0
            outstanding += 1
        elif e['event'] == 'DISPATCH_RESULT':
            outstanding -= 1
    slots = [e['reason'].split()[0] for e in all_of(ev, 'SLOT_RESERVED') if 'ttl=' in e['reason']]
    wait = summary['queue_wait_sec']
    ok = safety_ok(summary) and overlaps == 0 and len(slots) == len(set(slots)) and \
        bool(first(ev, 'GOAL_SUCCEED', 'robot1'))
    return ok, 'collisions %d, conflicts %d, overlapping dispatch %d, queue wait mean %s s' % (
        summary['counts']['collisions'], summary['counts']['crossing_conflicts'], overlaps,
        '%.1f' % wait['mean'] if wait.get('n') else '-'), {
        'queue_wait_mean_s': wait.get('mean'), 'boarded': summary['counts']['boarded']}


def check_s9(ev, summary):
    latch = first(ev, 'STOP_LATCHED', reason_has='ESTOP')
    lat = parse_latency_ms(latch['reason']) if latch else None
    rejected = first(ev, 'CLEAR_STOP_REJECTED', after=latch['t'] if latch else 0)
    cleared = first(ev, 'STOP_CLEARED', after=rejected['t'] if rejected else 1e9)
    restart = first(ev, 'GOAL_ACCEPTED', 'robot1', after=latch['t'] if latch else 1e9)
    ok = lat is not None and lat <= 200 and bool(rejected) and bool(cleared) and \
        not restart and bool(first(ev, 'GOAL_ABORT', 'robot1', reason_has='SAFE_STOP'))
    return ok, 'stop latency %.2f ms, reset rejected while stale: %s, then accepted: %s' % (
        lat or -1, bool(rejected), bool(cleared)), {'stop_latency_ms': lat}


def check_after_kill(event, entity, within):
    def check(ev, summary, kill_t=None):
        hit = first(ev, event, entity, after=(kill_t or 0) - 0.5)
        dt = hit['t'] - kill_t if hit and kill_t is not None else None
        ok = dt is not None and dt <= within and summary['counts']['collisions'] == 0
        return ok, '%s %.2fs after process kill' % (event, dt if dt is not None else -1), {
            'reaction_s': dt}
    return check


def check_camera_silence(ev, summary):
    unknown = first(ev, 'QUEUE_UNKNOWN')
    ok = bool(unknown) and safety_ok(summary) and bool(first(ev, 'GOAL_SUCCEED', 'robot1'))
    return ok, 'queue UNKNOWN reported; mission completed without crossing conflict', {}


def check_parking_stale(ev, summary):
    ok = bool(first(ev, 'SLOT_UNKNOWN', 'P2'))
    return ok, 'P2 becomes UNKNOWN when its sensor is stale', {}


def check_s6(ev, summary):
    labels = [(e['entity_id'], e['event']) for e in ev if e['event'].startswith('EDGE_')]
    ok = ('Cs-J', 'EDGE_BLOCKED') in labels and ('B-Cn', 'EDGE_BLOCKED') in labels
    return ok, 'Cs-J and B-Cn reported BLOCKED; planner oracle in unit tests', {}


# ---------------------------------------------------------------- cases
def build_cases(repeat):
    cases = []

    def add(test, scenario, check, duration, variant='', seeds=(42,), mode='predictive',
            kill=None, record=False):
        for i, seed in enumerate(seeds):
            cases.append({'test': test, 'scenario': scenario, 'variant': variant, 'seed': seed,
                          'mode': mode, 'duration': duration, 'check': check, 'kill': kill,
                          'record': record and i == 0})
    seeds = tuple(range(1, repeat + 1))
    add('T01/T02', 'S1', check_s1, 45)
    add('T04', 'S2', check_s2_dispatch, 200)
    add('T05', 'S3', check_dock_ok, 75, seeds=seeds, record=True)
    add('T05', 'S3', check_dock_disturbed, 85, variant='pedestrian_before_entry')
    add('T05', 'S3', check_dock_disturbed, 100, variant='blocked_bay')
    add('T05', 'S3', check_dock_stale, 40, variant='stale_bus_pose')
    add('T06', 'S3', check_dock_cancel, 30, variant='cancel_during_approach')
    add('T07', 'S4_PAIRS', check_pairs, 130)
    add('T08', 'S4', check_s4, 45, record=True)
    add('T09/T10', 'S5', check_s5, 80, record=True)
    add('T09', 'S5_FREE', check_free_flow, 60, seeds=seeds)
    add('T11/T12', 'S6', check_s6, 75)
    add('T13', 'S7', check_escort_ok, 120, seeds=seeds, record=True)
    add('T14', 'S7', check_goal_outcome('GOAL_ABORT', 'READY_TIMEOUT'), 40, variant='no_ready')
    add('T14', 'S7', check_escort_cancel, 30, variant='cancel_while_moving')
    add('T14', 'S7', check_goal_outcome(
        'GOAL_REJECTED', 'BUSY', lambda ev: bool(first(ev, 'GOAL_SUCCEED', 'robot1'))),
        120, variant='second_goal_while_busy')
    add('T14', 'S7', check_goal_outcome('GOAL_REJECTED', 'INVALID_NODE'), 15,
        variant='unreachable_destination')
    add('T14', 'S7', check_goal_outcome('GOAL_ABORT', 'ODOM_STALE'), 30,
        variant='odometry_freeze')
    add('T14', 'S7', check_escort_ok, 120, variant='obstacle_before_crossing')
    add('T14', 'S7', check_goal_outcome(
        'GOAL_SUCCEED', 'ARRIVED', lambda ev: bool(first(ev, 'REPLAN_CONGESTION', 'robot1'))),
        120, variant='congestion')
    add('T15', 'S8', check_s8, 300, seeds=seeds, mode='predictive', record=True)
    add('T15', 'S8', check_s8, 300, seeds=seeds, mode='reactive')
    add('T16', 'S9', check_s9, 45, record=True)
    # Each fault five times at different mission phases (16.3).
    phases = ('waiting', 'to_crossing', 'in_crossing', 'after_crossing', 'near_destination')
    for phase in phases:
        add('T16', 'FAULTS', check_estop, 60, variant='estop_' + phase)
        add('T16', 'FAULTS', check_goal_outcome('GOAL_ABORT', 'ODOM_STALE'), 60,
            variant='odometry_gap_' + phase)
        add('T16', 'FAULTS', check_camera_silence, 120, variant='camera_silence_' + phase)
    add('T16', 'FAULTS', check_parking_stale, 15, variant='parking_sensor_stale')
    for delay in (2.0, 10.0, 18.0, 28.0, 40.0):
        add('T17', 'FAULTS', check_after_kill('SAFETY_INHIBIT', 'robot1', 1.0), delay + 15,
            kill=(('ROUTE', 'robot1', 'ESCORT', delay), 'robot_manager'))
        add('T17', 'FAULTS', check_after_kill('CMD_TIMEOUT', 'robot1', 1.0), delay + 15,
            kill=(('ROUTE', 'robot1', 'ESCORT', delay), 'safety_supervisor'))
    add('T18', 'FAULTS', check_goal_outcome('GOAL_ABORT', 'PLANNER_UNAVAILABLE'), 30,
        kill=(('SCENARIO_STARTED', 'scenario_runner', '', 0.0), 'route_planner'))
    return cases


# ---------------------------------------------------------------- execution
def newest_run(runs_dir, label, seed, since):
    pattern = os.path.join(runs_dir, '%s-s%d-*' % (label, seed))
    dirs = [d for d in glob.glob(pattern) if os.path.getmtime(d) >= since - 1]
    return max(dirs, key=os.path.getmtime) if dirs else None


def wait_for_event(runs_dir, label, seed, since, event, entity, reason_has, timeout):
    """Poll the run's events.csv until an event appears; return its run time or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        run_dir = newest_run(runs_dir, label, seed, since)
        if run_dir and os.path.exists(os.path.join(run_dir, 'events.csv')):
            hit = first(load_events(run_dir), event, entity, reason_has=reason_has)
            if hit:
                return hit['t']
        time.sleep(0.2)
    return None


def run_case(case, index, opts, lock):
    try:
        row = _run_case(case, index, opts, lock)
        if row.get('startup_missing'):
            # The ROS graph did not come up (DDS discovery under load): an infrastructure
            # failure, not a test result. Run the case once more and say so.
            note = row['startup_missing']
            row = _run_case(case, index, opts, lock, retry=True)
            row['details'] = '%s (rerun: first attempt started without %s)' % (
                row['details'], note)
        row.pop('startup_missing', None)
        return row
    except Exception as exc:              # one broken run must not stop the matrix
        with lock:
            print('%-8s %-9s ERROR %r' % (case['test'], case['scenario'], exc), flush=True)
        return {'test': case['test'], 'scenario': case['scenario'],
                'variant': case['variant'], 'seed': case['seed'], 'mode': case['mode'],
                'run_id': '', 'passed': False, 'details': 'runner error: %r' % exc}


def _run_case(case, index, opts, lock, retry=False):
    domain = opts.domain_base + index % 60           # valid ROS_DOMAIN_ID range is 0..101
    tag = 'm%05d' % (opts.tag_base + index) + ('r' if retry else '')
    label = case['scenario'] + ('-' + case['variant'] if case['variant'] else '') + '-' + tag
    cmd = ['ros2', 'launch', 'smart_hub', 'hub.launch.py', 'scenario:=' + case['scenario'],
           'seed:=%d' % case['seed'], 'dispatch_mode:=' + case['mode'],
           'ui_port:=%d' % (opts.ui_port_base + index),
           'record:=' + ('true' if case['record'] else 'false')]
    if case['variant']:
        cmd.append('variant:=' + case['variant'])
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), SMART_HUB_RUN_TAG=tag)
    start = time.time()
    log = open(os.path.join(opts.out, 'logs', '%03d_%s.log' % (index, label)), 'w')
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    if case['kill']:
        at, pattern = case['kill']
        if isinstance(at, tuple):                 # (event, entity, reason, delay): event-based
            event, entity, reason_has, delay = at
            wait_for_event(opts.runs_dir, label, case['seed'], start, event, entity,
                           reason_has, 60)
            time.sleep(delay)
        else:
            time.sleep(max(0.0, STARTUP_SEC + at))
        pids = subprocess.run(['pgrep', '-g', str(proc.pid), '-f', 'lib/smart_hub/' + pattern],
                              capture_output=True, text=True).stdout.split()
        for pid in pids:
            os.kill(int(pid), signal.SIGKILL)
        kill_wall = time.time()
        if not pids:
            with lock:
                print('WARNING: %s: no %s process to kill' % (label, pattern), flush=True)
    time.sleep(max(0.0, start + STARTUP_SEC + case['duration'] - time.time()))
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    log.close()
    run_dir = newest_run(opts.runs_dir, label, case['seed'], start)
    row = {'test': case['test'], 'scenario': case['scenario'], 'variant': case['variant'],
           'seed': case['seed'], 'mode': case['mode'], 'run_id': os.path.basename(run_dir or '')}
    if not run_dir:
        row.update(passed=False, details='no run record found')
        return row
    events = load_events(run_dir)
    started = first(events, 'SCENARIO_STARTED', 'scenario_runner')
    missing = started['reason'].split('missing=')[-1] if started else 'all nodes'
    if missing and not retry:
        row.update(passed=False, details='startup incomplete', startup_missing=missing)
        return row
    time.sleep(1.0)                       # let metrics_logger write its final summary
    with open(os.path.join(run_dir, 'summary.json')) as f:
        summary = json.load(f)
    kwargs = {}
    if case['kill'] and 'kill_t' in inspect.signature(case['check']).parameters:
        # Map the wall-clock kill onto the run's event clock (t of metrics_logger).
        offset = os.path.getmtime(os.path.join(run_dir, 'events.csv')) - \
            (events[-1]['t'] if events else 0)
        kwargs['kill_t'] = kill_wall - offset
    try:
        ok, details, metrics = case['check'](events, summary, **kwargs)
    except Exception as exc:          # a broken record is a failed run, not a crash
        ok, details, metrics = False, 'check error: %r' % exc, {}
    row.update(passed=bool(ok), details=details, **metrics)
    with lock:
        print('%-8s %-9s %-26s seed %-3d %-10s %s  %s' % (
            row['test'], case['scenario'], case['variant'] or '-', case['seed'], case['mode'],
            'PASS' if ok else 'FAIL', details), flush=True)
    return row


def recheck(opts):
    """Re-evaluate the pass criteria on recorded runs (no new runs; kill cases kept)."""
    with open(os.path.join(opts.out, 'results.csv')) as f:
        rows = list(csv.DictReader(f))
    cases = {(c['test'], c['scenario'], c['variant'], str(c['seed']), c['mode']): c
             for c in build_cases(opts.repeat) if not c['kill']}
    out = []
    for row in rows:
        row['passed'] = row['passed'] == 'True'
        case = cases.get((row['test'], row['scenario'], row['variant'], row['seed'],
                          row['mode']))
        run_dir = os.path.join(opts.runs_dir, row['run_id'])
        if case and row['run_id'] and os.path.isdir(run_dir):
            with open(os.path.join(run_dir, 'summary.json')) as f:
                summary = json.load(f)
            ok, details, metrics = case['check'](load_events(run_dir), summary)
            row = {k: v for k, v in row.items() if k in (
                'test', 'scenario', 'variant', 'seed', 'mode', 'run_id')}
            row.update(passed=bool(ok), details=details, **metrics)
        out.append(row)
    write_summary(out, opts)


def write_summary(rows, opts):
    keys = sorted({k for r in rows for k in r})
    with open(os.path.join(opts.out, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    lines = ['# Verification matrix (T01-T18)', '',
             'Generated by `ros2 run smart_hub run_test_matrix`; one row per run, details '
             'in results.csv and evidence/runs/<run_id>/.', '',
             '| Test | Scenario | Variant | Mode | Runs | Passed | Details (last run) |',
             '|------|----------|---------|------|------|--------|--------------------|']
    groups = {}
    for r in rows:
        groups.setdefault((r['test'], r['scenario'], r['variant'], r['mode']), []).append(r)
    for (test, sc, var, mode), rs in groups.items():
        lines.append('| %s | %s | %s | %s | %d | %d/%d | %s |' % (
            test, sc, var or '-', mode, len(rs), sum(r['passed'] for r in rs), len(rs),
            rs[-1]['details']))
    pred = [r['queue_wait_mean_s'] for r in rows if r['scenario'] == 'S8' and
            r['mode'] == 'predictive' and r.get('queue_wait_mean_s') is not None]
    reac = [r['queue_wait_mean_s'] for r in rows if r['scenario'] == 'S8' and
            r['mode'] == 'reactive' and r.get('queue_wait_mean_s') is not None]
    if pred and reac:
        lines += ['', '## Predictive vs reactive dispatch (S8, identical seeds)', '',
                  '| Policy | Runs | Mean queue wait (s) |', '|---|---|---|',
                  '| predictive | %d | %.1f |' % (len(pred), sum(pred) / len(pred)),
                  '| reactive | %d | %.1f |' % (len(reac), sum(reac) / len(reac))]
    passed = sum(r['passed'] for r in rows)
    lines += ['', '**Total: %d/%d runs passed.**' % (passed, len(rows))]
    with open(os.path.join(opts.out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines[-1:]))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', default=os.path.join('evidence', 'test_matrix'))
    parser.add_argument('--runs-dir', default=os.path.join('evidence', 'runs'))
    parser.add_argument('--parallel', type=int, default=4)
    parser.add_argument('--repeat', type=int, default=10)
    parser.add_argument('--cases', default='', help='regex on test id, e.g. "T13|T05"')
    parser.add_argument('--domain-base', type=int, default=30)
    parser.add_argument('--ui-port-base', type=int, default=9000)
    parser.add_argument('--tag-base', type=int, default=int(time.time()) % 900 * 100,
                        help='run tags are unique per invocation')
    parser.add_argument('--recheck', action='store_true',
                        help='re-evaluate criteria on the recorded runs in <out>/results.csv')
    opts, _ = parser.parse_known_args(args)
    if opts.recheck:
        recheck(opts)
        return
    os.makedirs(os.path.join(opts.out, 'logs'), exist_ok=True)
    cases = [c for c in build_cases(opts.repeat) if re.search(opts.cases, c['test'])]
    # Longest first keeps the pool busy until the end.
    order = sorted(range(len(cases)), key=lambda i: -cases[i]['duration'])
    lock = threading.Lock()
    with ThreadPoolExecutor(max_workers=opts.parallel) as pool:
        futures = {i: pool.submit(run_case, cases[i], i, opts, lock) for i in order}
        rows = [futures[i].result() for i in sorted(futures)]
    write_summary(rows, opts)


if __name__ == '__main__':
    main()
