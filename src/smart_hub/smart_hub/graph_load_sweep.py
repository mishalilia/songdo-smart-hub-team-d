# Author: Mikhail Li (12236295)
"""
Whole-graph load sweep: the full launch (14 nodes) under growing simulated load.

    ros2 run smart_hub graph_load_sweep --out evidence/graph_load.csv

For every variant of scenarios/LOAD.yaml (idle, light, medium, heavy, saturated) one launch
runs alone on the machine. The harness reads the CPU time of every node process from
/proc/<pid>/stat over a measurement window after the load is placed (executor load, % of
one core); the run record gives the receipt age of each topic (stamp to metrics_logger
receipt, p95 / max: the backlog an observer sees), the measured rates and the graph audit
of test_matrix. Placed agents come from the LOAD_SPAWNED events. One machine, one seed:
the rows show how the graph degrades, not a capacity guarantee for other hardware.
"""
import argparse
import csv
import json
import os
import signal
import subprocess
import time

from smart_hub.test_matrix import graph_problems, load_events, newest_run, STARTUP_SEC

VARIANTS = ('idle', 'light', 'medium', 'heavy', 'saturated')
WINDOW = (20.0, 60.0)          # measurement window, seconds after the scenario clock starts
AGE_TOPICS = ('/hub/vehicles', '/robot/odom', '/hub/safety', '/crossing/state', '/hub/queue',
              '/camera/image_raw')
CLK_TCK = os.sysconf('SC_CLK_TCK')


def node_cpu_seconds(group):
    """Return {node executable: CPU seconds} of the processes in a process group."""
    pids = subprocess.run(['pgrep', '-g', str(group)], capture_output=True,
                          text=True).stdout.split()
    out = {}
    for pid in pids:
        try:
            with open('/proc/%s/cmdline' % pid, 'rb') as f:
                cmd = f.read().split(b'\0')
            with open('/proc/%s/stat' % pid) as f:
                fields = f.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        exe = next((c.decode().rsplit('/', 1)[-1] for c in cmd
                    if b'lib/smart_hub/' in c), None)
        if exe:
            out[exe] = out.get(exe, 0.0) + (int(fields[11]) + int(fields[12])) / CLK_TCK
    return out


def run_variant(variant, seed, runs_dir, ui_port):
    label = 'LOAD-' + variant
    start = time.time()
    proc = subprocess.Popen(
        ['ros2', 'launch', 'smart_hub', 'hub.launch.py', 'scenario:=LOAD',
         'variant:=' + variant, 'seed:=%d' % seed, 'ui_port:=%d' % ui_port, 'record:=false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
        env=dict(os.environ, FASTDDS_BUILTIN_TRANSPORTS=os.environ.get(
            'FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')))
    time.sleep(STARTUP_SEC + WINDOW[0])
    t0, cpu0 = time.monotonic(), node_cpu_seconds(proc.pid)
    time.sleep(WINDOW[1] - WINDOW[0])
    t1, cpu1 = time.monotonic(), node_cpu_seconds(proc.pid)
    time.sleep(8.0)
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
    time.sleep(1.0)
    run_dir = newest_run(runs_dir, label, seed, start)
    if not run_dir:
        return {'variant': variant, 'problem': 'no run record'}
    with open(os.path.join(run_dir, 'summary.json')) as f:
        summary = json.load(f)
    events = load_events(run_dir)
    row = {'variant': variant, 'run_id': os.path.basename(run_dir)}
    for e in events:
        if e['event'] == 'LOAD_SPAWNED':
            kind, value = e['reason'].split('=')
            row['%s_placed' % kind], row['%s_requested' % kind] = value.split('/')
    cpu = {k: 100.0 * (cpu1.get(k, 0.0) - cpu0.get(k, 0.0)) / (t1 - t0) for k in cpu1}
    busiest = max(cpu, key=cpu.get) if cpu else ''
    row.update(cpu_total_pct=round(sum(cpu.values()), 1), busiest_node=busiest,
               busiest_cpu_pct=round(cpu.get(busiest, 0.0), 1),
               cpu_by_node=json.dumps({k: round(v, 1) for k, v in sorted(cpu.items())}))
    ages = summary.get('topic_age_ms', {})
    for topic in AGE_TOPICS:
        a = ages.get(topic, {})
        row['%s_age_p95_ms' % topic.strip('/').replace('/', '_')] = round(a.get('p95', -1), 1)
        row['%s_age_max_ms' % topic.strip('/').replace('/', '_')] = round(a.get('max', -1), 1)
    problems = graph_problems(summary)
    row['graph_audit'] = 'ok' if not problems else '; '.join(problems)
    return row


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--variants', nargs='*', default=list(VARIANTS))
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--runs-dir', default=os.path.join('evidence', 'runs'))
    parser.add_argument('--ui-port', type=int, default=8090)
    parser.add_argument('--out', default=os.path.join('evidence', 'graph_load.csv'))
    opts, _ = parser.parse_known_args(args)
    rows = []
    for variant in opts.variants:
        row = run_variant(variant, opts.seed, opts.runs_dir, opts.ui_port)
        print(json.dumps({k: v for k, v in row.items() if k != 'cpu_by_node'}))
        rows.append(row)
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    os.makedirs(os.path.dirname(opts.out) or '.', exist_ok=True)
    with open(opts.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


if __name__ == '__main__':
    main()
