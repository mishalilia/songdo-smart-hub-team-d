"""
hub_ui — owner M8.

Operator UI and metrics dashboard served over HTTP (default http://localhost:8080):
  /           control page — live map, hub state, robot mission progress, event log and
              operator commands (e-stop, clear stop, scenario reset, escort / navigate /
              cancel, passenger ready, parking reserve / release, barrier) plus
              simulation controls injected into scenario_runner via /sim/inject
  /dashboard  metrics dashboard — mission outcomes, safety counts, latency vs. assignment
              thresholds, topic data age and rates, energy per device, IoT device status
JSON API: GET /api/map, /api/state, /api/metrics; POST /api/command.
Commands go through the normal hub interfaces; the UI never publishes motion commands.
"""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import queue
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from smart_hub import hub_map
from smart_hub.qos import R1, R10, R100, R1T
from smart_hub.route_engine import plan
from smart_hub_interfaces.action import EscortPassenger, NavigateTo
from smart_hub_interfaces.msg import (
    BusArrival, CrossingState, DemandPrediction, EdgeStateArray, HubEvent, ParkingState,
    QueueState, SafetyState, VehicleStateArray,
)
from smart_hub_interfaces.srv import (
    ClearSafetyStop, ReleaseParking, ReserveParking, ResetScenario,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'application/javascript',
                 '.css': 'text/css'}


def _diag(msg):
    return {s.name: {'message': s.message, 'level': int.from_bytes(s.level, 'little')
                     if isinstance(s.level, bytes) else int(s.level),
                     **{kv.key: kv.value for kv in s.values}} for s in msg.status}


class HubUi(Node):

    def __init__(self):
        super().__init__('hub_ui')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('web_dir', '')
        self.declare_parameter('congestion_density_threshold', 0.70)

        self.lock = threading.Lock()
        self.state = {}
        self.metrics = {}
        self.iot = {}
        self.events = deque(maxlen=60)
        self.mission = {}
        self.goal_handle = None
        self.commands = queue.Queue()
        self.counter = 0

        subs = [
            (VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10),
            (PoseArray, '/sim/pedestrians', self.on_pedestrians, R10),
            (CrossingState, '/crossing/state', self.on_crossing, R1T),
            (ParkingState, '/parking/state', self.on_parking, R1T),
            (QueueState, '/hub/queue', self.on_queue, R10),
            (DemandPrediction, '/hub/demand', self.on_demand, R10),
            (BusArrival, '/hub/bus_arrival', self.on_bus, R10),
            (SafetyState, '/hub/safety', self.on_safety, R1T),
            (EdgeStateArray, '/traffic/edges', self.on_edges, R10),
            (HubEvent, '/hub/events', self.on_event, R100),
            (DiagnosticArray, '/hub/metrics', self.on_metrics, R10),
            (DiagnosticArray, '/iot/status', self.on_iot, R10),
            (Bool, '/parking/barrier_open', self.on_barrier, R1),
            (Bool, '/bus/door_open', self.on_door, R1),
        ]
        for msg_type, topic, cb, qos in subs:
            self.create_subscription(msg_type, topic, cb, qos)

        self.estop_pub = self.create_publisher(Bool, '/hub/estop', R1T)
        self.barrier_pub = self.create_publisher(Bool, '/parking/barrier_request', R1)
        self.inject_pub = self.create_publisher(String, '/sim/inject', R10)
        self.clients_ = {
            'reset': self.create_client(ResetScenario, '/scenario/reset'),
            'clear_stop': self.create_client(ClearSafetyStop, '/safety/clear_stop'),
            'confirm_ready': self.create_client(Trigger, '/robot/confirm_ready'),
            'reserve': self.create_client(ReserveParking, '/parking/reserve_slot'),
            'release': self.create_client(ReleaseParking, '/parking/release_slot'),
        }
        self.escort_client = ActionClient(self, EscortPassenger, '/robot/escort')
        self.navigate_client = ActionClient(self, NavigateTo, '/robot/navigate_to')
        self.create_timer(0.05, self.process_commands)
        self.start_http()

    # ------------------------------------------------------------------ state cache
    def put(self, key, value):
        with self.lock:
            self.state[key] = value

    def on_vehicles(self, msg):
        self.put('vehicles', [{'id': v.vehicle_id, 'kind': v.kind, 'x': v.pose.x, 'y': v.pose.y,
                               'theta': v.pose.theta, 'speed': v.speed_mps, 'edge': v.edge_id,
                               'valid': v.valid} for v in msg.vehicles])

    def on_pedestrians(self, msg):
        self.put('pedestrians', [[p.position.x, p.position.y] for p in msg.poses])

    def on_crossing(self, msg):
        self.put('crossing', {'owner': msg.owner_id, 'state': msg.state,
                              'pedestrian': msg.pedestrian_present, 'valid': msg.valid})

    def on_parking(self, msg):
        self.put('parking', {'available': msg.available_count, 'slots': [
            {'slot': s.slot_id, 'state': s.state, 'vehicle': s.vehicle_id,
             'reservation': s.reservation_id} for s in msg.slots]})

    def on_queue(self, msg):
        self.put('queue', {'count': msg.count, 'valid': msg.valid,
                           'confidence': msg.confidence,
                           'crossing_occupied': msg.crossing_occupied})

    def on_demand(self, msg):
        self.put('demand', {'predicted': msg.predicted_count, 'baseline': msg.baseline_count,
                            'horizon': msg.horizon_sec, 'model': msg.model_id,
                            'valid': msg.valid})

    def on_bus(self, msg):
        self.put('bus', {'state': msg.state, 'eta': msg.eta_sec, 'eta_valid': msg.eta_valid,
                         'door_open': msg.door_open})

    def on_safety(self, msg):
        self.put('safety', {'latched': msg.stop_latched, 'permitted': msg.motion_permitted,
                            'reason': msg.reason})

    def on_edges(self, msg):
        threshold = self.get_parameter('congestion_density_threshold').value
        edges = {}
        for e in msg.edges:
            label = 'UNKNOWN' if not e.valid else 'BLOCKED' if e.blocked else \
                'CONGESTED' if e.density_ratio >= threshold else 'NORMAL'
            edges[e.edge_id] = {'label': label, 'density': e.density_ratio,
                                'speed': e.mean_speed_mps}
        self.put('edges', edges)

    def on_event(self, msg):
        with self.lock:
            self.events.append({'t': time.strftime('%H:%M:%S'), 'entity': msg.entity_id,
                                'event': msg.event, 'reason': msg.reason})
            if msg.event == 'RUN_START':
                self.state['run'] = {'run_id': msg.run_id, 'scenario': msg.scenario_id}
            if msg.entity_id == 'robot1':
                self.track_mission(msg)

    def track_mission(self, msg):
        """Follow robot missions started by anyone (scenario, UI, CLI) from its events."""
        m = self.mission
        if msg.event == 'GOAL_ACCEPTED':
            if m.get('request_id') != msg.correlation_id:
                self.mission = m = {'kind': msg.reason, 'request_id': msg.correlation_id}
            m.update({'status': 'ACTIVE', 'reason': ''})
        elif msg.event == 'ROUTE':
            m['route'] = msg.reason
            m['phase'] = msg.reason.split(' ')[0]
        elif msg.event == 'PICKUP_ARRIVED':
            m['phase'] = 'TO_PICKUP/WAIT_READY'
        elif msg.event in ('REROUTE', 'CROSSING_ACQUIRED', 'CROSSING_RELEASED'):
            m['last'] = msg.event
        elif msg.event.startswith('GOAL_') and msg.event != 'GOAL_REJECTED':
            m.update({'status': msg.event[5:], 'reason': msg.reason})

    def on_metrics(self, msg):
        with self.lock:
            self.metrics = _diag(msg)

    def on_iot(self, msg):
        with self.lock:
            self.iot = _diag(msg)

    def on_barrier(self, msg):
        self.put('barrier_open', msg.data)

    def on_door(self, msg):
        self.put('door_open', msg.data)

    def log(self, event, reason=''):
        with self.lock:
            self.events.append({'t': time.strftime('%H:%M:%S'), 'entity': 'ui',
                                'event': event, 'reason': reason})

    # ------------------------------------------------------------------ commands
    def process_commands(self):
        while not self.commands.empty():
            cmd = self.commands.get_nowait()
            try:
                self.execute(cmd)
            except (KeyError, ValueError, TypeError) as exc:
                self.log('UI_COMMAND_INVALID', '%s: %s' % (cmd.get('cmd'), exc))

    def call(self, name, request):
        client = self.clients_[name]
        if not client.service_is_ready():
            self.log('UI_' + name.upper() + '_FAILED', 'service unavailable')
            return
        future = client.call_async(request)

        def done(f):
            res = f.result()
            ok = getattr(res, 'success', True)
            reason = getattr(res, 'reason', getattr(res, 'message', ''))
            if name == 'reserve' and ok:
                reason = '%s token=%s' % (res.slot_id, res.reservation_id)
            elif name == 'reset' and ok:
                reason = res.run_id
            self.log('UI_' + name.upper() + ('_OK' if ok else '_REJECTED'), reason)

        future.add_done_callback(done)

    def execute(self, cmd):
        kind = cmd['cmd']
        if kind == 'estop':
            self.estop_pub.publish(Bool(data=True))
            self.log('UI_ESTOP', 'operator e-stop')
        elif kind == 'clear_stop':
            self.call('clear_stop', ClearSafetyStop.Request(
                operator_id='ui', acknowledgement=cmd.get('ack', 'operator confirmed clear')))
        elif kind == 'reset':
            self.call('reset', ResetScenario.Request(
                scenario_id=cmd.get('scenario', ''), seed=int(cmd.get('seed', 42)),
                preserve_logs=True))
        elif kind == 'confirm_ready':
            self.call('confirm_ready', Trigger.Request())
        elif kind == 'reserve':
            self.counter += 1
            self.call('reserve', ReserveParking.Request(
                request_id='ui-%d-%d' % (int(time.time()), self.counter),
                vehicle_id=cmd['vehicle'], preferred_slot=cmd.get('slot', ''),
                ttl_sec=float(cmd.get('ttl', 30))))
        elif kind == 'release':
            self.counter += 1
            self.call('release', ReleaseParking.Request(
                request_id='ui-rel-%d' % self.counter, reservation_id=cmd['reservation'],
                vehicle_id=cmd['vehicle']))
        elif kind == 'barrier':
            self.barrier_pub.publish(Bool(data=bool(cmd['open'])))
            self.log('UI_BARRIER', 'open' if cmd['open'] else 'close')
        elif kind == 'sim':
            self.inject(cmd['event'])
        elif kind == 'sim_route':
            self.sim_route(cmd['agent'], cmd['goal'])
        elif kind in ('escort', 'navigate'):
            self.send_goal(kind, cmd)
        elif kind == 'cancel':
            if self.goal_handle is None:
                self.log('UI_CANCEL_SKIPPED', 'no active goal')
            else:
                self.goal_handle.cancel_goal_async()
                self.log('UI_CANCEL_REQUESTED')
        else:
            raise ValueError('unknown command')

    def inject(self, event):
        if not isinstance(event, dict) or 'type' not in event:
            raise ValueError('event needs a type')
        # sent_ns lets scenario_runner report UI -> simulator latency.
        self.inject_pub.publish(String(data=json.dumps(dict(event, sent_ns=time.time_ns()))))
        self.log('UI_SIM', event['type'])

    def sim_route(self, agent, goal):
        """Scripted simulator route for bus1 / robot1 from its nearest node (no M7 needed)."""
        with self.lock:
            vehicles = {v['id']: v for v in self.state.get('vehicles', [])}
        if agent not in vehicles:
            raise ValueError('no pose for ' + agent)
        v = vehicles[agent]
        kind = 'robot' if agent == 'robot1' else 'bus'
        nodes = {n for e in (hub_map.ROBOT_EDGES if kind == 'robot' else hub_map.VEHICLE_EDGES)
                 for n in e}
        start = min(nodes, key=lambda n: math.dist((v['x'], v['y']), hub_map.NODES[n]))
        blocked = {eid for eid, e in self.state.get('edges', {}).items()
                   if e['label'] == 'BLOCKED'}
        route, _ = plan(start, goal, kind, blocked) if start != goal else ([start], 0.0)
        if not route:
            self.log('UI_SIM_ROUTE_FAILED', '%s %s->%s NO_ROUTE' % (agent, start, goal))
            return
        self.inject({'type': 'agent_route', 'agent': agent, 'route': route})
        self.log('UI_SIM_ROUTE', '%s %s' % (agent, '-'.join(route)))

    def send_goal(self, kind, cmd):
        self.counter += 1
        request_id = 'ui-%s-%d' % (kind, self.counter)
        if kind == 'escort':
            client, goal = self.escort_client, EscortPassenger.Goal(
                request_id=request_id, passenger_id='P%02d' % self.counter,
                pickup_node=cmd.get('pickup', 'R'), destination_node=cmd.get('destination', 'D'),
                max_duration_sec=float(cmd.get('max_duration', 240)))
        else:
            client, goal = self.navigate_client, NavigateTo.Goal(
                request_id=request_id, goal_node=cmd['goal'],
                max_duration_sec=float(cmd.get('max_duration', 240)))
        if not client.server_is_ready():
            self.log('UI_GOAL_FAILED', 'action server unavailable')
            return
        with self.lock:
            self.mission = {'kind': kind, 'request_id': request_id, 'status': 'SENT'}

        def feedback(fb):
            with self.lock:
                self.mission.update({'phase': fb.feedback.phase,
                                     'progress': fb.feedback.progress,
                                     'remaining': fb.feedback.distance_remaining_m})

        def result(f):
            res = f.result()
            status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(res.status, res.status)
            with self.lock:
                self.mission.update({'status': status, 'reason': res.result.reason})
            self.goal_handle = None
            self.log('UI_GOAL_' + str(status), res.result.reason)

        def accepted(f):
            handle = f.result()
            if not handle.accepted:
                with self.lock:
                    self.mission['status'] = 'REJECTED'
                self.log('UI_GOAL_REJECTED', request_id)
                return
            self.goal_handle = handle
            with self.lock:
                self.mission['status'] = 'ACTIVE'
            handle.get_result_async().add_done_callback(result)

        client.send_goal_async(goal, feedback_callback=feedback).add_done_callback(accepted)

    # ------------------------------------------------------------------ HTTP
    def snapshot(self):
        with self.lock:
            out = dict(self.state)
            out['events'] = list(self.events)[-40:]
            out['mission'] = dict(self.mission)
            out['iot'] = dict(self.iot)
        return out

    def map_json(self):
        return {
            'nodes': hub_map.NODES,
            'vehicle_edges': hub_map.VEHICLE_EDGES, 'parking_edges': hub_map.PARKING_EDGES,
            'robot_edges': hub_map.ROBOT_EDGES,
            'crossing': hub_map.CROSSING_C, 'queue': hub_map.QUEUE_Q,
            'bounds': hub_map.MAP_BOUNDS, 'slots': hub_map.PARKING_SLOTS,
            'edge_ids': {hub_map.edge_id(a, b): [a, b] for a, b in
                         hub_map.VEHICLE_EDGES + hub_map.PARKING_EDGES + hub_map.ROBOT_EDGES},
        }

    def web_dir(self):
        directory = self.get_parameter('web_dir').value
        if directory:
            return directory
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('smart_hub'), 'web')

    def start_http(self):
        node = self
        web_dir = self.web_dir()

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, *args):
                pass

            def send(self, code, body, content_type='application/json'):
                data = body if isinstance(body, bytes) else json.dumps(body).encode()
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                path = self.path.split('?')[0]
                if path == '/api/state':
                    return self.send(200, node.snapshot())
                if path == '/api/metrics':
                    with node.lock:
                        return self.send(200, dict(node.metrics))
                if path == '/api/map':
                    return self.send(200, node.map_json())
                name = {'/': 'control.html', '/dashboard': 'dashboard.html'}.get(
                    path, path.lstrip('/'))
                full = os.path.normpath(os.path.join(web_dir, name))
                if '..' in name.split('/') or not full.startswith(os.path.normpath(web_dir)) \
                        or not os.path.isfile(full):
                    return self.send(404, {'error': 'not found'})
                with open(full, 'rb') as f:
                    self.send(200, f.read(),
                              CONTENT_TYPES.get(os.path.splitext(full)[1], 'text/plain'))

            def do_POST(self):
                if self.path != '/api/command':
                    return self.send(404, {'error': 'not found'})
                length = int(self.headers.get('Content-Length', 0))
                try:
                    cmd = json.loads(self.rfile.read(length) or b'{}')
                    if 'cmd' not in cmd:
                        raise ValueError('missing cmd')
                except ValueError as exc:
                    return self.send(400, {'error': str(exc)})
                node.commands.put(cmd)
                self.send(202, {'queued': cmd['cmd']})

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        self.httpd = ThreadingHTTPServer((host, port), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info('UI on http://%s:%d  (dashboard: /dashboard)' % (
            'localhost' if host == '0.0.0.0' else host, port))

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HubUi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
