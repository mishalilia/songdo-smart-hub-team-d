# Author: Mikhail Li (12236295)
"""
dashboard_marker_publisher. Read-only: no control authority.

Publishes /hub/markers (5 Hz) for RViz (assignment 3.2): map graph coloured by edge state,
crossing ownership, queue zone, parking state, agents and obstacle footprints, driven
paths of bus and robot, pedestrians, and a status text panel (queue, prediction, bus,
crossing, safety). Subscribes to state topics only; it never publishes commands.
"""
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from smart_hub import hub_map
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R1T
from smart_hub_interfaces.msg import (
    BusArrival, CrossingState, DemandPrediction, EdgeStateArray, ParkingState, QueueState,
    SafetyState, VehicleStateArray,
)
from visualization_msgs.msg import Marker, MarkerArray

PUBLISH_PERIOD_SEC = 0.2
TRAIL_LENGTH = 300

GREY = (0.6, 0.6, 0.6, 1.0)
DARK = (0.3, 0.3, 0.3, 1.0)
GREEN = (0.2, 0.8, 0.2, 1.0)
AMBER = (1.0, 0.65, 0.0, 1.0)
RED = (0.9, 0.1, 0.1, 1.0)
BLUE = (0.2, 0.4, 1.0, 1.0)
CYAN = (0.3, 0.8, 1.0, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)

EDGE_COLOURS = {'NORMAL': GREY, 'CONGESTED': AMBER, 'BLOCKED': RED, 'UNKNOWN': DARK}
SLOT_COLOURS = {'FREE': GREEN, 'RESERVED': AMBER, 'OCCUPIED': RED}
AGENT_STYLE = {  # kind -> (colour, height)
    'bus': (BLUE, 0.5), 'car': (GREY, 0.3), 'robot': (GREEN, 0.3), 'obstacle': (RED, 0.4),
}


def _rgba(marker, colour, alpha=None):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
    if alpha is not None:
        marker.color.a = alpha


def _point(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


class DashboardMarkerPublisher(Node):

    def __init__(self):
        super().__init__('dashboard_marker_publisher')
        # EdgeState has no congestion flag; the display re-applies the density rule (12.2).
        self.declare_parameter('congestion_density_threshold', 0.70)
        check_parameters(self)
        self.markers_pub = self.create_publisher(MarkerArray, '/hub/markers', R10)

        self.vehicles = None
        self.pedestrians = None
        self.edges = {}
        self.crossing = None
        self.parking = None
        self.queue = None
        self.demand = None
        self.bus = None
        self.safety = None
        self.trails = {'robot1': [], 'bus1': []}
        self.tick = 0

        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(PoseArray, '/sim/pedestrians', self.on_pedestrians, R10)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(ParkingState, '/parking/state', self.on_parking, R1T)
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_subscription(DemandPrediction, '/hub/demand', self.on_demand, R10)
        self.create_subscription(BusArrival, '/hub/bus_arrival', self.on_bus, R10)
        self.create_subscription(SafetyState, '/hub/safety', self.on_safety, R1T)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10)
        self.create_timer(PUBLISH_PERIOD_SEC, self.publish)

    # ------------------------------------------------------------------ inputs
    def on_vehicles(self, msg):
        self.vehicles = msg
        for v in msg.vehicles:
            if v.vehicle_id == 'bus1':
                self._append_trail('bus1', v.pose.x, v.pose.y)

    def on_pedestrians(self, msg):
        self.pedestrians = msg

    def on_edges(self, msg):
        self.edges = {e.edge_id: e for e in msg.edges}

    def on_crossing(self, msg):
        self.crossing = msg

    def on_parking(self, msg):
        self.parking = msg

    def on_queue(self, msg):
        self.queue = msg

    def on_demand(self, msg):
        self.demand = msg

    def on_bus(self, msg):
        self.bus = msg

    def on_safety(self, msg):
        self.safety = msg

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self._append_trail('robot1', p.x, p.y)

    def _append_trail(self, agent_id, x, y):
        trail = self.trails[agent_id]
        if not trail or abs(trail[-1][0] - x) + abs(trail[-1][1] - y) > 0.02:
            trail.append((x, y))
            del trail[:-TRAIL_LENGTH]

    # ------------------------------------------------------------------ markers
    def _marker(self, ns, mid, mtype, stamp, lifetime=True):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        if lifetime:
            m.lifetime = Duration(sec=0, nanosec=int(2.5 * PUBLISH_PERIOD_SEC * 1e9))
        return m

    def _text(self, ns, mid, stamp, x, y, text, size=0.3, colour=WHITE):
        m = self._marker(ns, mid, Marker.TEXT_VIEW_FACING, stamp)
        m.pose.position = _point(x, y, 0.5)
        m.scale.z = size
        _rgba(m, colour)
        m.text = text
        return m

    def _edge_state(self, eid):
        e = self.edges.get(eid)
        if e is None or not e.valid:
            return 'UNKNOWN'
        if e.blocked:
            return 'BLOCKED'
        threshold = self.get_parameter('congestion_density_threshold').value
        return 'CONGESTED' if e.density_ratio >= threshold else 'NORMAL'

    def graph_markers(self, stamp):
        out = []
        vehicle = {hub_map.edge_id(a, b): (a, b)
                   for a, b in hub_map.VEHICLE_EDGES + hub_map.PARKING_EDGES}
        robot_only = [(a, b) for a, b in hub_map.ROBOT_EDGES
                      if hub_map.edge_id(a, b) not in vehicle]
        for i, (eid, (a, b)) in enumerate(sorted(vehicle.items())):
            m = self._marker('edges', i, Marker.LINE_LIST, stamp)
            m.scale.x = 0.08
            state = self._edge_state(eid)
            _rgba(m, EDGE_COLOURS[state])
            m.points = [_point(*hub_map.NODES[a]), _point(*hub_map.NODES[b])]
            out.append(m)
            mx = (hub_map.NODES[a][0] + hub_map.NODES[b][0]) / 2
            my = (hub_map.NODES[a][1] + hub_map.NODES[b][1]) / 2
            if state != 'NORMAL':
                out.append(self._text('edge_state', i, stamp, mx, my - 0.25, state, 0.2,
                                      EDGE_COLOURS[state]))
        m = self._marker('robot_edges', 0, Marker.LINE_LIST, stamp)
        m.scale.x = 0.04
        _rgba(m, CYAN, 0.7)
        for a, b in robot_only:
            m.points += [_point(*hub_map.NODES[a], 0.01), _point(*hub_map.NODES[b], 0.01)]
        out.append(m)
        for i, (name, (x, y)) in enumerate(sorted(hub_map.NODES.items())):
            out.append(self._text('nodes', i, stamp, x + 0.2, y + 0.2, name, 0.25))
        return out

    def zone_markers(self, stamp):
        out = []
        for i, (rect, colour) in enumerate(((hub_map.QUEUE_Q, BLUE),
                                            (hub_map.CROSSING_C, self._crossing_colour()))):
            xmin, xmax, ymin, ymax = rect
            m = self._marker('zones', i, Marker.CUBE, stamp)
            m.pose.position = _point((xmin + xmax) / 2, (ymin + ymax) / 2, -0.01)
            m.scale.x, m.scale.y, m.scale.z = xmax - xmin, ymax - ymin, 0.01
            _rgba(m, colour, 0.25)
            out.append(m)
        c = self.crossing
        label = 'C: UNKNOWN' if c is None or not c.valid else \
            'C: %s %s%s' % (c.state, c.owner_id, ' PED' if c.pedestrian_present else '')
        out.append(self._text('zones', 2, stamp, 5.0, 0.9, label, 0.25))
        return out

    def _crossing_colour(self):
        c = self.crossing
        if c is None or not c.valid:
            return DARK
        if c.pedestrian_present:
            return RED
        return AMBER if c.owner_id else GREEN

    def parking_markers(self, stamp):
        out = []
        states = {s.slot_id: s.state for s in self.parking.slots} if self.parking else {}
        for i, slot in enumerate(hub_map.PARKING_SLOTS):
            x, y = hub_map.NODES[slot]
            state = states.get(slot, 'UNKNOWN')
            m = self._marker('parking', i, Marker.CUBE, stamp)
            m.pose.position = _point(x, y + 0.6, 0.05)
            m.scale.x, m.scale.y, m.scale.z = 1.0, 0.8, 0.1
            if state == 'UNKNOWN':
                _rgba(m, AMBER, 0.9 if self.tick % 2 else 0.15)  # blinking amber
            else:
                _rgba(m, SLOT_COLOURS.get(state, DARK), 0.8)
            out.append(m)
            out.append(self._text('parking_text', i, stamp, x, y + 1.15,
                                  '%s %s' % (slot, state), 0.2))
        return out

    def agent_markers(self, stamp):
        out = []
        if self.vehicles is not None:
            for i, v in enumerate(self.vehicles.vehicles):
                colour, height = AGENT_STYLE.get(v.kind, (WHITE, 0.3))
                radius = 0.35 if v.kind == 'obstacle' else 0.20 if v.kind == 'robot' else 0.30
                m = self._marker('agents', i, Marker.CYLINDER, stamp)
                m.pose.position = _point(v.pose.x, v.pose.y, height / 2)
                m.scale.x = m.scale.y = 2 * radius
                m.scale.z = height
                _rgba(m, colour, 0.9 if v.valid else 0.3)
                out.append(m)
                out.append(self._text('agent_ids', i, stamp, v.pose.x, v.pose.y + radius + 0.15,
                                      v.vehicle_id, 0.18))
        if self.pedestrians is not None:
            m = self._marker('pedestrians', 0, Marker.SPHERE_LIST, stamp)
            m.scale.x = m.scale.y = m.scale.z = 0.3
            _rgba(m, WHITE, 0.9)
            m.points = [_point(p.position.x, p.position.y, 0.15) for p in self.pedestrians.poses]
            out.append(m)
        for i, (agent_id, colour) in enumerate((('robot1', GREEN), ('bus1', BLUE))):
            trail = self.trails[agent_id]
            if len(trail) >= 2:
                m = self._marker('paths', i, Marker.LINE_STRIP, stamp)
                m.scale.x = 0.03
                _rgba(m, colour, 0.6)
                m.points = [_point(x, y, 0.02) for x, y in trail]
                out.append(m)
        return out

    def status_text(self):
        q, d, b, s = self.queue, self.demand, self.bus, self.safety
        lines = [
            'Queue: %s' % ('%d (conf %.2f)' % (q.count, q.confidence) if q and q.valid
                           else 'UNKNOWN'),
            'Demand +%ss: %s' % (
                d.horizon_sec if d else 30,
                '%.1f (baseline %.1f, %s)' % (d.predicted_count, d.baseline_count, d.model_id)
                if d and d.valid else 'prediction unavailable'),
            'Bus: %s' % ('%s ETA %s%s' % (
                b.state, '%.0fs' % b.eta_sec if b.eta_valid else '-',
                ' DOOR OPEN' if b.door_open else '') if b else '-'),
            'Safety: %s' % ('STOP (%s)' % s.reason if s and not s.motion_permitted
                            else 'OK' if s else '-'),
        ]
        return '\n'.join(lines)

    def publish(self):
        self.tick += 1
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()
        out.markers += self.graph_markers(stamp)
        out.markers += self.zone_markers(stamp)
        out.markers += self.parking_markers(stamp)
        out.markers += self.agent_markers(stamp)
        out.markers.append(self._text('status', 0, stamp, 3.0, -1.1, self.status_text(), 0.3))
        self.markers_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
