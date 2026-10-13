"""
scenario_runner — owner M8.

Deterministic 2D simulator at 20 Hz (assignment 3.2) wrapping sim_world.World:
seeded scenario events, private ground truth, noisy observations for detectors.
Also the test driver: scenario events may call hub services / actions.
Serves /scenario/reset (test orchestration only, assignment 7.1).

Published observations:
  /camera/image_raw       sensor_msgs/Image       top-down synthetic image, 10 Hz
  /sim/pedestrians        geometry_msgs/PoseArray noisy pedestrian foot-points (map), 10 Hz
  /hub/vehicles           VehicleStateArray       bus, robot, cars, obstacles (kind=obstacle)
  /parking/observed       SlotObservationArray    5 Hz
  /robot/odom             nav_msgs/Odometry       20 Hz, plus TF <agent>/odom -> base_link
  /hub/estop              std_msgs/Bool           10 Hz (scenario-driven stop input)
Private ground truth (metrics_logger only):
  /sim/truth/vehicles     VehicleStateArray
  /sim/truth/pedestrians  geometry_msgs/PoseArray
"""
from datetime import datetime
import math
import os

from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from smart_hub import hub_map
from smart_hub.qos import R1, R10, R100, R1T, SENSOR
from smart_hub.sim_world import STEP_SEC, World
from smart_hub_interfaces.action import DockBus, EscortPassenger
from smart_hub_interfaces.msg import (
    HubEvent, SlotObservation, SlotObservationArray, VehicleState, VehicleStateArray,
)
from smart_hub_interfaces.srv import (
    ClearSafetyStop, ReleaseParking, ReserveParking, ResetScenario,
)
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

PIXELS_PER_M = 10


def _yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _pose(x, y):
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.w = 1.0
    return p


def _goal_id(handle):
    return ''.join('%02x' % b for b in bytes(handle.goal_id.uuid))


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('scenario_runner')
        self.declare_parameter('scenario', 'S1')
        self.declare_parameter('seed', 42)
        self.declare_parameter('scenario_dir', '')
        self.declare_parameter('pedestrian_noise_m', 0.05)
        self.declare_parameter('vehicle_noise_m', 0.02)
        self.declare_parameter('publish_image', True)

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', SENSOR)
        self.peds_pub = self.create_publisher(PoseArray, '/sim/pedestrians', R10)
        self.vehicles_pub = self.create_publisher(VehicleStateArray, '/hub/vehicles', R10)
        self.slots_pub = self.create_publisher(SlotObservationArray, '/parking/observed', R10)
        self.odom_pub = self.create_publisher(Odometry, '/robot/odom', R10)
        self.estop_pub = self.create_publisher(Bool, '/hub/estop', R1T)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.truth_vehicles_pub = self.create_publisher(
            VehicleStateArray, '/sim/truth/vehicles', R10)
        self.truth_peds_pub = self.create_publisher(PoseArray, '/sim/truth/pedestrians', R10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_frames()

        self.create_subscription(Twist, '/robot/cmd_vel', self.on_robot_cmd, R1)
        self.create_subscription(Twist, '/bus/cmd_vel', self.on_bus_cmd, R1)
        self.create_subscription(Bool, '/bus/door_open', self.on_door_open, R1)
        self.create_service(ResetScenario, '/scenario/reset', self.on_reset)

        # Test-driver clients (scenario events that exercise other nodes).
        self.reserve_client = self.create_client(ReserveParking, '/parking/reserve_slot')
        self.release_client = self.create_client(ReleaseParking, '/parking/release_slot')
        self.ready_client = self.create_client(Trigger, '/robot/confirm_ready')
        self.clear_stop_client = self.create_client(ClearSafetyStop, '/safety/clear_stop')
        self.dock_client = ActionClient(self, DockBus, '/bus/dock')
        self.escort_client = ActionClient(self, EscortPassenger, '/robot/escort')
        self.goal_handles = {}

        self.vehicle_cache = {}
        self.tick_count = 0
        self.completed = False
        self.world = None
        self.scenario_id = ''
        self.run_id = ''
        self.load(self.get_parameter('scenario').value, self.get_parameter('seed').value)

        self.create_timer(STEP_SEC, self.step)

    # ------------------------------------------------------------------ scenario
    def _scenario_file(self, scenario_id):
        directory = self.get_parameter('scenario_dir').value
        if not directory:
            from ament_index_python.packages import get_package_share_directory
            directory = os.path.join(get_package_share_directory('smart_hub'), 'scenarios')
        return os.path.join(directory, scenario_id + '.yaml')

    def load(self, scenario_id, seed):
        with open(self._scenario_file(scenario_id)) as f:
            scenario = yaml.safe_load(f)
        self.scenario_id = scenario_id
        self.world = World(scenario, seed)
        self.run_id = '%s-s%d-%s' % (scenario_id, seed, datetime.now().strftime('%Y%m%d-%H%M%S'))
        self.vehicle_cache = {}
        self.completed = False
        self.event('scenario_runner', 'RUN_START', 'seed=%d' % seed, self.run_id)
        self.get_logger().info('Loaded %s seed=%d run_id=%s' % (scenario_id, seed, self.run_id))

    def event(self, entity_id, name, reason='', correlation_id=''):
        msg = HubEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.run_id = self.run_id
        msg.scenario_id = self.scenario_id
        msg.entity_id = entity_id
        msg.event = name
        msg.reason = reason
        msg.correlation_id = correlation_id
        self.events_pub.publish(msg)

    # ------------------------------------------------------------------ inputs
    def on_robot_cmd(self, msg):
        self.world.command('robot1', msg.linear.x, msg.angular.z)

    def on_bus_cmd(self, msg):
        self.world.command('bus1', msg.linear.x, msg.angular.z)

    def on_door_open(self, msg):
        self.world.set_door_open(msg.data)

    def on_reset(self, request, response):
        scenario_id = request.scenario_id or self.scenario_id
        for handle in self.goal_handles.values():
            handle.cancel_goal_async()
        self.goal_handles = {}
        try:
            self.load(scenario_id, int(request.seed))
        except (OSError, yaml.YAMLError, KeyError, ValueError) as exc:
            response.success = False
            response.reason = 'RESET_FAILED: %s' % exc
            return response
        self.event('scenario_runner', 'RESET', 'preserve_logs=%s' % request.preserve_logs,
                   self.run_id)
        # Reset acknowledgement by every node is not part of the interface contracts yet;
        # nodes observe RESET on /hub/events (correlation_id = new run_id).
        response.success = True
        response.run_id = self.run_id
        response.reason = 'OK'
        return response

    # ------------------------------------------------------------------ loop
    def step(self):
        w = self.world
        w.step()
        for e in w.log:
            self.event(e.entity_id, e.event, e.reason, e.correlation_id)
        w.log.clear()
        for request in w.requests:
            self.drive(request)
        w.requests.clear()
        if w.finished() and not self.completed:
            self.completed = True
            self.event('scenario_runner', 'SCENARIO_COMPLETE', 't=%.2f' % w.t, self.run_id)

        stamp = self.get_clock().now().to_msg()
        self.tick_count += 1
        self.publish_odometry(stamp)
        if self.tick_count % 2 == 0:
            self.publish_vehicles(stamp)
            self.publish_pedestrians(stamp)
            self.estop_pub.publish(Bool(data=w.estop))
        if self.tick_count % 4 == 0:
            self.publish_slots(stamp)

    # ------------------------------------------------------------------ publishers
    def _publish_static_frames(self):
        frames = []
        for agent in ('robot', 'bus'):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = agent + '/odom'
            t.transform.rotation.w = 1.0
            frames.append(t)
        self.static_tf.sendTransform(frames)

    def publish_odometry(self, stamp):
        if self.world.odom_frozen():
            return
        transforms = []
        for agent_id, prefix in (('robot1', 'robot'), ('bus1', 'bus')):
            a = self.world.agents[agent_id]
            qx, qy, qz, qw = _yaw_to_quaternion(a.yaw)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = prefix + '/odom'
            t.child_frame_id = prefix + '/base_link'
            t.transform.translation.x, t.transform.translation.y = a.x, a.y
            t.transform.rotation.x, t.transform.rotation.y = qx, qy
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            transforms.append(t)
            if agent_id == 'robot1':
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'robot/odom'
                odom.child_frame_id = 'robot/base_link'
                odom.pose.pose.position.x, odom.pose.pose.position.y = a.x, a.y
                o = odom.pose.pose.orientation
                o.x, o.y, o.z, o.w = qx, qy, qz, qw
                odom.twist.twist.linear.x = a.v
                odom.twist.twist.angular.z = a.w
                self.odom_pub.publish(odom)
        self.tf_broadcaster.sendTransform(transforms)

    def _vehicle_msg(self, stamp, a, x, y, yaw, speed, edge_id):
        v = VehicleState()
        v.header.stamp = stamp
        v.header.frame_id = 'map'
        v.vehicle_id = a.agent_id
        v.kind = a.kind
        v.pose.x, v.pose.y, v.pose.theta = float(x), float(y), float(yaw)
        v.speed_mps = float(speed)
        v.edge_id = edge_id
        v.valid = True
        return v

    def publish_vehicles(self, stamp):
        noise = self.get_parameter('vehicle_noise_m').value
        observed = VehicleStateArray()
        observed.header.stamp = stamp
        observed.header.frame_id = 'map'
        truth = VehicleStateArray()
        truth.header = observed.header
        for a, x, y, yaw, speed, edge_id in self.world.observed_vehicles(noise):
            if a.frozen and a.agent_id in self.vehicle_cache:
                # Frozen source: repeat the old observation with its original stamp.
                observed.vehicles.append(self.vehicle_cache[a.agent_id])
            else:
                msg = self._vehicle_msg(stamp, a, x, y, yaw, speed, edge_id)
                self.vehicle_cache[a.agent_id] = msg
                observed.vehicles.append(msg)
            truth.vehicles.append(
                self._vehicle_msg(stamp, a, a.x, a.y, a.yaw, abs(a.v), self.world.edge_of(a)))
        self.vehicles_pub.publish(observed)
        self.truth_vehicles_pub.publish(truth)

    def publish_pedestrians(self, stamp):
        truth = PoseArray()
        truth.header.stamp = stamp
        truth.header.frame_id = 'map'
        for _, x, y, _ in self.world.truth_pedestrians():
            truth.poses.append(_pose(x, y))
        self.truth_peds_pub.publish(truth)

        observed = self.world.observed_pedestrians(
            self.get_parameter('pedestrian_noise_m').value)
        if observed is None:
            return  # camera frozen: silence
        msg = PoseArray()
        msg.header = truth.header
        msg.poses = [_pose(x, y) for x, y in observed]
        self.peds_pub.publish(msg)
        if self.get_parameter('publish_image').value:
            self.image_pub.publish(self._render(stamp, observed))

    def _render(self, stamp, points):
        xmin, xmax, ymin, ymax = hub_map.MAP_BOUNDS
        width = int((xmax - xmin) * PIXELS_PER_M)
        height = int((ymax - ymin) * PIXELS_PER_M)
        img = np.zeros((height, width), dtype=np.uint8)
        for x, y in points:
            col = int((x - xmin) * PIXELS_PER_M)
            row = int((ymax - y) * PIXELS_PER_M)
            img[max(0, row - 2):row + 3, max(0, col - 2):col + 3] = 255
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'
        msg.height, msg.width = height, width
        msg.encoding = 'mono8'
        msg.step = width
        msg.data = img.tobytes()
        return msg

    def publish_slots(self, stamp):
        msg = SlotObservationArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        for slot_id, occupied in self.world.observed_slots().items():
            s = SlotObservation()
            s.header = msg.header
            s.slot_id = slot_id
            s.occupied = occupied
            s.confidence = 1.0
            s.valid = True
            msg.slots.append(s)
        self.slots_pub.publish(msg)

    # ------------------------------------------------------------------ test driver
    def drive(self, ev):
        kind = ev['type']
        if kind == 'reserve':
            req = ReserveParking.Request()
            req.request_id = ev['request_id']
            req.vehicle_id = ev['client']
            req.preferred_slot = ev.get('preferred_slot', '')
            req.ttl_sec = float(ev.get('ttl_sec', 30.0))
            self._call(self.reserve_client, req, kind, ev['client'], self._on_reserved)
        elif kind == 'release':
            req = ReleaseParking.Request()
            req.request_id = ev['request_id']
            req.reservation_id = ev.get('reservation_id', '')
            req.vehicle_id = ev['client']
            self._call(self.release_client, req, kind, ev['client'])
        elif kind == 'confirm_ready':
            self._call(self.ready_client, Trigger.Request(), kind, 'robot1')
        elif kind == 'clear_stop_request':
            req = ClearSafetyStop.Request()
            req.operator_id = 'scenario_runner'
            req.acknowledgement = ev.get('acknowledgement', 'scenario')
            self._call(self.clear_stop_client, req, kind, 'safety_supervisor')
        elif kind == 'dock_request':
            goal = DockBus.Goal()
            goal.bus_id = ev.get('bus_id', 'bus1')
            goal.bay_id = ev.get('bay_id', 'B')
            goal.max_duration_sec = float(ev.get('max_duration_sec', 240.0))
            self._send_goal(self.dock_client, goal, 'dock')
        elif kind == 'escort_request':
            goal = EscortPassenger.Goal()
            goal.request_id = ev.get('request_id', 'escort-01')
            goal.passenger_id = ev.get('passenger_id', 'P01')
            goal.pickup_node = ev.get('pickup', 'R')
            goal.destination_node = ev.get('destination', 'D')
            goal.max_duration_sec = float(ev.get('max_duration_sec', 240.0))
            self._send_goal(self.escort_client, goal, 'escort')
        elif kind == 'cancel_goal':
            handle = self.goal_handles.get(ev['target'])
            if handle is None:
                self.event('scenario_runner', 'CANCEL_SKIPPED', 'no active ' + ev['target'])
            else:
                handle.cancel_goal_async()
                self.event('scenario_runner', 'CANCEL_REQUESTED', ev['target'],
                           _goal_id(handle))

    def _call(self, client, request, name, entity_id, on_result=None):
        if not client.service_is_ready():
            self.event(entity_id, name.upper() + '_FAILED', 'service unavailable')
            return
        future = client.call_async(request)

        def done(f):
            try:
                result = f.result()
            except Exception as exc:
                self.event(entity_id, name.upper() + '_FAILED', str(exc))
                return
            ok = getattr(result, 'success', True)
            reason = getattr(result, 'reason', getattr(result, 'message', ''))
            self.event(entity_id, name.upper() + ('_OK' if ok else '_REJECTED'), reason)
            if on_result is not None:
                on_result(entity_id, result)

        future.add_done_callback(done)

    def _on_reserved(self, client_id, result):
        if result.success:
            self.world.reservations[client_id] = result.slot_id

    def _send_goal(self, client, goal, name):
        if not client.server_is_ready():
            self.event('scenario_runner', name.upper() + '_FAILED', 'action server unavailable')
            return
        future = client.send_goal_async(goal)

        def accepted(f):
            handle = f.result()
            if not handle.accepted:
                self.event('scenario_runner', name.upper() + '_REJECTED', '', _goal_id(handle))
                return
            self.goal_handles[name] = handle
            self.event('scenario_runner', name.upper() + '_ACCEPTED', '', _goal_id(handle))
            handle.get_result_async().add_done_callback(
                lambda r: self._on_goal_result(name, handle, r))

        future.add_done_callback(accepted)

    def _on_goal_result(self, name, handle, future):
        result = future.result()
        if self.goal_handles.get(name) is handle:
            del self.goal_handles[name]
        self.event('scenario_runner', name.upper() + '_RESULT',
                   'status=%d reason=%s' % (result.status, result.result.reason),
                   _goal_id(handle))


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
