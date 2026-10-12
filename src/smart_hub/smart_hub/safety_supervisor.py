"""
safety_supervisor — owner M9 (scenario S9).

The ONLY publisher of final velocities (/robot/cmd_vel, /bus/cmd_vel). 20 Hz tick on a
monotonic clock: e-stop latch, freshness of critical inputs and candidate commands,
clearance, door interlock, speed caps, crossing permission. Starts latched (no motion
until /safety/clear_stop succeeds). See assignment 16, 18.7.
"""
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from smart_hub.qos import R1, R10, R1T
from smart_hub_interfaces.msg import CrossingState, QueueState, SafetyState, VehicleStateArray
from smart_hub_interfaces.srv import ClearSafetyStop
from std_msgs.msg import Bool


class SafetySupervisor(Node):

    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('robot_speed_limit', 0.25)
        self.declare_parameter('bus_speed_limit', 0.40)
        self.declare_parameter('max_angular_rad_s', 0.8)

        self.stop_latched = True
        self.reason = 'STARTUP_LATCHED'
        self.candidates = {'robot': (None, None), 'bus': (None, None)}  # (twist, receipt time)

        self.final_pubs = {
            'robot': self.create_publisher(Twist, '/robot/cmd_vel', R1),
            'bus': self.create_publisher(Twist, '/bus/cmd_vel', R1),
        }
        self.safety_pub = self.create_publisher(SafetyState, '/hub/safety', R1T)

        self.create_subscription(
            TwistStamped, '/robot/cmd_vel_candidate', lambda m: self.on_candidate('robot', m), R1)
        self.create_subscription(
            TwistStamped, '/bus/cmd_vel_candidate', lambda m: self.on_candidate('bus', m), R1)
        self.create_subscription(Bool, '/hub/estop', self.on_estop, R1T)
        self.create_subscription(Odometry, '/robot/odom', self.on_odom, R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_subscription(QueueState, '/hub/queue', self.on_queue, R10)
        self.create_service(ClearSafetyStop, '/safety/clear_stop', self.on_clear_stop)

        self.create_timer(0.05, self.tick)

    def on_candidate(self, agent, msg):
        self.candidates[agent] = (msg.twist, time.monotonic())

    def on_estop(self, msg):
        if msg.data:
            self.stop_latched = True
            self.reason = 'ESTOP'
        # A false message never clears the latch.

    def on_odom(self, msg):
        pass  # TODO(M9): odometry freshness (> 1 s gap => stop + abort escort)

    def on_vehicles(self, msg):
        pass  # TODO(M9): bus pose freshness, clearance checks

    def on_crossing(self, msg):
        pass  # TODO(M9): crossing permission per agent

    def on_queue(self, msg):
        pass  # TODO(M9): pedestrian in C / UNKNOWN => deny entry

    def on_clear_stop(self, request, response):
        # TODO(M9): reject while hazards, stale critical sensors, e-stop asserted or motion != 0
        response.success = False
        response.reason = 'NOT_IMPLEMENTED'
        return response

    def tick(self):
        now = time.monotonic()
        timeout = self.get_parameter('command_timeout_sec').value
        limits = {
            'robot': self.get_parameter('robot_speed_limit').value,
            'bus': self.get_parameter('bus_speed_limit').value,
        }
        w_max = self.get_parameter('max_angular_rad_s').value
        for agent, pub in self.final_pubs.items():
            out = Twist()
            twist, received = self.candidates[agent]
            if not self.stop_latched and twist is not None and now - received <= timeout:
                # TODO(M9): remaining checks from 16.2 before passing motion through
                out.linear.x = max(-limits[agent], min(limits[agent], twist.linear.x))
                out.angular.z = max(-w_max, min(w_max, twist.angular.z))
            pub.publish(out)

        state = SafetyState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.stop_latched = self.stop_latched
        state.motion_permitted = not self.stop_latched
        state.reason = self.reason
        self.safety_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
