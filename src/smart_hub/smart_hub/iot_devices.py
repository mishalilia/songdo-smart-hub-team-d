"""
Simulated IoT devices of the hub (assignment 11.3, 16.2, 22). Owner: M8.

Barrier servo, bus door actuator, parking LEDs, slot sensors and camera, each with a
simple timing model, the device-level interlocks the assignment requires and a power model
(teaching assumptions, not measured hardware). Pure Python; used by sim_world, which plays
the physical devices in simulation mode (actuator_bridge drives real ones in hardware mode).
"""
from dataclasses import dataclass

LED_COLOURS = {'FREE': 'GREEN', 'RESERVED': 'AMBER', 'OCCUPIED': 'RED', 'UNKNOWN': 'AMBER'}

POWER_W = {
    'servo_moving': 2.0, 'servo_holding': 0.3,
    'door_moving': 3.0, 'door_holding': 0.2,
    'led_on': 0.2,
    'slot_sensor': 0.1,
    'camera': 2.5,
}


@dataclass
class Device:
    name: str
    power_w: float = 0.0
    energy_j: float = 0.0

    def integrate(self, dt):
        self.energy_j += self.power_w * dt


class Barrier(Device):
    """Servo barrier 0 deg (closed) .. 90 deg (open). Never closes on an occupied passage."""

    def __init__(self, speed_deg_s=90.0):
        super().__init__('barrier')
        self.angle = 0.0
        self.target = 0.0
        self.speed = speed_deg_s
        self.passage_occupied = False
        self.reason = ''

    def request(self, open_):
        if not open_ and self.passage_occupied:
            self.reason = 'CLOSE_REFUSED_PASSAGE_OCCUPIED'
            return False
        self.target = 90.0 if open_ else 0.0
        self.reason = ''
        return True

    def step(self, dt):
        if not self.passage_occupied or self.target > self.angle:
            delta = max(-self.speed * dt, min(self.speed * dt, self.target - self.angle))
            self.angle += delta
        moving = abs(self.target - self.angle) > 1e-6
        self.power_w = POWER_W['servo_moving'] if moving else POWER_W['servo_holding']
        self.integrate(dt)

    @property
    def is_open(self):
        return self.angle >= 90.0 - 1e-6

    @property
    def state(self):
        if self.is_open:
            return 'OPEN'
        return 'CLOSED' if self.angle <= 1e-6 else 'MOVING'


class Door(Device):
    """Bus door: opens only while the bus is measured stopped; 1 s travel."""

    def __init__(self, travel_sec=1.0):
        super().__init__('bus_door')
        self.position = 0.0          # 0 closed .. 1 open
        self.target = 0.0
        self.travel_sec = travel_sec
        self.reason = ''

    def request(self, open_, bus_stopped):
        if open_ and not bus_stopped:
            self.reason = 'OPEN_REFUSED_BUS_MOVING'
            return False
        self.target = 1.0 if open_ else 0.0
        self.reason = ''
        return True

    def step(self, dt):
        rate = dt / self.travel_sec
        self.position += max(-rate, min(rate, self.target - self.position))
        moving = abs(self.target - self.position) > 1e-6
        self.power_w = POWER_W['door_moving'] if moving else POWER_W['door_holding']
        self.integrate(dt)

    @property
    def is_open(self):
        # Reported open as soon as it is not fully closed: propulsion stays inhibited.
        return self.position > 1e-6


class Led(Device):
    """Slot LED: green FREE, amber RESERVED, red OCCUPIED, blinking amber UNKNOWN."""

    def __init__(self, slot_id):
        super().__init__('led/' + slot_id)
        self.slot_state = 'UNKNOWN'
        self.lit = True

    def step(self, dt, now):
        blink = self.slot_state not in ('FREE', 'RESERVED', 'OCCUPIED')
        self.lit = not blink or int(now * 2) % 2 == 0
        self.power_w = POWER_W['led_on'] if self.lit else 0.0
        self.integrate(dt)

    @property
    def colour(self):
        return LED_COLOURS.get(self.slot_state, 'AMBER') if self.lit else 'OFF'


class Sensor(Device):
    """Always-on sensor (slot sensor, camera): draws power while online."""

    def __init__(self, name, power_key):
        super().__init__(name)
        self.power_key = power_key
        self.online = True

    def step(self, dt):
        self.power_w = POWER_W[self.power_key] if self.online else 0.0
        self.integrate(dt)
