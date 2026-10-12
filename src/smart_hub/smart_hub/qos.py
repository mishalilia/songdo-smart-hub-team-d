"""
QoS profiles from assignment section 5.

R1  = reliable, volatile, keep-last 1
R10 = reliable, volatile, keep-last 10
R1T = reliable, transient-local, keep-last 1
R100 = reliable, volatile, keep-last 100 (events)
SENSOR = ROS sensor-data profile (camera)

Reliable delivery is not a freshness guarantee: always check message age.
"""
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSPresetProfiles,
    QoSProfile,
    ReliabilityPolicy,
)


def _reliable(depth, durability=DurabilityPolicy.VOLATILE):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=durability,
    )


R1 = _reliable(1)
R10 = _reliable(10)
R100 = _reliable(100)
R1T = _reliable(1, DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSPresetProfiles.SENSOR_DATA.value
