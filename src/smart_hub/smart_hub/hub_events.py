# Author: Mikhail Li (12236295)
"""
Small helpers for /hub/events shared by all nodes.

RESET handshake (assignment 7.1): scenario_runner publishes RESET with the new run_id as
correlation_id; every stateful node clears its run state and answers RESET_ACK with the
same correlation_id, so /scenario/reset can return after all nodes acknowledged.
"""
from smart_hub_interfaces.msg import HubEvent


def ack_reset(node, publisher, reset_msg):
    """Publish RESET_ACK for a RESET event (no-op for other events)."""
    if reset_msg.event != 'RESET':
        return
    ack = HubEvent()
    ack.header.stamp = node.get_clock().now().to_msg()
    ack.run_id = reset_msg.correlation_id
    ack.entity_id = node.get_name()
    ack.event = 'RESET_ACK'
    ack.correlation_id = reset_msg.correlation_id
    publisher.publish(ack)
