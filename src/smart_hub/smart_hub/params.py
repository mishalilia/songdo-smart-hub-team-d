# Author: Mikhail Li (12236295)
"""
Parameter range checks shared by all nodes (assignment 17).

Every node calls check_parameters(self) after declaring its parameters: values outside the
documented ranges are rejected at startup (the node logs every problem and refuses to
start). metrics_logger records the final parameter values of all nodes with every run.
"""
RANGES = {
    # perception / prediction
    'queue_confidence_min': (0.0, 1.0),
    'confidence_min': (0.0, 1.0),
    'prediction_horizon_sec': (1, 300),
    'history_window_sec': (31, 600),
    'stale_timeout_sec': (0.1, 10.0),
    'crossing_clear_sec': (0.0, 30.0),
    # dispatch / crossing
    'dispatch_high': (0.0, 100.0),
    'dispatch_low': (0.0, 100.0),
    'dispatch_cooldown_sec': (0.0, 600.0),
    'crossing_lease_sec': (0.2, 30.0),
    # parking
    'reservation_ttl_sec': (5.0, 120.0),
    'debounce_sec': (0.0, 5.0),
    'barrier_open_sec': (1.0, 60.0),
    # traffic / routing
    'congestion_density_threshold': (0.0, 1.0),
    'slow_speed_ratio': (0.0, 1.0),
    'window_sec': (0.5, 60.0),
    'headway_m': (0.1, 10.0),
    'replan_improvement_ratio': (0.0, 1.0),
    'replan_period_sec': (0.5, 60.0),
    'edge_states_timeout_sec': (0.1, 10.0),
    # motion and safety
    'robot_speed_limit': (0.01, 0.5),
    'bus_speed_limit': (0.01, 1.0),
    'max_angular_rad_s': (0.05, 3.0),
    'command_timeout_sec': (0.05, 2.0),
    'pose_timeout_sec': (0.1, 5.0),
    'clearance_m': (0.0, 2.0),
    'reaction_time_sec': (0.0, 2.0),
    'min_deceleration_mps2': (0.05, 10.0),
    'arrival_tolerance_m': (0.01, 1.0),
    'ready_timeout_sec': (1.0, 300.0),
    'pose_tolerance_m': (0.01, 1.0),
    'yaw_tolerance_rad': (0.01, 1.0),
    'boarding_dwell_sec': (1.0, 120.0),
    # simulator / tools
    'pedestrian_noise_m': (0.0, 1.0),
    'vehicle_noise_m': (0.0, 1.0),
    'startup_timeout_sec': (0.0, 120.0),
    'reset_ack_timeout_sec': (0.1, 10.0),
    'port': (1, 65535),
}
CHOICES = {
    'mode': ('sim', 'hardware'),
    'dispatch_mode': ('predictive', 'reactive'),
}


def problems(values):
    """Return a list of human-readable problems for a {name: value} mapping."""
    out = []
    for name, value in sorted(values.items()):
        if name in RANGES and isinstance(value, (int, float)) and not isinstance(value, bool):
            lo, hi = RANGES[name]
            if not lo <= value <= hi:
                out.append('%s=%s outside [%s, %s]' % (name, value, lo, hi))
        if name in CHOICES and value not in CHOICES[name]:
            out.append('%s=%r not in %s' % (name, value, CHOICES[name]))
    if 'dispatch_low' in values and 'dispatch_high' in values and \
            values['dispatch_low'] >= values['dispatch_high']:
        out.append('dispatch_low must be < dispatch_high (hysteresis)')
    return out


def check_parameters(node):
    """Reject invalid parameter values at startup (raises ValueError)."""
    names = node.list_parameters([], 0).names
    values = {p.name: p.value for p in node.get_parameters(names)}
    found = problems(values)
    for p in found:
        node.get_logger().error('Invalid parameter: ' + p)
    if found:
        raise ValueError('%s: invalid parameters: %s' % (node.get_name(), '; '.join(found)))
    return values
