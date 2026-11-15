# Author: Mikhail Li (12236295)
"""Map M1 invariants (assignment 2-3)."""
from smart_hub import hub_map
from smart_hub.hub_map import edge_id, edge_length


def test_edge_id_is_undirected():
    assert edge_id('J', 'Cs') == edge_id('Cs', 'J') == 'Cs-J'


def test_reference_route_lengths():
    lower = edge_length('A', 'Cs') + edge_length('Cs', 'J') + edge_length('J', 'E')
    upper = sum(edge_length(a, b) for a, b in
                [('A', 'N'), ('N', 'Cn'), ('Cn', 'B'), ('B', 'U'), ('U', 'E')])
    assert lower == 10.0
    assert upper == 14.0


def test_crossing_edges():
    # Spec list (3.1) plus W-Cn: Cn itself lies inside C (decision #3 in ARCHITECTURE.md).
    expected = {edge_id(*e) for e in
                [('N', 'Cn'), ('Cn', 'B'), ('A', 'Cs'), ('Cs', 'J'), ('Cn', 'Cs'), ('W', 'Cn')]}
    assert set(hub_map.crossing_edges()) == expected


def test_robot_start_in_queue_zone():
    assert hub_map.point_in_rect(hub_map.NODES['R'], hub_map.QUEUE_Q)


def test_crossing_geometry_has_one_definition():
    """Managers, supervisor, arbiter, simulator and evaluator share radii and the C test."""
    from smart_hub import bus_manager, hub_coordinator, metrics, robot_manager
    from smart_hub import safety_supervisor, sim_world
    radius = hub_map.FOOTPRINT_RADIUS_M
    assert hub_coordinator.footprint_in_c is hub_map.footprint_in_crossing
    assert safety_supervisor.footprint_in_c is hub_map.footprint_in_crossing
    assert metrics.in_crossing is hub_map.footprint_in_crossing
    assert sim_world._footprint_in_c is hub_map.footprint_in_crossing
    assert robot_manager.RADIUS == sim_world.ROBOT_RADIUS == radius['robot']
    assert bus_manager.RADIUS == sim_world.VEHICLE_RADIUS == radius['bus'] == radius['car']
    assert safety_supervisor.RADIUS is radius
    assert metrics.AGENT_RADIUS == hub_coordinator.AGENT_RADIUS
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    y = (ymin + ymax) / 2
    for module, r in ((robot_manager, radius['robot']), (bus_manager, radius['bus'])):
        assert module.footprint_in_c(xmin - r + 0.01, y)
        assert not module.footprint_in_c(xmin - r - 0.01, y)
