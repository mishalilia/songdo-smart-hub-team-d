# Songdo Smart Hub — Team D

Smart Mobility Engineering Lab, Inha University (ISE) — ROS 2 + Python project assignment.
Instructor: Prof. Mehdi Pirahandeh.

Miniature Songdo transportation station: passenger detection, demand prediction, bus docking,
parking, congestion detection, route replanning and an assistance robot. Eight functional
scenarios S1–S8, safety/fault suite S9, verification tests T01–T18.

> Educational model, not a road-ready autonomous transport system.

## Platform

| Item | Version |
|------|---------|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Python | _record exact installed version_ |
| ROS packages | _record exact installed versions_ |

## Workspace

```
smart_hub_ws/src/
  smart_hub_interfaces/        # ament_cmake; msg/, srv/, action/
  smart_hub/                   # ament_python; application nodes
    smart_hub/
      passenger_detector.py  demand_predictor.py
      bus_manager.py         parking_manager.py
      traffic_monitor.py     route_planner.py
      robot_manager.py       hub_coordinator.py
      safety_supervisor.py   scenario_runner.py
      metrics_logger.py      actuator_bridge.py
      dashboard_marker_publisher.py
      hub_ui.py              # web UI / dashboard
    launch/hub.launch.py
    config/map.yaml
    config/hub.yaml
    scenarios/S1.yaml ... S9.yaml, DEMO.yaml, SCALE.yaml
    web/                       # operator UI + metrics dashboard (served by hub_ui)
    tests/
    package.xml  setup.py  setup.cfg
evidence/                      # videos, bags, metric CSVs, model, data (assignment 24.1)
```

## Build and run

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-numpy python3-sklearn python3-opencv \
  ros-jazzy-rviz2 ros-jazzy-ros2bag ros-jazzy-rosbag2-storage-default-plugins

git clone https://github.com/mishalilia/songdo-smart-hub-team-d.git smart_hub_ws
cd smart_hub_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=44
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch smart_hub hub.launch.py scenario:=S8 seed:=42 mode:=sim
```

`ROS_DOMAIN_ID` is team-specific so neighbouring groups do not receive each other's commands
(assignment 20.1). Team D uses **44** — set it in every terminal.

Launch arguments: `scenario` (S1..S9), `seed`, `mode` (`sim` | `hardware`; hardware is an
explicit opt-in with startup motion disabled and no simulator), `rviz` (`true` | `false`).

## Midterm demo

```bash
ros2 launch smart_hub hub.launch.py scenario:=DEMO seed:=42 rviz:=true
```

Open **http://localhost:8080** (control UI) and **http://localhost:8080/dashboard** (metrics).
`DEMO` runs without the mission and safety nodes: the simulator drives the robot and the bus
along scripted routes, boards passengers, operates the IoT devices, injects a pedestrian,
an obstacle, sensor faults and load.

| Professor's item | Where |
|------------------|-------|
| Simulation — robot | `scenario_runner` / `sim_world.py`: unicycle model, swept collisions, odometry and TF, energy; scripted routes (`agent_route`, UI "Drive (sim)"); autonomous missions come from `robot_manager` (M7) through `safety_supervisor` (M9) |
| Simulation — IoT devices | `iot_devices.py` in the simulator: barrier servo, bus door, parking LEDs, slot sensors, camera — `/iot/status`, `/bus/door_open`, `/parking/barrier_open` |
| Simulation — scalability | `ros2 run smart_hub scalability_benchmark` (compute vs. load → `evidence/scalability.csv`) and `scenario:=SCALE` (whole graph under 100+ pedestrians and cars) |
| UI (midterm) | `hub_ui` control page: live map, simulation controls, e-stop / clear stop, scenario reset, robot mission, parking, barrier |
| Dashboard (final) | `hub_ui` `/dashboard`: mission outcome, collisions, crossing conflicts, latency vs. targets, data age and rates, energy |
| Latency | `metrics_logger`: data age per topic (end-to-end), topic rates, `latency_ms` reported in events (UI → simulator, service round trips, and the other nodes' stop / grant / route timings) |
| Energy consumption | robot and bus model in `sim_world.py`, device model in `iot_devices.py`; `/sim/energy`, `energy.csv` |

Every run writes `evidence/runs/<run_id>/` with `events.csv`, `latency.csv`, `energy.csv`,
`summary.json` (run ID, scenario, seed, commit, config hash, machine, metrics).
In simulation mode the simulator plays the devices and applies the final motion commands;
`actuator_bridge` is the hardware layer. `/sim/inject` (`std_msgs/String`, JSON event as in
the scenario files) is a test-only input used by the UI.

## Energy and IoT models

Teaching assumptions, not measured hardware:
`P = P_idle + (C_rr·m·g·|v| + max(0, m·a·v)) / η + c_turn·|ω|` — robot m = 8 kg, P_idle = 5 W;
bus m = 25 kg, P_idle = 12 W; C_rr = 0.03, η = 0.7.
IoT: servo 2.0 W moving / 0.3 W holding, door 3.0 / 0.2 W, LED 0.2 W lit, slot sensor 0.1 W,
camera 2.5 W. The barrier never closes on an occupied passage; the door opens only while the
bus is measured stopped; UNKNOWN slots blink amber.

## Simulator (scenario_runner)

Deterministic 2D world (`smart_hub/sim_world.py`, pure Python) stepped at 20 Hz with a fixed
seed (assignment 3.2). Scenario files `scenarios/S1..S9.yaml` list timed events.

| Topic | Type | Notes |
|-------|------|-------|
| `/camera/image_raw` | `sensor_msgs/Image` | 10 Hz, top-down mono8, 10 px/m; silent while the camera is frozen |
| `/sim/pedestrians` | `geometry_msgs/PoseArray` | 10 Hz, noisy pedestrian foot-points in `map` (synthetic detections, no IDs) |
| `/hub/vehicles` | `VehicleStateArray` | 10 Hz, `bus1`, `robot1`, cars; obstacles have `kind: obstacle` |
| `/parking/observed` | `SlotObservationArray` | 5 Hz; a disconnected sensor is omitted |
| `/robot/odom` | `nav_msgs/Odometry` | 20 Hz; TF `map -> robot/odom -> robot/base_link` (same for `bus/`) |
| `/hub/estop` | `std_msgs/Bool` | 10 Hz, driven by `estop` events |
| `/sim/energy` | `diagnostic_msgs/DiagnosticArray` | 1 Hz, robot / bus / device energy model |
| `/iot/status` | `diagnostic_msgs/DiagnosticArray` | 2 Hz, barrier, door, LEDs, slot sensors, camera |
| `/bus/door_open`, `/parking/barrier_open` | `std_msgs/Bool` | measured device status, on change + 2 Hz |
| `/sim/truth/vehicles`, `/sim/truth/pedestrians` | | private ground truth — metrics only |

Inputs: final `/robot/cmd_vel`, `/bus/cmd_vel` (zero after 0.5 s without a command),
`/bus/door_request` and `/parking/barrier_request` (simulated devices; the bus boards up to
4 queue passengers over 8 s while docked with the door open), `/parking/state` (LEDs),
`/hub/estop` (true cuts propulsion power, no automatic restart).
`/scenario/reset` reloads a scenario with a new seed and returns a new `run_id`.
The scenario clock starts (`RUN_START`) once the services and actions it drives are up.
Waiting passengers stand beside the robot lane R–W, at least 0.6 m from it.

Event types: `spawn_queue_tracks`, `remove_queue_tracks`, `fill_queue`, `arrival_rate`,
`crossing_pedestrian`, `occlude`, `freeze_camera`, `freeze_odometry`, `freeze_vehicle_source`,
`restore_sensors`, `place_obstacle`, `remove_obstacle`, `block_edges`, `restore_all`,
`spawn_car`, `spawn_slow_vehicles`, `set_slot`, `disconnect_slot_sensor`,
`reconnect_slot_sensor`, `occupy_reserved_slot`, `estop`, and test-driver calls `reserve`,
`release`, `dock_request`, `escort_request`, `confirm_ready`, `clear_stop_request`,
`cancel_goal`.
Scalability load: `spawn_random_cars`, `spawn_crowd`. Scripted motion and devices:
`agent_route` (`agent`, `route`, `speed_mps`), `door`, `barrier` (`open`).
Scripted agents do not use the crossing lock.

RViz: `ros2 launch smart_hub hub.launch.py scenario:=S5 rviz:=true`.

## Inspect

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /hub/queue --verbose
ros2 topic hz /hub/queue
ros2 service list -t
ros2 action list -t
ros2 interface show smart_hub_interfaces/action/EscortPassenger
ros2 service call /route/compute smart_hub_interfaces/srv/ComputeRoute \
  "{start_node: 'R', goal_node: 'D', agent_type: 'robot', excluded_edges: []}"
ros2 action send_goal /robot/escort smart_hub_interfaces/action/EscortPassenger \
  "{request_id: 'demo-01', passenger_id: 'P01', pickup_node: 'R', destination_node: 'D',
    max_duration_sec: 240.0}" --feedback
```

## Record and test

```bash
ros2 bag record -o bags/S8_seed42 \
  /hub/queue /hub/demand /hub/vehicles /hub/bus_arrival \
  /parking/state /traffic/edges /crossing/state \
  /robot/odom /robot/cmd_vel_candidate /robot/cmd_vel \
  /bus/cmd_vel_candidate /bus/cmd_vel /hub/estop \
  /hub/safety /hub/events /tf /tf_static

colcon test --packages-select smart_hub
colcon test-result --verbose
```

## Units and invalid-value conventions

- Frame `map`; origin south-west corner, +x east, +y north, +z up; metres, radians, seconds, m/s.
- `QueueState`: `confidence` in [0, 1]; low-confidence or stale (> 1.0 s) ⇒ `valid=false` (UNKNOWN,
  not zero people). A zero count requires a valid observation.
- `ParkingSlot.state`: `FREE`, `RESERVED`, `OCCUPIED`, `UNKNOWN`. Only fresh `FREE` slots are
  allocatable; reservation expiry never turns a physically occupied slot `FREE`.
- `EdgeState.density_ratio` = min(1, n_e / K_e). Invalid speed never becomes zero travel time;
  stale edge ⇒ `valid=false`.
- `BusArrival`: `eta_sec = -1` when `eta_valid = false`.
- `HubEvent`: transitions only; `correlation_id` = reservation token or action goal ID.
- Crossing edges (need ownership of C): A–Cs, Cs–J, N–Cn, Cn–B, Cn–Cs and W–Cn, found by
  segment–rectangle intersection (assignment 3.1). W–Cn is included because node Cn lies inside C.

## Team

| ID | Member | Nodes / files | Scenarios, tests | Reviews |
|----|--------|---------------|------------------|---------|
| M1 | Andrey Li | `passenger_detector` — detection, tracking, homography, queue/crossing occupancy | S1; T01, T02 | M5 state validity |
| M2 | Hsu Shwe Sin Htun | `demand_predictor` — 30 episodes, features, persistence baseline, ridge model, data split | S2; T03 | M6 state validity |
| M3 | Komronbek Tohirov | `hub_coordinator` — crossing mutex `/crossing/request`, demand dispatch, S8 integration | S2 dispatch, S8; T04, T15 | M7 cancellation |
| M4 | Vadim Lifrov | `bus_manager` — DockBus action, ETA, docking, door interlock | S3; T05, T06 | M9 stop latch / interlocks |
| M5 | Nikita Pak | `parking_manager`, `parking_engine` — reservations, expiry, barrier requests | S4; T07, T08 | M1 data labels |
| M6 | Vladislav Li | `traffic_monitor`, `route_planner`, `route_engine` — edge states, Dijkstra, replanning | S5, S6; T09–T12 | M2 data labels |
| M7 | Timur Klechshev | `robot_manager` — EscortPassenger and NavigateTo actions, `/robot/confirm_ready` | S7; T13, T14 | M3 crossing logic |
| M8 | Mikhail Li | `scenario_runner` / `sim_world` (2D simulator, IoT device and energy models), scenario YAMLs, launch, `hub_ui` (UI + dashboard), `metrics_logger`, `dashboard_marker_publisher`, scalability benchmark | S1–S9 stimuli, `/scenario/reset`, seeded runs, metrics | M4 integrated failures |
| M9 | Maftuna Khalimjonova | `safety_supervisor`, `actuator_bridge` (hardware layer) — watchdogs, e-stop latch, reset | S9; T16–T18 | M8 integrated failures |

Every member writes tests for their own subsystem and reviews another subsystem (23.2).
Weekly issues record owner, estimate, acceptance evidence and reviewer. `TODO(Mn)` in the
code marks each member's work.

## Schedule

| Week | Required work | Exit evidence |
|------|---------------|---------------|
| 1 | Freeze map, interfaces, ownership and safety policy; packages and basic simulator | Map file, ROS graph, build log, queue topic, zero-motion startup |
| 2 | Queue perception and parking state/services; unit tests | S1 deterministic counts; S4 reservation concurrency proof |
| 3 | Forecast baseline/model and bus docking | Held-out data split; S2 comparison; S3 action feedback/cancel |
| 4 | Congestion monitor, graph planner, robot escort | S5 edge labels; oracle-tested S6 routes; S7 nominal/negative tests |
| 5 | Crossing arbiter, safety, dashboard, all nodes | S8 full pipeline; first S9 fault-injection report |
| 6 | Repeat experiments; improve weak cases; physical adapter if available | T01–T18 matrix; reproducible bags; measured hardware limits |
| 7 | Freeze code; final demonstration; submit and defend | Report, source, models/data, run instructions, individual defense |
