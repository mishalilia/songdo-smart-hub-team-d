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
| OS | Ubuntu 24.04.5 LTS |
| ROS 2 | Jazzy (rclpy 7.1.12, ros2launch 0.26.12, rosbag2 0.26.11) |
| Python | 3.12.3 |
| Python packages | numpy 1.26.4, PyYAML 6.0.1, OpenCV 4.6.0, scikit-learn 1.4.1 |

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
      sim_world.py  iot_devices.py               # simulator world, devices, energy
      tracker.py  demand_model.py  crossing_arbiter.py  dispatch_policy.py
      parking_engine.py  traffic_logic.py  route_engine.py  path_follower.py
      hub_map.py  qos.py  params.py  hub_events.py  metrics.py
      train_demand_model.py  scalability_benchmark.py  test_matrix.py
    launch/hub.launch.py
    config/map.yaml
    config/hub.yaml
    models/demand_ridge.json
    scenarios/S1.yaml ... S9.yaml, S4_PAIRS.yaml, S5_FREE.yaml, FAULTS.yaml,
              DEMO.yaml, SCALE.yaml, SANDBOX.yaml
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

# unpack the submitted source archive, then from its top-level folder (contains src/):
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

Launch arguments: `scenario` (S1..S9, DEMO, SCALE, SANDBOX, S4_PAIRS, S5_FREE, FAULTS), `seed`,
`mode` (`sim` | `hardware`; hardware is an explicit opt-in with startup motion disabled, no
simulator, and the UI's simulator injection rejected), `variant` (test variant listed in the
scenario file), `rviz` (`true`), `record` (`true`: rosbag of the 20.2 topics to
`evidence/bags/`), `dispatch_mode` (`predictive` | `reactive`), `ui_port` (default 8080; give
every parallel launch on one machine its own port).

Every node checks its parameters at startup against `smart_hub/params.py` and refuses to
start on invalid values; `metrics_logger` stores all nodes' parameter values per run
(`evidence/runs/<run_id>/params.json`). `/scenario/reset` returns after every stateful node
answered `RESET_ACK` on `/hub/events`.

## Midterm demo

```bash
ros2 launch smart_hub hub.launch.py scenario:=DEMO seed:=42 rviz:=true
```

Open **http://localhost:8080** (control UI) and **http://localhost:8080/dashboard** (metrics).
`DEMO` starts the same 14 nodes as every other scenario (the launch file has one node list;
the graph actually discovered is stored per run, see below). What differs is who moves the
vehicles: in `DEMO` the simulator drives the robot and the bus along scripted routes
(`agent_route`), boards passengers, operates the IoT devices, injects a pedestrian, an
obstacle, sensor faults and load. Scripted motion is a presentation aid and is not evidence
for the mission controllers: the assessed path is S7 / S8, where `robot_manager` and
`bus_manager` produce candidate commands and `safety_supervisor` owns the final ones.

| Professor's item | Where |
|------------------|-------|
| Simulation — robot | `scenario_runner` / `sim_world.py`: unicycle model, swept collisions, odometry and TF, energy; scripted routes (`agent_route`, UI "Drive (sim)"); autonomous missions come from `robot_manager` (M7) through `safety_supervisor` (M9) |
| Simulation — IoT devices | `iot_devices.py` in the simulator: barrier servo, bus door, parking LEDs, slot sensors, camera — `/iot/status`, `/bus/door_open`, `/parking/barrier_open` |
| Simulation — scalability | `ros2 run smart_hub scalability_benchmark` (simulator-core compute vs. placed load → `evidence/scalability.csv`) and `scenario:=SCALE` (whole graph with a crowd and extra cars) |
| UI (midterm) | `hub_ui` control page: live map, simulation controls, e-stop / clear stop, scenario reset, robot mission, parking, barrier |
| Dashboard (final) | `hub_ui` `/dashboard`: mission outcome, collisions, strict crossing measures, passenger waits, latency vs. targets, data age and rates, energy |
| Latency | `metrics_logger`: data age per topic (end-to-end), topic rates, `latency_ms` reported in events (UI → simulator, service round trips, and the other nodes' stop / grant / route timings) |
| Energy consumption | robot and bus model in `sim_world.py`, device model in `iot_devices.py`; `/sim/energy`, `energy.csv` |

Every run writes `evidence/runs/<run_id>/` with `events.csv`, `latency.csv`, `energy.csv`,
`crossing_occupancy.csv` (ground-truth occupant set of C at every change), `params.json` and
`summary.json`: run ID, scenario, seed, metrics, and the provenance of the run — source
commit and dirty flag (`SMART_HUB_COMMIT` / `SMART_HUB_DIRTY`, else `git`; the commit
identifies the source code), SHA-256 of `hub.yaml`, `map.yaml`, the demand model and the
scenario file, evaluator revision, ROS distribution, RMW, machine (CPU model, RAM, OS,
Python) — plus a graph audit taken 2 s into the run (discovered nodes,
publisher QoS and subscribers of the section 5 topics, measured rates).
In simulation mode the simulator plays the devices and applies the final motion commands;
`actuator_bridge` is the hardware layer. `/sim/inject` (`std_msgs/String`, JSON event as in
the scenario files) is a test-only input used by the UI; it is subscribed only by the
simulator, which hardware mode never starts, and `hub_ui` in hardware mode neither creates
the publisher nor accepts the command (`UI_SIM_REJECTED`).

## Energy and IoT models

Analytical teaching model with assumed parameters, not measured consumption (no current or
voltage was measured; J per run are comparable only at equal duration and completed service):
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
| `/parking/entrance` | `std_msgs/String` | simulated entrance sensor: id of the car waiting at the barrier |
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
`agent_route` (`agent`, `route`, `speed_mps`), `door`, `barrier` (`open`), `car_at_entrance`
(`client`; the parking manager opens the barrier only for a valid reservation).
Scripted bus / robot routes request the crossing lock from `hub_coordinator` like the
managers do and wait at the entry of C until it is granted.

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
# one run with a rosbag of the assignment 20.2 topics (evidence/bags/)
ros2 launch smart_hub hub.launch.py scenario:=S8 seed:=42 record:=true

# unit, lint and launch tests
colcon test --packages-select smart_hub
colcon test-result --verbose

# verification matrix T01-T18 with repeats -> evidence/test_matrix/
#   attempts.csv (every attempt, append-only), results.csv (last attempt per case),
#   summary.md, records/<attempt_id>/ (run records of every case that failed at least once)
ros2 run smart_hub run_test_matrix --parallel 4 --repeat 10
ros2 run smart_hub run_test_matrix --cases "T13|T14" --repeat 3      # subset

# demand model: generate 30 episodes, train, evaluate -> models/, evidence/demand/
ros2 run smart_hub train_demand_model --episodes 30

# simulator-core compute vs. placed load -> evidence/scalability.csv
ros2 run smart_hub scalability_benchmark

# S1 on a recorded video (camera path): live, and offline against manual counts.
# vtest.avi is not in the repository (see NOTICE); get it from OpenCV's samples and check it:
curl -L -o vtest.avi https://github.com/opencv/opencv/raw/4.x/samples/data/vtest.avi
sha256sum vtest.avi   # 45cddc9490be69345cbdab64ca583be65987e864ca408038e648db99e10516cf
ros2 launch smart_hub s1_video.launch.py video:=/path/to/vtest.avi
ros2 run smart_hub evaluate_s1_video --video /path/to/vtest.avi --truth evidence/s1_video/truth.csv \
    --split evidence/s1_video/split.json [--ablation]      # YOLOX vs HOG on frozen labels
ros2 launch smart_hub s1_video.launch.py video:=/path/to/vtest.avi detector:=hog   # live, HOG

# whole ROS graph under growing load -> evidence/graph_load.csv
ros2 run smart_hub graph_load_sweep
```

## Evidence rules and metric definitions

**Validity in time.** A retained message is permission only while it is valid and fresh by
both its source stamp and its local receipt time (`freshness.py`; future, out-of-order and
replayed stamps are rejected). `demand_predictor` needs 60 s of time-contiguous valid queue
history, erases it on an invalid message, and stamps the forecast with the acquisition time
of the newest supporting observation; `hub_coordinator` falls back to reactive dispatch when
the forecast, queue or vehicle state is rejected. Stale parking / crossing / queue / demand
state is shown as UNKNOWN by the simulator, RViz and the UI.

**Crossing C** (`metrics.CrossingMonitor`, ground truth, no grace interval, no speed
threshold; truth is sampled at 10 Hz, so times resolve to 100 ms). Reported separately:
strict occupancy (two or more users — robot, bus, car, pedestrian — physically inside C in
the same sample: episodes and seconds by pair class; one footprint test and one radius table
in `hub_map.py` serve the managers, the supervisor, the arbiter, the simulator and the
evaluator), unauthorized entry (`no_grant`: robot
or bus entering without owning a fresh `/crossing/state`; `pedestrian_present`: entering
while a pedestrian is inside; `car_against_owner`), token overlap, pedestrian response
latency (pedestrian enters C → every robot / bus inside is at ground-truth standstill;
unresolved cases are counted), and contact (`COLLISION`). The response is held to the stop
requirement of the assignment (16.3: zero command within 0.20 s), measured on the
supervisor clock together with the measured standstill; the 10 Hz ground truth resolves only
100 ms, so the harness's own pass threshold on it is a loose 1000 ms backstop, not a
grace interval: no occupancy is removed from the count.
`crossing_occupancy.csv` keeps every change of the occupant set. The midterm "conflict"
definition (1 s grace) survives only as a labelled diagnostic and is not a pass criterion.

**Stop times** are three different measures and are never mixed: (1) *zero command* —
`STOP_LATCHED latency_ms`, monotonic interval inside `safety_supervisor` from receipt of the
stop input to publication of the zero command; (2) *measured standstill* —
`STANDSTILL_MEASURED`, from the same receipt to receipt of the first pose observation below
the standstill speed (reported against the 0.20 s stop requirement; harness backstop
1000 ms; monotonic; `observer_delay_ms` = receipt − observation stamp is reported next to
it); (3) *route stop* —
`ROUTE_STOP_MEASURED`, from a `REROUTE` / `NO_ROUTE` event to ground-truth standstill
(budget 500 ms, 10 Hz truth); motion after `NO_ROUTE` is a `MOTION_WITHOUT_ROUTE` finding.
`summary.md` lists each with its clock domain.

**Routing** (S6 / S7). The planner cost is the sum of explicit terms per edge — length /
bounded observed speed, density penalty, expected crossing wait — and blocked, invalid or
missing edges are removed; the edge snapshot is rejected by receipt age and by observation
stamp. S6 calls `/route/compute` on the live graph before, during and after each blockage
(`ROUTE_REPLY`) and the checker recomputes continuity, permission and cost from `map.yaml`
with an oracle that shares no code with the planner. A replan passes only as the causal
chain blockage → `EDGE_BLOCKED` → `REROUTE` (old path, route generation) → measured
standstill → new route of a higher generation that avoids the edge.

**Passenger waits.** Every passenger is logged on entering the queue (`QUEUE_ENTERED`;
`QUEUE_BALKED` when the zone is full). Reported: wait of boarded passengers (mean, p50, p95,
n), the all-passenger lower bound in which abandoned and still-waiting passengers count with
their censored wait, completion rate (boarded / arrived) and queue clearance rate. Dispatch
policies are compared per seed: Δ = W_predictive − W_reactive for the same seed, with a
bootstrap that resamples whole paired episodes, not passenger rows, on two demand shapes: S2
(rising demand, 2 → 6 → 10 arrivals per minute, 9.4) and S8 (step surge at 60 s).

**Dispatch re-arm.** Rule 9.3.4 clears demand escalation only after five predictions below 4.
Read strictly, a queue that stays above 4 after a trip blocks every later request: in the
midterm S8 runs the bus made one trip per episode and boarded 4 of about 17 arrivals. With
`dispatch_rearm_after_service: true` (default) a mission that ended `DOCKED_AND_BOARDED` also
clears the escalation (`ESCALATION_CLEARED SERVED`); the next request still needs three new
consecutive predictions ≥ 6 (or the fallback rule), a free bus, no outstanding goal and the
30 s cooldown. `false` restores the strict reading.

**Demand model.** Errors are computed per episode; the model is compared with persistence by
a paired whole-episode bootstrap on the 6 test episodes and on all 30 unseen episodes (the 6
test episodes plus 24 further seeds; `evidence/demand/evaluation.json`). The test matrix
repeats this evaluation as the offline case T03.

**Camera path.** `passenger_detector` processes only the newest frame; a frame replaced
before processing is counted as dropped, and a frame older than `stale_timeout_sec` is expired
instead of processed. `CAMERA_FRAMES` reports received / processed / dropped / expired and
acquisition-to-update p95 / p99 every 10 s. The crossing is assessed independently of the
queue count: one foot-point inside C marks it occupied at once (no 0.4 s confirmation), while
the queue count keeps the 0.4 s confirmation and 0.5 s occlusion rules.
The real-video result (`evidence/s1_video/result.json`) is bound to its inputs by the SHA-256
of the clip and of the label file; the clip itself and images of people are not stored in the
repository, only our hand counts and the outputs. The image-to-map quadrilateral used for it
is an evaluation mapping, not a surveyed calibration.

**Detectors and ablation.** `camera_detector` (launch argument `detector`, node parameter
`camera_detector`): `yolox` — YOLOX-s CNN from the OpenCV Model Zoo
(`models/yolox_s_2022nov.onnx`, Apache-2.0 licence next to it, SHA-256 in every result) on a
320 × 320 input — is the default; `yolox640` is the same model on 640 × 640; `hog` is the
HOG baseline with thresholds chosen the same way; `hog_midterm` keeps the midterm
thresholds. The 108 labels were split in time before any change
(`evidence/s1_video/split.json`: first 54 frames for development, last 54 for test); score
threshold, NMS overlap, smallest box, foot-point height and (second round) input size were
chosen on the development part only and frozen in `ablation_plan.json` in a commit before
each test evaluation. The test part was used twice: once for the first-round
configurations, once for the single configuration chosen in the second round by a runtime
rule (fastest input whose development error is not worse); both uses are recorded.
`evaluate_s1_video --split … --ablation` reproduces `ablation.json`; the confirmation (0.4 s),
occlusion (0.5 s) and association (0.5 m) rules of the tracker are unchanged.

**Test matrix** (`run_test_matrix`, harness 2.6). Every attempt gets an ID
`<matrix>-<case>-<attempt>` and a row in `attempts.csv`, which is append-only across matrices
and harness versions. A failure is *infrastructure* (no run record, startup gate incomplete,
kill target missing, unreadable summary, graph audit mismatch) or *application* (the
checker's criterion is not met). Only infrastructure failures are retried, once; the failed
attempt stays in `attempts.csv` with `superseded_by`, and the run records of both attempts
are copied to `records/`. `summary.md` reports first-attempt and eventual totals separately.
Each run is bound to one source commit (`--commit`, or `SMART_HUB_COMMIT`, else `git`) and
carries the hashes listed above. The graph audit compares each run with the interface
table: the 14 nodes, exactly one publisher per section 5 topic with the specified QoS, and a
measured rate of at least 0.8 × nominal (and at most 1.25 × nominal for purely periodic
topics; `/traffic/edges` and `/parking/state` also publish on change). The launches of a
matrix use the UDP transport of Fast DDS (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` unless the
caller sets it): shared-memory segments left by earlier runs can fill `/dev/shm`, after which
nodes start with mixed transports and a subscriber may silently receive nothing. Each
running launch gets its own ROS domain from a pool of 30 (least recently used first), so
two cases never share a domain. T03 and T11 run inside the matrix as offline cases; the
dispatch policies are compared on 2 × `--repeat` paired seeds, the second half reported
separately as confirmation seeds. Every policy pair must show the same arrival attempts
(entered or balked, each within 0.25 s): the simulator draws arrivals from their own random
stream (common random numbers), so a seed fixes the passengers whatever the policy does.

**Scalability benchmark.** `cars_requested` / `pedestrians_requested` are what the load
event asks for; `cars_placed` / `pedestrians_placed` are what fits on free road and walkway
space, and only rows with `placement_complete` say anything about that load.
`world_step_compute_ratio` is the 50 ms step period divided by the mean compute time of one
simulator step: it is head-room of the simulator core, not whole-graph speed.

**Whole-graph load** (`graph_load_sweep`, `evidence/graph_load.csv`). The full launch runs
alone with scenario `LOAD` at five load levels (idle … saturated). Per level: placed
pedestrians and cars (`LOAD_SPAWNED`), CPU of every node process over a 40 s window
(`/proc/<pid>/stat`, % of one core; the busiest node is the executor-load bottleneck),
receipt age of the main topics (stamp → metrics_logger receipt, p95 / max: what a consumer
sees as backlog), measured rates and the graph audit.

## Results

Tested revision `a321fa5`; the submitted snapshot adds only the evidence of these runs and
documentation (no source file changes). Everything below — the matrix, `colcon test`
(140 tests, 0 failures), the retraining of the forecast, the whole-graph load sweep, the
simulator-core benchmark, the S1 video evaluation, the live camera path and the scenario clips —
was produced on that commit, two launches in parallel on one 8-CPU machine; the host load
during the matrix is logged every 5 minutes in `evidence/test_matrix/host_load.csv`.

| Matrix | Cases | First attempt | After the single re-run |
|--------|-------|---------------|-------------------------|
| `M20261113-150132` (all T01–T18, harness 2.6) | 176 | 174/176 | **176/176** |

The two first attempts that failed (T09 S5_FREE seed 5, T15 S3 pedestrian_in_crossing seed 3)
were infrastructure failures of the graph audit: `/hub/demand` (1 Hz) showed no message in the
short rate window right after discovery although `demand_predictor` was discovered with its
publisher and subscribers; both passed on the single permitted re-run. No application
criterion failed. `train_demand_model --episodes 30` on this revision reproduced the committed
model byte for byte. Every attempt and the records of every failed one are kept;
`attempts.csv` holds all twelve matrices since the midterm:

| Matrix | Harness | Commit | Cases | First attempt | Eventual | Notes |
|--------|---------|--------|-------|---------------|----------|-------|
| `M20261022-*` (three) | 1.0 / 1.1 | unknown | 95 + 95 + 11 | 82 / 93 | — | midterm; judged by the midterm criteria, restored with attempt IDs |
| `M20261111-233205` | 2.0 | `3773edf` | 95 | 93/95 | 94/95 | T05 seed 6 application failure: `/dev/shm` of the test container full → planner without edge states, bus safely at rest; fixed by UDP transport (2.1) |
| `M20261112-004912` | 2.1 | `eff8e52` | 95 | 95/95 | 95/95 | |
| `M20261112-054841` | 2.1 | `0523e7d` | 114 | 114/114 | 114/114 | |
| `M20261112-171643` | 2.3 | `eca7d17` | 116 | 116/116 | 116/116 | before the detector input change and the dispatch floor |
| `M20261113-043251` | 2.4 | `0162266` | 156 | 152/156 | 152/156 | 4 infrastructure failures: two launches shared a ROS domain (`index % 60` with longest-first order); fixed by a domain pool (2.5) |
| `M20261113-082431` / `-121230` | 2.5 | `b0f176a` | 156 / 30 | 148/156, 28/30 | 153/156, 30/30 | 3 cases failed the graph audit twice under load from another workload (load average 16–18); passed in a supplementary matrix on the same commit (`supplementary/`). Policy pairs did not see the same passengers (one random stream for arrivals and sensor noise) |
| `M20261113-150132` | 2.6 | `a321fa5` | 176 | 174/176 | 176/176 | final, above |

What changed between `b0f176a` and `a321fa5` (all found in the evidence of the earlier matrices):
arrivals in the simulator draw from their own random stream, so both policies on one seed see
the same passengers (the earlier pairs did not, which made two runs of identical code disagree);
the forecast was retrained on the regenerated episodes with the unchanged procedure;
`passenger_detector` publishes the invalid state within 50 ms of the 1 s freshness limit instead
of at the next 5 Hz publication (T02 had 1.13 s against 1.2 s); T15 has 20 cases of a
pedestrian stepping into C while the robot or the bus is inside it, because since the dispatch
floor the S8 bus has passed C before the scripted pedestrian; the harness checks that the
arrival attempts of every policy pair match.

| Item | Result (tested revision `a321fa5`) |
|------|--------|
| S1 queue counting (simulated camera) | stable counts 0 → 3 → 5 → 4; UNKNOWN 0.98 s after camera freeze (T02 limit 1.2 s; 1.0 s freshness limit plus at most 50 ms) |
| S1 on real video (offline, PETS 2009 view, 108 hand-annotated frames; `evidence/s1_video/`) | **YOLOX-s on a 320 input: test MAE 0.41 (54 held-out frames), 0.42 on all 108 (target ≤ 1 met)**; test exact 65 %, within ±1 96 %; partially occluded frames 0.67 (test, 6) / 0.64 (all 22); 6 frames reported empty with people present (midterm HOG: 36). Same test frames: YOLOX on 640 0.46, HOG with re-chosen thresholds 0.93, HOG with the midterm thresholds 1.11 (on the 22 occluded frames 1.59 with the current code; 1.77 as reviewed — the midterm code took HOG windows at OpenCV's default threshold 0, the current profiles at −0.5). The 320 input was chosen in a second round by a rule frozen before the test part was used again (`ablation_plan.json`). Samples are small: one video, 54 test frames. Detector time per frame: YOLOX-320 127 ms (p95 149, p99 156); YOLOX-640 about 4× slower |
| S1 live camera path (`live_camera.json`, 90 s) | YOLOX-320: 465 of 466 received frames processed (none dropped or expired), acquisition → tracker update p95 0.16 s, p99 0.18 s (1.0 s freshness limit). HOG: 94 %, 0.34 / 0.42 s. Before the input change YOLOX-640 processed 33–43 % with p99 0.88–0.94 s |
| S2 demand forecast (T03, offline case) | test MAE 1.77 vs persistence 2.07 (6 episodes, model better in 6); on all 30 unseen episodes (6 test + 24 further) paired Δ −0.28, 95 % interval −0.38 .. −0.18, model better in 27 of 30; inference p95 0.007 ms |
| S3 bus docking (10 runs) | position error 0.109 ± 0.017 m (max 0.135, limit 0.20), yaw 0.005 ± 0.055 rad (limit 0.17), ETA error at 5 m −0.85 ± 0.21 s; cancel → zero command 152 ms |
| S4 parking (20 simultaneous pairs) | 40 grants, 0 double allocations, service p95 22.4 ms |
| S5 congestion | CONGESTED 0.09 s, UNKNOWN 0.99 s, BLOCKED 0.09 s after the stimulus; precision 1.00, recall 1.00; 0 false labels in 10 free-flow runs |
| S6 routing on the live graph (T11 / T12) | baseline 28.6 s, detour 40.0 s, `NO_ROUTE` with both corridors blocked; robot with its active edge blocked: detect 92 ms, react 41 ms, standstill 69 ms, replan 399 ms, held at standstill; route computation p95 0.29 ms. Offline T11: planner = independent oracle on 20 seeded edge states |
| S7 robot escort (10 runs) | all arrived, arrival error 0.095 ± 0.002 m; in every run the active edge Cs–J is blocked and the route changes (R–W–Cn–Cs–J–D → … Cn–B–U–E–J–D); standstill 82 ± 41 ms after the replan decision (max 138, budget 500 ms) |
| S8 integrated hub (20 + 20 runs) | **Strict occupancy of C (any two users at once, ground truth, no grace): 0 in all 40 runs** — robot/bus + pedestrian 0, robot/bus + robot/bus 0, robot/bus + car 0, car + pedestrian 0, car + car 0; 0 contacts, 0 unauthorized entries, 0 token overlaps. The bus has passed C before the scripted pedestrian at 80 s, so 15.1 is tested by its own cases (next row) |
| Pedestrian steps into an occupied C (T15, 15.1; 10 + 10 runs) | robot inside C: pedestrian intrusion in all 10, measured standstill 98–102 ms, escort then completed (arrival error ≤ 0.099 m); bus inside C: intrusion in all 10, standstill 100–300 ms (ground truth at 10 Hz), docking then completed. Limit 1000 ms; 0 hub-caused episodes, 0 contacts |
| S9 / faults (e-stop, odometry gap, camera silence × 5 mission phases) | six e-stop runs (S9 and five phases): zero command 0.22–2.42 ms after e-stop receipt (requirement 200 ms), measured standstill 17–49 ms, observer delay 0.9–1.4 ms; reset rejected while a sensor is stale, accepted after recovery; every fault stops or aborts safely |
| Node kills (10 runs + planner) | zero output 0.53 ± 0.03 s after the kill of `robot_manager` or `safety_supervisor` (0.5 s watchdog plus one cycle); planner kill → `GOAL_ABORT PLANNER_UNAVAILABLE` |
| Dispatch service (re-arm after a served mission) | S8: 240 / 240 boarded in 20 episodes per policy (midterm: 40 in 10), completion 0.69 / 0.69 (midterm 0.24); all-passenger wait lower bound 134 / 134 s (midterm 165 s) |
| Dispatch policy, rising demand (S2, 20 paired seeds) | identical arrival attempts in 20 of 20 pairs. All passengers (lower bound): Δ = W_predictive − W_reactive +0.0 s, 95 % interval −0.7 .. +0.9, predictive lower in 14 of 20; boarded only +0.3 s (−0.4 .. +1.1). **No transport benefit of the forecast**: on identical episodes both policies dispatch within 1 s of each other in 19 of 20 seeds (the forecast was 7 s earlier in one, Δ −2.3 s there) |
| Dispatch policy, step surge (S8, 20 paired seeds) | identical arrival attempts in 20 of 20 pairs. All passengers Δ +0.3 s (−0.0 .. +0.7), boarded +0.1 s (−0.2 .. +0.7); first dispatch within 1 s in 19 of 20 seeds: the jump to seven people is observed, not forecast — a tie by construction |
| Energy (analytical model, not measured; equal duration, equal service) | S8 (299 s): bus 4387 / 4379 J, robot 1566 / 1564 J per run; bus + robot per boarded passenger 496 / 495 J (midterm, one trip: 1363 J). S2 (359 s): 612 / 611 J per boarded passenger |
| Whole-graph load (`evidence/graph_load.csv`, full launch alone) | up to `saturated` (400 pedestrians and 100 cars requested, 153 and 25 placed): all 14 nodes together 191 → 239 % of one core, busiest node `scenario_runner` ≤ 47 %; receipt age p95 ≤ 15 ms for `/hub/vehicles`, ≤ 8.2 ms for `/hub/safety`, ≤ 29 ms for `/camera/image_raw`; graph audit passed at every level except `idle`, where the 1 Hz `/hub/demand` showed no message in the rate window (the sweep has no re-run) |
| Simulator core (`evidence/scalability.csv`, world step only) | fully placed up to 15 cars + 60 pedestrians: step p95 0.34 ms, p99 0.37 ms. The assignment asks for one bus and one robot; 100 cars were requested only to find the placement limit of the map: 23 cars and 146 of 400 pedestrians fit — not a 100-car result |

**When does each policy win?** Neither, in the tested demand shapes. With the observed floor
the forecast can only make a dispatch earlier, never later. It did so in one S2 seed (7 s,
Δ −2.3 s) and by 1 s in one S8 seed. In the other 38 pairs the observed queue reached the
threshold within the same second as the forecast. The forecast fires early only when it
crosses 6 people before the queue does, and the model, fitted for MAE, under-predicts a rising
queue. Two limits would cap any benefit anyway: a bus boards at most four passengers per stop
(8–12 of 17–24 arrivals board in an S2 episode under either policy), and under sustained demand both
policies redispatch as soon as the bus returns, so only the first dispatch can move. The forecast
is more accurate than persistence (T03); that accuracy does not translate into lower waits here.

Repeated runs, mean ± standard deviation (n runs): S3 position error 0.109 ± 0.017 m, yaw
0.005 ± 0.055 rad, ETA error at 5 m −0.85 ± 0.21 s (10); S7 arrival error 0.095 ± 0.002 m,
standstill after the replan decision 82 ± 41 ms (10); measured standstill after e-stop
36.4 ± 11.1 ms (6); zero output after a process kill 0.53 ± 0.03 s (10). All-passenger wait
(lower bound) per episode: S2 126.4 ± 11.3 s predictive, 126.4 ± 10.9 s reactive; S8
134.1 ± 13.7 s, 133.8 ± 13.2 s; completion rate S2 0.54 ± 0.05 / 0.53 ± 0.05, S8
0.69 ± 0.06 / 0.69 ± 0.06 (20 each). Individual run values are in `results.csv`.

The harness (since 2.2) classifies every robot/bus + pedestrian episode in C from the raw trace
(`crossing_occupancy.csv`): a *pedestrian intrusion* when the robot / bus already owned and
occupied C and no pedestrian was inside in the sample before, otherwise *hub-caused*, which must
be zero. Intrusions stay in the strict-occupancy count, are not erased by any grace interval,
and are judged by the response: measured standstill within 1000 ms. A T15
pedestrian_in_crossing case passes only if an intrusion actually happened, the standstill was
measured, and the mission completed afterwards.

Not covered: a surveyed calibration of a real queue scene (the PETS mapping is an evaluation
mapping; surveyed floor points need a physical site) and measured energy (the energy model is
analytical; measurement needs a robot with current / voltage sensing). Both stay as stated
limitations.

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
| M1 | Andrey Li | `passenger_detector`, `tracker` — detection, tracking, homography, queue/crossing occupancy | S1; T01, T02 | M5 state validity |
| M2 | Hsu Shwe Sin Htun | `demand_predictor` — 30 episodes, features, persistence baseline, ridge model, data split | S2; T03 | M6 state validity |
| M3 | Komronbek Tohirov | `hub_coordinator`, `crossing_arbiter`, `dispatch_policy` — crossing mutex `/crossing/request`, demand dispatch, S8 integration | S2 dispatch, S8; T04, T15 | M7 cancellation |
| M4 | Vadim Lifrov | `bus_manager` — DockBus action, ETA, docking, door interlock | S3; T05, T06 | M9 stop latch / interlocks |
| M5 | Nikita Pak | `parking_manager`, `parking_engine` — reservations, expiry, barrier requests | S4; T07, T08 | M1 data labels |
| M6 | Vladislav Li | `traffic_monitor`, `traffic_logic`, `route_planner`, `route_engine` — edge states, Dijkstra, replanning | S5, S6; T09–T12 | M2 data labels |
| M7 | Timur Klechshev | `robot_manager`, `path_follower` — EscortPassenger and NavigateTo actions, `/robot/confirm_ready` | S7; T13, T14 | M3 crossing logic |
| M8 | Mikhail Li | `scenario_runner` / `sim_world` (2D simulator, IoT device and energy models), DEMO / SCALE / SANDBOX scenarios, launch, config, `hub_ui` (UI + dashboard), `metrics_logger`, `dashboard_marker_publisher`, parameter checks, scalability benchmark, test matrix runner | simulator events for S1–S9, `/scenario/reset`, seeded runs, metrics, T01–T18 runner | M4 integrated failures |
| M9 | Maftuna Khalimjonova | `safety_supervisor`, `actuator_bridge` (hardware layer) — watchdogs, e-stop latch, reset | S9, FAULTS; T16–T18 | M8 integrated failures |

Every member writes tests for their own subsystem and reviews another subsystem (23.2).
The first line of every source file names its author.

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
