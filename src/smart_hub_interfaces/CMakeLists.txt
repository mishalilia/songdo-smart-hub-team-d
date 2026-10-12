cmake_minimum_required(VERSION 3.8)
project(smart_hub_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QueueState.msg"
  "msg/DemandPrediction.msg"
  "msg/VehicleState.msg"
  "msg/VehicleStateArray.msg"
  "msg/BusArrival.msg"
  "msg/SlotObservation.msg"
  "msg/SlotObservationArray.msg"
  "msg/ParkingSlot.msg"
  "msg/ParkingState.msg"
  "msg/EdgeState.msg"
  "msg/EdgeStateArray.msg"
  "msg/CrossingState.msg"
  "msg/SafetyState.msg"
  "msg/HubEvent.msg"
  "srv/ReserveParking.srv"
  "srv/ReleaseParking.srv"
  "srv/ComputeRoute.srv"
  "srv/RequestCrossing.srv"
  "srv/ResetScenario.srv"
  "srv/ClearSafetyStop.srv"
  "action/DockBus.action"
  "action/EscortPassenger.action"
  "action/NavigateTo.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
