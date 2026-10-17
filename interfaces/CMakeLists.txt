cmake_minimum_required(VERSION 3.8)
project(interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PredictionPoint.msg"
  "msg/PredictedTrajectory.msg"
  "msg/RobotPrediction.msg"
  "msg/Predictions.msg"
  "msg/CameraObservation.msg"
  "msg/PerceptionHealth.msg"
  "msg/DockState.msg"
  "msg/DockStates.msg"
  "msg/DockLandmark.msg"
  "msg/DockMap.msg"
  "msg/TrajectoryPoint.msg"
  "msg/PlannedTrajectory.msg"
  "msg/SagaState.msg"
  "msg/CoordinatorRequest.msg"
  "msg/CoordinatorStatus.msg"
  "msg/UnderstandingRegion.msg"
  "msg/Understanding.msg"
  "msg/RobotTruth.msg"
  "msg/RobotTruthArray.msg"
  "msg/CollisionEvent.msg"
  "msg/SimRouteEvent.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(contracts test/test_contracts.py)
endif()
ament_export_dependencies(rosidl_default_runtime)
ament_package()
