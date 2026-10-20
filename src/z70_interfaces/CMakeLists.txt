cmake_minimum_required(VERSION 3.16)
project(z70_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EvidenceStamp.msg"
  "msg/Validity.msg"
  "msg/RobotState.msg"
  "msg/CameraState.msg"
  "msg/FrameMetadata.msg"
  "msg/ArmEstimate.msg"
  "msg/OdometryStatus.msg"
  "msg/RoomLabel.msg"
  "msg/MapSnapshot.msg"
  "msg/GarmentObservation.msg"
  "msg/GarmentObservations.msg"
  "msg/CommandOutcome.msg"
  "msg/CommandEvent.msg"
  "msg/CalibrationState.msg"
  "srv/SelectCamera.srv"
  "srv/RefreshMap.srv"
  "srv/SetArmAnchor.srv"
  "srv/Stop.srv"
  "srv/SetControlSession.srv"
  "srv/RawRpc.srv"
  "srv/TaskSession.srv"
  "action/BasePulse.action"
  "action/ArmStep.action"
  "action/HandStep.action"
  "action/ArmOperation.action"
  "action/DeployArm.action"
  "action/AlignToTarget.action"
  "action/MoveArm.action"
  "action/GraspAndLift.action"
  "action/ReleaseAndStow.action"
  "action/PinAndGo.action"
  "action/DeliverGarment.action"
  "action/PatrolCollect.action"
  "action/Pick.action"
  "action/Stack.action"
  DEPENDENCIES std_msgs builtin_interfaces unique_identifier_msgs sensor_msgs geometry_msgs nav_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
