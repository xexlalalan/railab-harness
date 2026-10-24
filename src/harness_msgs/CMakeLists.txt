cmake_minimum_required(VERSION 3.8)
project(harness_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/GripperStatus.msg"
  "msg/TagDetection.msg"
  "msg/TagArray.msg"
  "msg/FlaskLines.msg"
  "srv/SetGripper.srv"
  "srv/GripperCalibrate.srv"
  "srv/SetSpeed.srv"
  "srv/SetJoints.srv"
  "srv/MoveZ.srv"
  "srv/MoveV.srv"
  "srv/WeighTo.srv"
  "srv/DispenseVolume.srv"
  "srv/LiquidPush.srv"
  "srv/LiquidRun.srv"
  "srv/Barcode.srv"
  "srv/GantryJog.srv"
  "srv/Volume.srv"
  "srv/Lld.srv"
  "action/MoveJ.action"
  "action/MovePose.action"
  "action/FollowTrajectory.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces trajectory_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
