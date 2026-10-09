cmake_minimum_required(VERSION 3.8)
project(hwr_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelSpeeds.msg"
  "msg/ServoCommand.msg"
  "msg/LocalizationHealth.msg"
  "action/NavigateToPose.action"
  "srv/ArmPose.srv"
  "srv/ArmReach.srv"
  "srv/AimCamera.srv"
  "srv/Gripper.srv"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)

ament_package()
