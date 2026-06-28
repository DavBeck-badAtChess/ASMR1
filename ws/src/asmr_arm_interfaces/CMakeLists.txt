cmake_minimum_required(VERSION 3.16)
project(asmr_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeFK.srv"
  "srv/ComputeIK.srv"
  "srv/BSService.srv"
  "action/ExecuteTrajectory.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
