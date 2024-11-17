cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/ChangeState.srv"
  "action/MoveRobot.action"
  DEPENDENCIES std_msgs
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  robot_control/nodes/robot_state_publisher.py
  robot_control/nodes/robot_monitor.py
  robot_control/nodes/state_controller.py
  robot_control/nodes/robot_mover.py
  robot_control/nodes/move_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
