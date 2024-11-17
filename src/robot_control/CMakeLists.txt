cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_ISGNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
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

install(PROGRAMS
  scripts/robot_state_publisher
  scripts/robot_monitor
  scripts/state_controller
  scripts/robot_mover
  scripts/move_commander
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
