# ros2_ws2
robot operating system workspace 2

## Test 1

```sh
# Create the workspace and package
mkdir -p ros2_ws/src
cd ros2_ws/src
ros2 pkg create robot_control --build-type ament_cmake --dependencies rclpy std_msgs

# Create necessary directories
cd robot_control
mkdir msg srv action scripts launch

# After adding all the files, build the workspace
cd ../../
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run the nodes (in separate terminals, after sourcing in each)
ros2 run robot_control robot_state_publisher.py
ros2 run robot_control robot_monitor.py
ros2 run robot_control state_controller.py
ros2 run robot_control robot_mover.py
ros2 run robot_control move_commander.py

# Alternative: use launch file
ros2 launch robot_control robot_system.launch.py

# Useful commands for testing
ros2 topic list
ros2 topic echo /robot_status
ros2 service call /change_state robot_control/srv/ChangeState "{new_state: 'MOVING'}"
ros2 action list
ros2 action send_goal /move_robot robot_control/action/MoveRobot "{target_position: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

## Test 2

```sh
mkdir -p ros2_interview_ws/src
cd ros2_interview_ws/src

# Create the interfaces package (for all custom msgs/srvs/actions)
ros2 pkg create --build-type ament_cmake interview_interfaces

# Create Python packages that will use the interfaces
ros2 pkg create --build-type ament_python sensor_system --dependencies rclpy geometry_msgs interview_interfaces
ros2 pkg create --build-type ament_python diagnostic_system --dependencies rclpy interview_interfaces
ros2 pkg create --build-type ament_python navigation_system --dependencies rclpy geometry_msgs interview_interfaces action_msgs

# Create necessary directories
mkdir -p interview_interfaces/msg
mkdir -p interview_interfaces/srv
mkdir -p interview_interfaces/action

# Create source directories for Python packages
mkdir -p sensor_system/sensor_system
mkdir -p diagnostic_system/diagnostic_system
mkdir -p navigation_system/navigation_system

# Create launch directories
mkdir -p sensor_system/launch
mkdir -p diagnostic_system/launch
mkdir -p navigation_system/launch
```
