# ros2_ws2
robot operating system workspace 2

## Test 60 minutes

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
