# ros2_ws2
robot operating system workspace 2

## Test 60 minutes

```sh
# Create the workspace and source directories
mkdir -p ros2_ws2/src
cd ros2_ws2/src

# Create the ROS2 package
ros2 pkg create --build-type ament_python robot_control --dependencies rclpy std_msgs

# Create the message, service, and action directories
cd robot_control
mkdir -p msg srv action

# Create source directories for Python modules
mkdir -p robot_control/nodes

# Create the message files
touch msg/RobotStatus.msg
touch srv/ChangeState.srv
touch action/MoveRobot.action

# Create the Python node files
touch robot_control/nodes/__init__.py
touch robot_control/nodes/robot_state_publisher.py
touch robot_control/nodes/robot_monitor.py
touch robot_control/nodes/state_controller.py
touch robot_control/nodes/robot_mover.py
touch robot_control/nodes/move_commander.py

# Return to workspace root
cd ../..

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Commands to run the nodes (in separate terminals after sourcing):
# Terminal 1: ros2 run robot_control robot_state_publisher
# Terminal 2: ros2 run robot_control robot_monitor
# Terminal 3: ros2 run robot_control state_controller
# Terminal 4: ros2 run robot_control robot_mover
# Terminal 5: ros2 run robot_control move_commander
```

```sh
ros2 launch robot_control robot_control.launch.py
ros2 run robot_control state_controller MOVING
ros2 run robot_control move_commander "[1.0, 0.5, 0.0, -0.5, 1.0, 0.0]"
```
