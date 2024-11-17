# ROS2 Interview Coding Test
Time Limit: 1 hour

## Environment Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- Your preferred code editor

## Task 1: Sensor Data Processing (15 minutes)
Create a sensor data processing system with three nodes:

1. Create a custom message type `SensorData.msg`:
```
time timestamp
float32 temperature
float32 humidity
float32 pressure
geometry_msgs/Point location
```

2. Implement `sensor_simulator` node:
- Generate realistic sensor readings at 5Hz
- Temperature range: 20-30°C
- Humidity range: 30-70%
- Pressure range: 980-1020 hPa
- Simulate sensor movement in a 10x10m area
- Add Gaussian noise to all readings

3. Create `data_filter` node:
- Subscribe to raw sensor data
- Implement moving average filter (window size=5)
- Detect and flag outliers (>2 standard deviations)
- Publish filtered data on separate topic

4. Develop `data_analyzer` node:
- Subscribe to both raw and filtered data
- Calculate and publish statistics every 10 seconds:
  - Min/max/average values
  - Number of outliers detected
  - Sensor coverage area

## Task 2: Diagnostic System (15 minutes)
Implement a diagnostic system using services:

1. Create custom service `DiagnosticCheck.srv`:
```
# Request
string component_name
bool detailed_check
---
# Response
bool status_ok
string[] warnings
string[] errors
float32 check_duration
```

2. Create `system_monitor` node:
- Implement service server for diagnostic checks
- Monitor system resources (CPU, memory)
- Check node health and communication
- Simulate component-specific checks
- Return detailed diagnostics if requested

3. Build `diagnostic_client` node:
- Command-line interface for triggering checks
- Option to check specific components or all
- Pretty-print diagnostic results
- Implement timeout handling

## Task 3: Waypoint Navigation (20 minutes)
Create a waypoint navigation system using actions:

1. Define custom action `Navigate.action`:
```
# Goal
geometry_msgs/Pose[] waypoints
float32 max_velocity
bool allow_partial_completion
---
# Result
uint32 completed_waypoints
float32 total_distance
bool all_waypoints_reached
---
# Feedback
uint32 current_waypoint
geometry_msgs/Pose current_pose
float32 distance_remaining
float32 estimated_time_remaining
```

2. Create `navigation_server` node:
- Accept waypoint list and execute sequentially
- Simulate robot movement with realistic acceleration
- Check for obstacles (simulate random obstacles)
- Provide regular feedback
- Allow goal modification while executing
- Support premature completion if partial completion allowed

3. Develop `navigation_commander` node:
- Load waypoints from YAML file
- Send navigation goals
- Monitor progress and display feedback
- Handle navigation failures
- Implement waypoint modification commands

## Bonus Challenges (10 minutes)
Choose any to implement:
- Add visualization using RViz markers
- Implement dynamic parameter reconfiguration
- Create a simple web interface using roslibjs
- Add logging and performance metrics
- Implement multi-robot support

## Evaluation Criteria

### Code Structure (25%)
- Clear organization and modularity
- Proper package structure
- Consistent coding style
- Documentation quality

### ROS2 Implementation (25%)
- Proper use of ROS2 concepts
- Efficient communication patterns
- Resource management
- Node lifecycle handling

### Technical Implementation (25%)
- Algorithm correctness
- Error handling
- Performance considerations
- System reliability

### Problem Solving (25%)
- Solution approach
- Edge case handling
- Debug-friendly implementation
- Scalability considerations

## Project Structure
```
interview_ws/
├── src/
│   ├── sensor_system/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── sensor_system/
│   │   │   ├── __init__.py
│   │   │   ├── sensor_simulator.py
│   │   │   ├── data_filter.py
│   │   │   └── data_analyzer.py
│   │   └── msg/
│   │       └── SensorData.msg
│   ├── diagnostic_system/
│   │   └── [similar structure]
│   ├── navigation_system/
│   │   └── [similar structure]
│   └── CMakeLists.txt
└── README.md
```

## Sample Test Cases

### Task 1
```bash
# Terminal 1
ros2 run sensor_system sensor_simulator

# Terminal 2
ros2 run sensor_system data_filter

# Terminal 3
ros2 run sensor_system data_analyzer

# Verify
ros2 topic echo /sensor_data/raw
ros2 topic echo /sensor_data/filtered
ros2 topic echo /sensor_data/statistics
```

### Task 2
```bash
# Terminal 1
ros2 run diagnostic_system system_monitor

# Terminal 2
ros2 run diagnostic_system diagnostic_client check_all
ros2 run diagnostic_system diagnostic_client check_component sensor_system --detailed
```

### Task 3
```bash
# Terminal 1
ros2 run navigation_system navigation_server

# Terminal 2
ros2 run navigation_system navigation_commander load_waypoints.yaml
```

## Required README.md Content
Your README should include:
1. System requirements
2. Installation steps
3. Running instructions
4. Testing procedures
5. Design decisions explained
6. Known limitations
7. Future improvements

## Submission Guidelines
1. Create a new GitHub repository
2. Implement all required nodes
3. Include proper documentation
4. Add example configuration files
5. Include basic unit tests
6. Provide launch files

## Notes
- Focus on core functionality first
- Comment critical sections
- Use meaningful variable names
- Implement proper error handling
- Follow ROS2 naming conventions
- Clean up resources properly
- Use type hints in Python code
