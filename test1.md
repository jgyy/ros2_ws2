# ROS2 Coding Interview Test
Time Limit: 1 hour

## Overview
This test evaluates your understanding of ROS2 fundamentals, including node creation, publisher/subscriber patterns, services, and basic robot control concepts.

## Environment Setup
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- Visual Studio Code (or preferred editor)

## Task 1: Basic Node Communication (15 minutes)
Create two ROS2 nodes that communicate using a custom message type:

1. Create a custom message type `RobotStatus.msg`:
```
float64 battery_level
string current_state
float64[] joint_positions
```

2. Create a publisher node `robot_state_publisher`:
- Publish robot status at 10Hz
- Simulate battery drain (starts at 100%, decreases by 0.1% each message)
- Randomly cycle through states: "IDLE", "MOVING", "CHARGING"
- Generate random joint positions for 6 joints

3. Create a subscriber node `robot_monitor`:
- Subscribe to robot status
- Print warning when battery below 20%
- Log joint positions to a file when state changes
- Calculate and print average joint position

## Task 2: Service Implementation (15 minutes)
Create a service to control the robot's state:

1. Create a custom service `ChangeState.srv`:
```
# Request
string new_state
---
# Response
bool success
string previous_state
string current_state
```

2. Modify the publisher node to:
- Add service server that changes robot state
- Only accept valid states ("IDLE", "MOVING", "CHARGING")
- Return previous and new state in response
- Return success=false if invalid state requested

3. Create a service client node `state_controller`:
- Implement command-line interface to change robot state
- Handle service call failures gracefully
- Print response information

## Task 3: Action Server Integration (20 minutes)
Implement an action server for a simulated robot movement:

1. Create a custom action `MoveRobot.action`:
```
# Goal
float64[] target_position
---
# Result
bool success
float64[] final_position
float64 completion_time
---
# Feedback
float64 percent_complete
float64[] current_position
```

2. Create an action server node `robot_mover`:
- Accept target joint positions
- Simulate movement by gradually updating positions
- Provide feedback every 0.5 seconds
- Implement goal preemption
- Handle invalid target positions

3. Create an action client node `move_commander`:
- Send goal positions
- Print feedback
- Allow cancellation of goals
- Handle results

## Bonus Tasks (10 minutes)
If time permits, implement any of these additional features:
- Add parameter server integration
- Implement basic error handling and recovery
- Add unit tests for core functionality
- Implement launch files
- Add rqt_graph visualization support

## Evaluation Criteria
1. Code Quality (30%)
- Clean, well-organized code
- Proper error handling
- Good naming conventions
- Appropriate comments

2. ROS2 Best Practices (30%)
- Proper node lifecycle management
- Efficient message passing
- Appropriate use of ROS2 patterns
- Resource cleanup

3. Functionality (30%)
- All basic requirements met
- Code runs without errors
- Proper handling of edge cases
- System behaves as expected

4. Extra Features (10%)
- Bonus tasks attempted
- Creative solutions
- Additional useful features

## Submission
1. Create a workspace with the following structure:
```
interview_ws/
├── src/
│   ├── robot_control/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── robot_control/
│   │   │   ├── __init__.py
│   │   │   ├── publisher_node.py
│   │   │   ├── subscriber_node.py
│   │   │   ├── service_node.py
│   │   │   └── action_node.py
│   │   └── msg/
│   │       └── RobotStatus.msg
│   └── CMakeLists.txt
└── README.md
```

2. Include a README.md with:
- Setup instructions
- Usage examples
- Any assumptions made
- Known limitations
- Future improvements

## Testing Instructions
To test your implementation:
1. Build the workspace: `colcon build`
2. Source the setup files: `. install/setup.bash`
3. Run nodes in separate terminals
4. Verify communication between nodes
5. Test error cases and edge conditions
