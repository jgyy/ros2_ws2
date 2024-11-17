#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control.msg import RobotStatus
from robot_control.srv import ChangeState
import random
import numpy as np


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.battery_level = 100
        self.current_state = "IDLE"
        self.valid_states = ["IDLE", "MOVING", "CHARGING"]
        self.joint_positions = [0.0] * 6
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.srv = self.create_service(ChangeState, 'change_state',
                                       self.change_state_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state_change_timer = self.create_timer(5.0, self.random_state_change)
        self.get_logger().info('Robot State Publisher has been started')

    def timer_callback(self):
        msg = RobotStatus()
        if self.current_state != 'CHARGING':
            self.battery_level = max(0.0, self.battery_level - 0.1)
        else:
            self.battery_level = min(100.0, self.battery_level + 0.5)
        if self.current_state == 'MOVING':
            self.joint_positions = [random.uniform(-3.14, 3.14) for _ in range(6)]
        msg.battery_level = self.battery_level
        msg.current_state = self.current_state
        msg.joint_positions = self.joint_positions
        self.publisher.publish(msg)

    def random_state_change(self):
        if random.random() < 0.3:
            new_state = random.choice(self.valid_states)
            if new_state != self.current_state:
                self.get_logger().info(f'Randomly changing state from {self.current_state} to {new_state}')
                self.current_state = new_state

    def charge_state_callback(self, request, response):
        previous_state = self.current_state
        if request.new_state in self.valid_states:
            self.current_state = request.new_state
            response.success = True
        else:
            response.success = False
        response.previous_state = previous_state
        response.current_state = self.current_state
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
