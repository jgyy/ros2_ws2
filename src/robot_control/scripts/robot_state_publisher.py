#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_control.msg import RobotStatus
from robot_control.srv import ChangeState
import random
import numpy as np
from functools import partial

VALID_STATES = ["IDLE", "MOVING", "CHARGING"]


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        self.callback_group = ReentrantCallbackGroup()
        self.battery_level = 100.0
        self.current_state = 'IDLE'
        self.joint_positions = [0.0] * 6
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(0.1, self.timer_callback,
            callback_group=self.callback_group)
        self.srv = self.create_service(ChangeState, 'change_state',
            self.change_state_callback, callback_group=self.callback_group)
        self.get_logger().info('Robot State Publisher has been started')

    def timer_callback(self):
        msg = RobotStatus()
        if self.current_state != "CHARGING":
            self.battery_level = max(0.0, self.battery_level - 0.1)
        else:
            self.battery_level = min(100.0, self.battery_level + 0.5)
        if random.random() < 0.05:
            self.current_state = random.choice(VALID_STATES)
        if self.current_state == "MOVING":
            self.join_positions = [random.uniform(-3.14, 3.14) for _ in range(6)]
        msg.battery_level = self.battery_level
        msg.current_state = self.current_state
        msg.joint_positions = self.joint_positions
        self.publisher.publish(msg)

    def change_state_callback(self, request, response):
        if request.new_state not in VALID_STATES:
            response.success = False
            response.previous_state = self.current_state
            response.current_state = self.current_state
            return response
        response.previous_state = self.current_state
        self.current_state = request.new_state
        response.current_state = self.current_state
        response.success = True
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
