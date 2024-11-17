#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control.msg import RobotStatus
import numpy as np
from datetime import datetime


class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.last_state = None
        self.log_file = f'joint_positions_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
        self.subscription = self.create_subscription(RobotStatus, 'robot_status',
                                                     self.status_callback, 10)
        self.get_logger().info('Robot Monitor has been started')

    def status_callback(self, msg):
        if msg.battery_level < 20.0:
            self.get_logger().warn(f'Low battery warning: {msg.battery_level:.1f}%')
        if self.last_state != msg.current_state:
            self.log_joint_positions(msg.joint_positions, msg.current_state)
            self.last_state = msg.current_state
        avg_position = np.mean(msg.joint_positions)
        self.get_logger().info(
            f'State: {msg.current_state}, '
            f'Battery: {msg.battery_level:.1f}%, '
            f'Avg Joint Pos: {avg_position:.2f}'
        )

    def log_joint_positions(self, positions, state):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.log_file, 'a') as f:
            pos_str = ', '.join([f'{p:.3f}' for p in positions])
            f.write(f'{timestamp} - State: {state} - Positions [{pos_str}]\n')
        self.get_logger().info(f'Joint positions logged for state change to {state}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
