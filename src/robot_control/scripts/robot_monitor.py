#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control.msg import RobotStatus
import numpy as np
from datetime import datetime
import os


class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.subscription = self.create_subscription(RobotStatus, 'robot_status',
            self.status_callback, 10)
        self.last_state = None
        self.log_file = 'joint_positions.log'
        os.makedirs('logs', exist_ok=True)
        self.log_file = os.path.join('logs', self.log_file)
        self.get_logger().info('Robot Monitor has been started')

    def status_callback(self, msg):
        if msg.battery_level < 20.0:
            self.get_logger().warn(f'Low battery warning: {msg.battery_level:.1f}%')
        if self.last_state != msg.current_state:
            self.log_joint_positions(msg.current_state, msg.joint_positions)
            self.last_state = msg.current_state
        avg_position = np.mean(msg.joint_positions)
        self.get_logger().info(
            f'State: {msg.current_state}, '
            f'Battery: {msg.battery_level:.1f}%, '
            f'Avg Joint Pos: {avg_position:.2f}'
        )

    def log_joint_positions(self, state, position):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(self.log_file, 'a') as f:
            position_str = ','.join(f'{p:.3f}' for p in positions)
            f.write(f'{timestamp},{state},{position_str}\n')


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
