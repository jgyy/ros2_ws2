#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_control.action import MoveRobot
import sys
import ast


class MoveCommander(Node):
    def __init__(self):
        super().__init__('move_commander')
        self._action_client = ActionClient(self, MoveRobot, 'move_robot')
        self.get_logger().info('Move Commander has been started')

    def send_goal(self, target_position):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        goal_msg = MoveRobot.Goal()
        goal_msg.target_position = target_position
        self.get_logger().info(f'Sending goal: {target_position}')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = 'Success' if result.success else 'Failed'
        self.get_logger().info(
            f'Movement {status}\n'
            f'Final position: {result.final_position}\n'
            f'Completion time: {result.completion_time:.2f} seconds'
        )
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Progress: {feedback.percent_complete:.1f}% - '
            f'Current position: {[f"{x:.2f}" for x in feedback.current_position]}'
        )


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        print('Usage: move_commander.py "[j1, j2, j3, j4, j5, j6]"')
        print('Example: move_commander.py "[1.0, -0.5, 0.8, 1.2, -0.3, 0.7]"')
        return
    try:
        target_position = ast.literal_eval(sys.argv[1])
        if not isinstance(target_position, list) or len(target_position) != 6:
            raise ValueError
    except:
        print('Invalid target position format. Must be list of 6 numbers.')
        return
    action_client = MoveCommander()
    action_client.send_goal(target_position)
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        print('Interrupted, cancelling goal...')
    finally:
        action_client.destroy_node()


if __name__ == '__main__':
    main()
