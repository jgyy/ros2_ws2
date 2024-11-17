#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_control.action import MoveRobot
import sys
import ast
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor


class MoveCommander(Node):
    def __init__(self):
        super().__init__('move_commander')
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(self, MoveRobot, 'move_robot',
            callback_group=self.callback_group)
        self._goal_handle = None
        self._result_future = None
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Action server not available, waiting...')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Movement progress: {feedback.percentage_complete:.1f}%]n'
            f'Current position: {[f"{x:.2f}" for x in feedback.current_position]}'
        )

    async def send_goal(self, target_position):
        goal_msg = MoveRobot.Goal()
        goal_msg.target_position = target_position
        self.get_logger().info(f'Sending goal: {target_position}')
        self._goal_handle = await self._action_client.send_goal_async(goal_msg,
            feedback_callback=self.feedback_callback)
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        self.get_logger().info('Goal accepted')
        self._result_future = await self._goal_handle.get_result_async()
        result = self._result_future.result
        if result.success:
            self.get_logger().info(
                f'Goal succeeded!\n'
                f'Final position {[f"{x:.2f}" for x in result.final_position]}\n'
                f'Completion time: {result.completion_time:.2f} seconds'
            )
        else:
            self.get_logger().error('Goal failed!')
        return result.success

    async def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Canceling goal')
            cancel_future = await self._goal_handle.cancel_goal_async()
            if cancel_future.accepted:
                self.get_logger().info('Goal canceled')
            else:
                self.get_logger().error('Failed to cancel goal')


def main(args=None):
    rclpy.init(args=args)
    node = MoveCommander()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    if len(sys.argv) != 2:
        node.get_logger().error('Usage: ros2 run robot_control move_commander "[x1,x2,x3,x4,x5,x6]"')
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        target_position = ast.literal_eval(sys.argv[1])
        if not isinstance(target_position, list) or len(target_position) != 6:
            raise ValueError('Target position must be a list of 6 numbers')
        future = node.send_goal(target_position)
        executor.spin_until_future_complete(future)
    except KeyboardInterrupt:
        future = node.cancel_goal()
        executor.spin_until_future_complete(future)
    except Exception as e:
        node.get_logger().error(f'Error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
