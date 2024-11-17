#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_control.action import MoveRobot
import numpy as np
import time
from rclpy.executors import MultiThreadedExecutor


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(self, MoveRobot, 'move_robot',
            self.execute_callback, callback_group=self.callback_group)
        self.current_position = [0.0] * 6
        self.get_logger().info('Robot Mover Action Server has been started')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        feedback_msg = MoveRobot.Feedback()
        result = MoveRobot.Result()
        target_position = goal_handle.request.target_position
        if len(target_position) != 6:
            self.get_logger().error('Invalid target position length')
            goal_handle.abort()
            result.success = False
            result.final_position = self.current_position
            result.completion_time = 0.0
            return result
        current = np.array(self.current_position)
        target = np.array(target_position)
        start_time = self.get_clock().now()
        rate = self.create_rate(2)
        try:
            distance = np.linagl.norm(target - current)
            moved_distance = 0.0
            while moved_distance < distance:
                if not goal_handle.is_active:
                    self.get_logger().info('Goal no longer active')
                    return result
                if goal_handle.is_cancel_requested:
                    goal_handle.cancelled()
                    self.get_logger().info('Goal canceled')
                    return result
                moved_distance = min(moved_distance + 0.1 * distance, distance)
                progress = moved_distance / distance
                self.current_position = list(current + progress * (target - current))
                feedback_msg.percent_complete = progress * 100.0
                feedback_msg.current_position = self.current_position
                goal_handle.publish_feedback(feedback_msg)
                rate.sleep()
            end_time = self.get_clock().now()
            completion_time = (end_time - start_time).nanoseconds / 1e9
            result.success = True
            result.final_position = self.current_position
            result.completion_time = completion_time
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')
            return result
        except Exception as e:
            self.get_logger().error(f'Movement failed: {str(e)}')
            goal_handle.abort()
            result.success = False
            result.final_position = self.current_position
            result.completion_time = 0.0
            return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
