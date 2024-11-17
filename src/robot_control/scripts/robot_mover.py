#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from robot_control.action import MoveRobot
import numpy as np
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.current_position = np.zeros(6)
        self.movement_duration = 5.0
        self._action_server = ActionServer(
            self,
            MoveRobot,
            'move_robot',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info('Robot Mover Action Server has been started')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing movement...')
        feedback_msg = MoveRobot.Feedback()
        result = MoveRobot.Result()
        target = np.array(goal_handle.request.target_position)
        if len(target) != 6:
            self.get_logger().error('Invalid target position length')
            goal_handle.abort()
            result.success = False
            return result
        start_time = time.time()
        start_position = self.current_position.copy()
        while time.time() - start_time < self.movement_duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.final_position = self.current_position.tolist()
                result.completion_time = time.time() - start_time
                return result
            elapsed_time = time.time() - start_time
            progress = min(elapsed_time / self.movement_duration, 1.0)
            self.current_position = start_position + progress * (
                    target - start_position)
            feedback_msg.percent_complete = progress * 100
            feedback_msg.current_position = self.current_position.tolist()
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        self.current_position = target
        result.success = True
        result.final_position = self.current_position.tolist()
        result.completion_time = time.time() - start_time
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
