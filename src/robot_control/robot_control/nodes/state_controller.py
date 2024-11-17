#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control.srv import ChangeState
import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor


class StateController(Node):
    def __init__(self):
        super().__init__('state_controller')
        self.callback_group = ReentrantCallbackGroup()
        self.client = self.create_client(ChangeState, 'change_state',
            callback_group=self.callback_group)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')

    async def change_state(self, new_state):
        request = ChangeState.Request()
        request.new_state = new_state
        try:
            self.get_logger().info(f'Requesting state change to: {new_state}')
            response = await self.client.call_async(request)
            if response.success:
                self.get_logger().info(
                    f'State change successful!\n'
                    f'Previous state: {response.previous_state}\n'
                    f'Current state: {response.current_state}'
                )
            else:
                self.get_logger().error(
                    f'State change failed!\n'
                    f'State remains: {response.current_state}'
                )
            return response.success
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = StateController()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    if len(sys.argv) != 2:
        node.get_logger().error('Usage: ros2 run robot_control state_controller <new_state>')
        node.destroy_node()
        rclpy.shutdown()
        return
    new_state = sys.argv[1].upper()
    try:
        future = node.change_state(new_state)
        executor.spin_until_future_complete(future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
