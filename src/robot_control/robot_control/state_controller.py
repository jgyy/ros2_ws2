#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control.srv import ChangeState
import sys

class StateController(Node):
    def __init__(self):
        super().__init__('state_controller')
        self.cli = self.create_client(ChangeState, 'change_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicce not available, waiting...')
        self.req = ChangeState.Request()

    def send_request(self, new_state):
        self.req.new_state = new_state
        future = self.cli.call_async(self.req)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = StateController()
    if len(sys.argv) != 2:
        node.get_logger().error('Usage: state_controller.py <new_state>')
        node.get_logger().error('Valid states: IDLE, MOVING, CHARGING')
        node.destroy_node()
        rclpy.shutdown()
        return
    new_state = sys.argv[1].upper()
    future = node.send_request(new_state)
    try:
        rclpy.spin_until_future_complete(node, future)
        if future.done():
            response = future.result()
            if response.success:
                node.get_logger().info(
                    f'Successfully changed state from {response.previous_state} '
                    f'to {response.current_state}'
                )
            else:
                node.get_logger().error(
                    f'Failed to change state. Current state remains: '
                    f'{response.current_state}'
                )
        else:
            node.get_logger().error('Service call failed')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
