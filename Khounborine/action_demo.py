# Based on:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from rclpy.node import Node
import math, sys

class RotateActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Demo")
        self._action_client = ActionClient(self, RotateAngle, f'{namespace}/rotate_angle')
        self.callback = callback
        
    def send_goal(self, goal_heading, radians_per_sec=1.0):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = goal_heading
        goal_msg.max_rotation_speed = radians_per_sec
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected...")
        else:
            print("Goal accepted.")    
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.callback)


def example_callback(future):
    result = future.result().result
    print("finished...", result)
    rclpy.shutdown()


def main(args=None, namespace=''):
    rclpy.init(args=args)

    action_client = RotateActionClient(example_callback, namespace)
    action_client.send_goal(math.pi)
    rclpy.spin(action_client)
    print("done")


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        main(namespace=f'/{sys.argv[1]}')
    else:
        main()
