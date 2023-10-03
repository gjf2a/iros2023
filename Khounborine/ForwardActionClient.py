# Based on:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance
from rclpy.node import Node
import math, sys, threading


class ForwardActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Drive_Forward")
        self._action_client = ActionClient(self, DriveDistance, f'{namespace}/drive_distance')
        self.callback = callback
        
    def send_goal(self, goal_distance, wheel_speed=0.5):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = goal_distance
        goal_msg.max_translation_speed = wheel_speed
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
    print("Entering example_callback")
    result = future.result().result
    print("finished...", result)


def main(args=None, namespace=''):
    global finish_flag
    rclpy.init(args=args)

    action_client = ForwardActionClient(example_callback, namespace)
    action_client.send_goal(0.5)
    rclpy.spin()
    print("done")


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        main(namespace=f'/{sys.argv[1]}')
    else:
        main()
