# Based on https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/pub_LED.py

import runner
import sonar

import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from rclpy.qos import qos_profile_sensor_data


class SonarBot1(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')

        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)

        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.forward = Twist()
        self.forward.linear.x = 0.5
        self.turn = Twist()
        self.turn.angular.z = 0.628

        self.sonar = sonar.Sonar(1, 93, 94)

    def timer_callback(self):
        self.record_first_callback()

        distance = self.sonar.read()
        if distance < 50:
            self.publisher.publish(self.turn)
        else:
            self.publisher.publish(self.forward)

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
        


if __name__ == '__main__':
    runner.run_single_node(lambda: SonarBot1('/archangel'))
