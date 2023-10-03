import sys
import math
import runner
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_demo import RotateActionClient


class PatrollerBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('patrol_bot')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.location = self.create_subscription(Odometry, namespace + '/odom', self.odom_callback, qos_profile_sensor_data)
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Timer Required for "Type Anything to Quit" from runner.py"
        self.true_loc = 0.0
        self.wheel_turn = False
        self.spin_wheel = False
        self.ir_clear_count = 0
        self.ir_sense = True

    def timer_callback(self):
        self.record_first_callback()

    def ir_callback(self, msg):
        for reading in msg.readings:
            det = reading.header.frame_id
            val = reading.value
            if det != "base_link":
                self.ir_check(det, val)
        if self.ir_clear_count == 7:
            self.ir_sense = True
        self.ir_clear_count = 0

    def ir_check(self, sensor: str = "", val: int = 0):
        if val > 100:
            self.ir_sense = False
        else:
            self.ir_clear_count += 1

    def odom_callback(self, msg):
        loc = msg.pose.pose.position.y
        if self.true_loc == 0.0:
            self.true_loc = loc
        act_loc = loc - self.true_loc
        print(f"position: {act_loc}")
        if not self.wheel_turn:
            if act_loc > 1.0 or act_loc < 0.0:
                self.publisher.publish(runner.straight_twist(0.0))
                self.wheel_turn = True
            elif not self.ir_sense:
                self.publisher.publish(runner.straight_twist(0.0))
            elif act_loc <= 1.0 and act_loc >= 0.0:
                self.publisher.publish(runner.straight_twist(0.5))
        else:
            if not self.spin_wheel:
                goal = math.pi
                print("Starting turn", goal)
                self.rotator.send_goal(goal)
                rclpy.spin_once(self.rotator)
            elif self.spin_wheel:
                self.publisher.publish(runner.straight_twist(0.5))
            if act_loc < 1.0 and act_loc > 0.0:
                self.wheel_turn = False
                self.spin_wheel = False
            
    def turn_finished_callback(self, future):
        self.spin_wheel = True
        print("finished with turn")


if __name__ == '__main__':
    runner.run_single_node(lambda: PatrollerBot(f'/{sys.argv[1]}'))
