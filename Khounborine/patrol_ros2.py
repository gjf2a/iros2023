import runner
import sys
import time
import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


class SonarBot1(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.location = self.create_subscription(Odometry, namespace + '/odom', self.odom_callback, qos_profile_sensor_data)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Timer Required for "Type Anything to Quit" from runner.py"
        self.forward = Twist()
        self.forward.linear.x = 0.5
        self.turn_left = Twist()
        self.turn_left.angular.z = 1.0
        self.turn_right = Twist()
        self.turn_right.angular.z = -1.0
        self.stop = Twist()
        self.stop.linear.x = 0.0
        self.ir_clear_count = 0
        self.true_loc = 0.0
        self.true_dir = 0.0
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
        dir = msg.pose.pose.orientation.z
        if self.true_loc == 0.0 and self.true_dir == 0.0:
            self.true_loc = loc
            self.true_dir = dir
        act_loc = loc - self.true_loc
        act_dir = dir - self.true_dir
        print(f"position: {act_loc}")
        print(f"orientation: {act_dir}")
        if act_loc > 1.0:
            self.publisher.publish(self.turn_right)
            if act_dir < -1.3:
                self.publisher.publish(self.forward)
        elif act_loc < 0.0:
            self.publisher.publish(self.turn_left)
            if act_dir > -0.1:
                self.publisher.publish(self.forward)
        else:
            if self.ir_sense:
                self.publisher.publish(self.forward)
            else:
                self.publisher.publish(self.stop)
        

if __name__ == '__main__':
    print(f"Starting up {sys.argv[1]}...")
    runner.run_single_node(lambda: SonarBot1(f'/{sys.argv[1]}'))
