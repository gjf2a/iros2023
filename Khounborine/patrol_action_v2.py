import sys
import math
import runner
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector
from action_demo import RotateActionClient
from ForwardActionClient import ForwardActionClient


class PatrollerBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('patrol_bot')
        self.driver = ForwardActionClient(self.move_finished_callback, namespace)
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Timer Required for "Type Anything to Quit" from runner.py"
        self.ir_clear_count = 0
        self.ir_sense = True
        self.driver.send_goal(1)

    def timer_callback(self):
        self.record_first_callback()

    # ir_callback does not interact with the actions. Not yet anyway.
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
            
    def move_finished_callback(self, future):
        print("ForwardNode: Move finished")
        self.rotator.send_goal(math.pi)

    def turn_finished_callback(self, future):
        print("finished with turn")
        self.driver.send_goal(1)


if __name__ == '__main__':
    runner.run_single_node(lambda: PatrollerBot(f'/{sys.argv[1]}'))
