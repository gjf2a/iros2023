import sys
import runner
import time
import rclpy
import cv2
import math

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, WheelStatus
from rclpy.qos import qos_profile_sensor_data

from morph_contour_demo import Timer, find_contours, find_close_contour, multi_flood_fill

from queue import Queue
import threading

from fuzzy import *
from ir_bump_turn import IrBumpTurnNode


class VisionBot(runner.WheelMonitorNode):
    def __init__(self, img_queue, namespace: str = "", ir_limit=50):
        super().__init__('wheel_publisher', namespace)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.ir_node = IrBumpTurnNode(namespace, ir_limit)
        timer_period = 0.10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.img_queue = img_queue
        self.last_target = None

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.ir_node.add_self_recursive(executor)

    def timer_callback(self):
        self.record_first_callback()
        if not self.ir_node.is_turning():
            if self.ir_node.ir_clear():
                self.publish_fuzzy_move()
            elif self.wheels_stopped():
                self.ir_node.request_turn_until_clear()
            else:
                print("waiting on ir_node")
        else:
            print("ir_node taking over")

    def publish_fuzzy_move(self):
        if not self.img_queue.empty():
            best = self.img_queue.get()
            if type(best) == runner.CvKey:
                print("Typed", best)
            elif best == "QUIT":
                self.quit()
            elif len(best) == 0:
                msg = runner.turn_twist(math.pi / 2)
                self.publisher.publish(msg)
                self.last_target = None
                print("Vision turning")
            else:
                if self.last_target is None or len(best) == 1:
                    self.last_target = best[0]
                else:
                    closest = 0
                    closest_x_gap = abs(best[0] - self.last_target)
                    for i in range(1, len(best)):
                        gap = abs(best[i] - self.last_target)
                        if gap < closest_x_gap:
                            closest = i
                            closest_x_gap = gap
                    self.last_target = best[closest]

                fuzzy_center = fuzzify_falling(self.last_target, 0, 640)
                msg = Twist()
                msg.linear.x = 0.1  
                msg.angular.z = defuzzify(fuzzy_center, -math.pi/4, math.pi/4)
                print(f"best: ({self.last_target}) {fuzzy_center} {msg.angular.z}")
                self.publisher.publish(msg)



def find_floor_contour(frame, cap, kernel_midwidth):
    kernel, min_space_width = kernel_midwidth
    kernel_size = (kernel, kernel)

    frame = cv2.resize(frame, (640, 480))
    contours, hierarchy = find_contours(frame, kernel_size)
    close_contour = find_close_contour(contours, cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    if close_contour is None:
        print("contour finding failure")
        return frame, []
    else:
        centroids = multi_flood_fill(frame, close_contour, 0.10, 0.20)
        return frame, centroids



class FloorContour(runner.OpenCvCode):
    def __init__(self, msg_queue):
        super().__init__(0, find_floor_contour, (9, 20), msg_queue)


if __name__ == '__main__':
    rclpy.init()
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    bot = VisionBot(msg_queue, f'/{sys.argv[1]}')
    runner.run_recursive_vision_node(FloorContour(msg_queue), bot)
