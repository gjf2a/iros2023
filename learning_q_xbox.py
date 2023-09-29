import rclpy
from qlearning import QBot, QParameters, QTable, QNodeTemplate
import runner
import math
import sys
from queue import Queue
import threading
from evdev import InputDevice

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

def forward_turning(forward_velocity, turn_velocity):
    t = Twist()
    t.linear.x = forward_velocity
    t.angular.z = turn_velocity
    return t


class QDemoNode(QNodeTemplate):
    def __init__(self, namespace, msg_queue, x_meters, x_squares, y_meters, y_squares):
        super().__init__('learning_q_xbox', namespace, runner.turn_twist(0.5, -math.pi / 4), runner.straight_twist(0.5), runner.turn_twist(0.5, math.pi / 4))
        self.odometry = self.create_subscription(Odometry, namespace + '/odom', self.odom_callback, qos_profile_sensor_data)        
        self.x_meters = x_meters
        self.y_meters = y_meters
        self.x_squares = x_squares
        self.y_squares = y_squares
        self.msg_queue = msg_queue
        self.last_cmd = None

    def num_squares(self):
        return self.x_squares * self.y_squares

    def num_states(self):
        return self.num_squares() + 1

    def out_of_bounds_state(self):
        return self.num_states() - 1

    def set_reward(self, state):
        if not self.msg_queue.empty():
            self.last_cmd = self.msg_queue.get()
        if state == self.out_of_bounds_state():
            return -100
        elif self.last_cmd == 'A':
            return 10
        elif self.last_cmd == 'B':
            return -10
        else:
            return 0

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        x = int(p.x * self.x_squares / self.x_meters)
        y = int(p.y * self.y_squares / self.y_meters)
        self.state = y * self.x_squares + x
        if not (0 <= self.state < self.num_squares()):
            self.state = self.out_of_bounds_state()


class XBoxReader:
    def __init__(self, msg_queue, incoming):
        self.msg_queue = msg_queue
        self.incoming = incoming

    def loop(self):
        dev = InputDevice('/dev/input/event0')
        for event in dev.read_loop():
            if event.value == 1:
                if event.code == 304:
                    self.msg_queue.put("A")
                elif event.code == 305:
                    self.msg_queue.put("B")
            if not self.incoming.empty():
                print("got message")
                break



if __name__ == '__main__':
    rclpy.init()
    namespace = f'{sys.argv[1]}' if len(sys.argv) >= 2 else ''
    params = QParameters()
    params.epsilon = 0.05
    from_x = Queue()
    to_x = Queue()
    xboxer = XBoxReader(from_x, to_x)

    demo_node = QDemoNode(namespace, from_x, x_meters=3.0, y_meters=2.0, x_squares=10, y_squares=12)
    main_node = QBot(demo_node, params)
    xbox_thread = threading.Thread(target=lambda x: x.loop(), args=(xboxer,))
    xbox_thread.start()
    runner.run_recursive_node(main_node)
    print("Runner done")
    main_node.print_status()
    to_x.put("QUIT")
