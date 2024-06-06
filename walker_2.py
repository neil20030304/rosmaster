#!/usr/bin/env python
# encoding: utf-8

# Import required libraries
import sys, select, termios, tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

# Define the key bindings and speed adjustments
msg = """
Control Your SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

speedBindings = {
    'Q': (1.1, 1.1),
    'Z': (.9, .9),
    'W': (1.1, 1),
    'X': (.9, 1),
    'E': (1, 1.1),
    'C': (1, .9),
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class Yahboom_Keyboard(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = None
        self.duration = 5  # Duration in seconds
        self.stop = False
        self.speed = 0.2
        self.turn = 1.0
        self.xspeed_switch = True

    def keyboard_listener(self):
        print(msg)
        while not self.stop:
            key = self.getKey()
            if key == "w":
                self.move_for_duration()
            elif key == "t" or key == "T":
                self.xspeed_switch = not self.xspeed_switch
            elif key == "s" or key == "S":
                self.stop = not self.stop
                print("stop keyboard control: {}".format(self.stop))
            elif key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                self.move(x, th)
            elif key in speedBindings.keys():
                self.speed *= speedBindings[key][0]
                self.turn *= speedBindings[key][1]
                if self.speed > 1.0:
                    self.speed = 1.0
                    print("Linear speed limit reached!")
                if self.turn > 5.0:
                    self.turn = 5.0
                    print("Angular speed limit reached!")
                print("currently:\tspeed {}\tturn {} ".format(self.speed, self.turn))
            elif key == ' ':
                self.move(0, 0)
            else:
                self.move(0, 0)
        self.move(0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def move(self, x, th):
        twist = Twist()
        twist.angular.z = self.turn * th
        if self.xspeed_switch:
            twist.linear.x = self.speed * x
        else:
            twist.linear.y = self.speed * x
        self.pub.publish(twist)

    def move_for_duration(self):
        self.move(1, 0)  # Move forward
        time.sleep(self.duration)
        self.move(0, 0)  # Stop moving

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main():
    rclpy.init()
    yahboom_keyboard = Yahboom_Keyboard("yahboom_keyboard_ctrl")
    yahboom_keyboard.keyboard_listener()
    yahboom_keyboard.destroy_node()
    rclpy.shutdown()