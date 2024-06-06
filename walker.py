
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard
import threading
import time

class Walker(Node):
    def __init__(self):
        super().__init__('walker')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.moving = False

        # Start the keyboard listener in a separate thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def keyboard_listener(self):
        while True:
            if keyboard.is_pressed('w'):
                self.linear_speed = 1.0
                self.angular_speed = 0.0
                self.moving = True
            # elif keyboard.is_pressed('s'):
            #     self.linear_speed = -1.0
            #     self.angular_speed = 0.0
            #     self.moving = True
            # elif keyboard.is_pressed('a'):
            #     self.linear_speed = 0.0
            #     self.angular_speed = 1.0
            #     self.moving = True
            # elif keyboard.is_pressed('d'):
            #     self.linear_speed = 0.0
            #     self.angular_speed = -1.0
            #     self.moving = True
            # elif keyboard.is_pressed('q'):
            #     self.linear_speed = 0.5
            #     self.angular_speed = 0.5
            #     self.moving = True
            # elif keyboard.is_pressed('e'):
            #     self.linear_speed = 0.5
            #     self.angular_speed = -0.5
            #     self.moving = True
            # else:
            #     self.moving = False
            
            # if self.moving:
            #     self.move_for_duration(5)
            #     self.moving = False
            
            time.sleep(0.1)

    def move_for_duration(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Stop the robot after moving
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    walker = Walker()
    try:
        rclpy.spin(walker)
    except KeyboardInterrupt:
        pass

    walker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()