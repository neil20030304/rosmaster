import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.is_moving = True

    def timer_callback(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        twist = Twist()
        if elapsed_time < 2.0:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.0
            self.is_moving = False

        self.publisher_.publish(twist)

        if not self.is_moving:
            self.get_logger().info('Stopping the robot.')
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    twist_publisher = TwistPublisher()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()