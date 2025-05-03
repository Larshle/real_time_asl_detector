import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.circle()

    def circle(self):
        twist = Twist()
        twist.linear.x = 0.1  # Linear speed
        twist.angular.z = 0.1  # Angular speed for a circular path

        # Publish for a certain duration
        duration = 10  # seconds
        start_time = self.get_clock().now()

        while self.get_clock().now() - start_time < rclpy.duration.Duration(seconds=duration):
            self.publisher_.publish(twist)
            time.sleep(0.1)

        # Stop the robot after completing the circle
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    circle_movement = CircleMovement()
    rclpy.spin(circle_movement)
    circle_movement.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()