import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_circle()

    def move_circle(self):
        msg = Twist()
        radius = 0.5  # meters
        speed = 0.3   # meters per second
        angular_speed = speed / radius
        
        # Set the linear and angular velocities
        msg.linear.x = speed
        msg.angular.z = angular_speed

        # Publish the move command for a full circle
        duration = 2 * math.pi * radius / speed  # time to complete the circle
        start_time = self.get_clock().now().nanoseconds

        while self.get_clock().now().nanoseconds - start_time < duration * 1e9:
            self.publisher_.publish(msg)
            time.sleep(0.1)

        # Stop the robot
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    circle_mover = CircleMover()
    rclpy.spin(circle_mover)
    circle_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()