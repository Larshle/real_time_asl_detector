import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CircleMover(Node):

    def __init__(self):
        super().__init__('circle_mover')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_circle()

    def move_circle(self):
        rate = self.create_rate(10)
        move_cmd = Twist()
        move_cmd.linear.x = 0.2  # Forward speed
        move_cmd.angular.z = 0.2  # Rotation speed

        # Drive in a circle for 20 seconds
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).seconds < 20:
            self.publisher.publish(move_cmd)
            rate.sleep()

        # Stop the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    circle_mover = CircleMover()
    rclpy.spin(circle_mover)
    circle_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()