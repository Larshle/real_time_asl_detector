```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_circle()

    def move_circle(self):
        twist = Twist()
        radius = 0.5  # radius of the circle in meters
        velocity = 0.2  # linear velocity in m/s
        angular_velocity = velocity / radius  # angular velocity

        # Duration for one full circle
        duration = (2 * math.pi * radius) / velocity

        # Set linear and angular velocities
        twist.linear.x = velocity
        twist.angular.z = angular_velocity

        # Publish the velocity command
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < duration * 1e9:
            self.publisher_.publish(twist)
            time.sleep(0.1)

        # Stop the robot after the circle
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
```