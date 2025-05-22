#!/usr/bin/env python
import math, sys, rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

# --- robot & motion limits (edit with care) -----------------
WHEEL_BASE_M      = 0.354     # distance between wheels (m)
MAX_LINEAR_MPS    = 0.22      # TurtleBot 2 safe limit
MAX_ANGULAR_RPS   = 2.84
SHAPE_SCALE       = 1       # multiplier × WHEEL_BASE_M
MAX_DURATION_S    = 60        # hard stop watchdog

# TurtleBot 2 circle driving node
# 2023-10-09T12:00:00Z | MAX_DURATION_S = 60 | drive in a circle with 30 cm radius

def drive_circle(radius_m):
    # Create publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Calculate linear and angular speed
    linear_speed_mps = min(MAX_LINEAR_MPS, radius_m * MAX_ANGULAR_RPS)
    angular_speed_rps = linear_speed_mps / radius_m

    # Clamp angular speed
    angular_speed_rps = min(MAX_ANGULAR_RPS, angular_speed_rps)

    # Create Twist message
    twist = Twist()
    twist.linear.x = linear_speed_mps
    twist.angular.z = angular_speed_rps

    # Drive for the complete circle
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

def safe_stop():
    # Create publisher
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_msg = Twist()
    pub.publish(stop_msg)

def main():
    rospy.init_node('drive_circle_node')
    
    # Register shutdown hook
    rospy.on_shutdown(safe_stop)

    # Start watchdog to enforce max duration
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown('Max duration reached!'))

    # Drive in a circle of 30 cm radius
    drive_circle(0.3)

    # Spin to keep the script alive
    rospy.spin()

if __name__ == '__main__':
    main()