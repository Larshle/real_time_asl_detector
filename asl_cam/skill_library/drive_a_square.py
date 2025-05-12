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

# File: line_shape.py
# Timestamp: 2023-11-05T17:30:00
# Max Duration: 60 s
# Instruction: Line

def drive_line(length_m):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    rate = rospy.Rate(10)
    
    # Clamp lengths
    length_m = max(0, min(length_m, MAX_LINEAR_MPS * MAX_DURATION_S))
    
    # Calculate time to drive straight given the max linear velocity
    duration = length_m / MAX_LINEAR_MPS
    
    # Set up linear motion
    twist.linear.x = MAX_LINEAR_MPS
    
    # Control loop
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration:
        pub.publish(twist)
        rate.sleep()
    
    # Stop the robot after moving the required distance
    twist.linear.x = 0.0
    pub.publish(twist)

def safe_stop():
    rospy.loginfo("Shutting down safely...")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

def main():
    rospy.init_node('draw_shape_node', anonymous=True)
    rospy.on_shutdown(safe_stop)
    
    # Watchdog timer to ensure the node doesn't run indefinitely
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown("Exceeded max duration"))

    # Drive in a line
    drive_line(SHAPE_SCALE * WHEEL_BASE_M * 5)

    rospy.spin()

if __name__ == '__main__':
    main()