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

"""
Shape: Line
Timestamp: 2023-10-04T15:21:00
MAX_DURATION_S: 60
Original Instruction: Line
"""

def drive_line():
    rospy.loginfo("Driving in a straight line.")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = MAX_LINEAR_MPS
    twist.angular.z = 0

    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration.from_sec(MAX_DURATION_S):
        pub.publish(twist)
        rate.sleep()

    rospy.loginfo("Line complete.")

def safe_stop():
    rospy.loginfo("Stop the robot safely.")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()  # zero twist to stop
    pub.publish(twist)

def main():
    rospy.init_node('drive_shape')
    rospy.on_shutdown(safe_stop)

    # Start watchdog timer
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown("Max duration reached"))

    drive_line()
    rospy.spin()

if __name__ == '__main__':
    main()