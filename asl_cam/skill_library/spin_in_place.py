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

# TurtleBot Line - 2023-10-05T12:00:00 - Duration: 60s
# User instruction: Line

def drive_line(length_m):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    move_cmd = Twist()
    move_cmd.linear.x = min(MAX_LINEAR_MPS, length_m / MAX_DURATION_S)
    move_cmd.angular.z = 0
    start_time = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - start_time) < (length_m / move_cmd.linear.x):
        pub.publish(move_cmd)
        rate.sleep()
    move_cmd.linear.x = 0
    pub.publish(move_cmd)

def safe_stop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_cmd = Twist()
    pub.publish(stop_cmd)

def main():
    rospy.init_node('turtlebot_shape_line', anonymous=True)
    rospy.on_shutdown(safe_stop)
    
    length_m = 1 * SHAPE_SCALE * WHEEL_BASE_M  # Edit to desired length
    drive_line(length_m)

    rospy.signal_shutdown('Shape completed or max duration reached')

if __name__ == '__main__':
    main()