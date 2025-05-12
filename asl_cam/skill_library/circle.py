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

# File: Line.py | Timestamp: 2023-10-03T14:03:00 | MAX_DURATION_S: 60
# Instruction: Line

def drive_line(length):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    move_cmd = Twist()
    move_cmd.linear.x = min(length / MAX_DURATION_S, MAX_LINEAR_MPS)  # Ensure not to exceed limits
    
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(length / move_cmd.linear.x):
        pub.publish(move_cmd)
        rate.sleep()

def safe_stop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    stop_cmd = Twist()
    pub.publish(stop_cmd)

def main():
    rospy.init_node('turtlebot_line', anonymous=True)
    rospy.on_shutdown(safe_stop)
    
    try:
        drive_line(WHEEL_BASE_M * SHAPE_SCALE)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__ == '__main__':
    main()