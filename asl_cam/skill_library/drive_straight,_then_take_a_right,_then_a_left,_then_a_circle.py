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

# TurtleBot Line - 2023-11-20T15:00:00Z
# MAX_DURATION_S: 60
# Original instruction: Line

def main():
    rospy.init_node('turtlebot_shape_driver', anonymous=True)
    safe_stop = rospy.ServiceProxy('/stop', Empty)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.on_shutdown(safe_stop)

    drive_line(pub)

    rospy.spin()

def drive_line(pub):
    rate = rospy.Rate(10) # 10 Hz
    move_cmd = Twist()
    move_cmd.linear.x = min(MAX_LINEAR_MPS, SHAPE_SCALE * WHEEL_BASE_M)
    move_cmd.angular.z = 0.0

    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(MAX_DURATION_S):
        pub.publish(move_cmd)
        rate.sleep()
    rospy.signal_shutdown("Line complete.")

if __name__ == '__main__':
    main()