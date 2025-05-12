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

# Drive in a straight line at a safe speed - 2023-11-24T16:47:00+00:00
# MAX_DURATION_S: 60
# Original Instruction: Line

def drive_line():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.linear.x = min(MAX_LINEAR_MPS, 0.1)
    rospy.Rate(10)  # 10 Hz

    start_time = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < MAX_DURATION_S:
        pub.publish(move_cmd)
        rospy.sleep(0.1)

def safe_stop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_cmd = Twist()
    pub.publish(stop_cmd)

def main():
    rospy.init_node('drive_pattern', anonymous=True)
    rospy.on_shutdown(safe_stop)
    drive_line()
    rospy.spin()

if __name__ == '__main__':
    main()