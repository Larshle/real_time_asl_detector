#!/usr/bin/env python
import math, sys, rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

# --- robot & motion limits (edit with care) -----------------
WHEEL_BASE_M      = 0.354     # distance between wheels (m)
MAX_LINEAR_MPS    = 0.22      # TurtleBot 2 safe limit
MAX_ANGULAR_RPS   = 2.84
SHAPE_SCALE       = 1         # multiplier × WHEEL_BASE_M
MAX_DURATION_S    = 60        # hard stop watchdog

# File: Line
# Timestamp: 2023-10-05T12:00:00
# MAX_DURATION_S: 60
# Original instruction: Line

def drive_line():
    rospy.loginfo("Driving in a straight line")
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.linear.x = MAX_LINEAR_MPS
    move_cmd.angular.z = 0.0

    rate = rospy.Rate(10)
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed_time = rospy.Time.now() - start_time
        if elapsed_time.to_sec() >= MAX_DURATION_S:
            rospy.signal_shutdown("Exceeded maximum duration")
        
        velocity_publisher.publish(move_cmd)
        rate.sleep()

def safe_stop():
    rospy.loginfo("Stopping the robot safely")
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_cmd = Twist()
    velocity_publisher.publish(stop_cmd)

def main():
    rospy.init_node('draw_line', anonymous=True)
    rospy.on_shutdown(safe_stop)
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown("Watchdog timer expired"), oneshot=True)
    
    drive_line()

    rospy.spin()

if __name__ == '__main__':
    main()