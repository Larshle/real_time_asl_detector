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
# Timestamp: 2023-11-20T13:00:00Z
# Max Duration (s): 60
# User Instruction: Line

def drive_line(length):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    
    # Calculate the time needed to move the specified distance at max speed
    duration = min(MAX_DURATION_S, length / MAX_LINEAR_MPS)
    
    move_cmd.linear.x = MAX_LINEAR_MPS
    move_cmd.angular.z = 0.0
    
    rate = rospy.Rate(10) # 10 Hz
    start_time = rospy.Time.now()
    
    while rospy.Time.now() - start_time < rospy.Duration(duration) and not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()
    
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)

def safe_stop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0
    pub.publish(stop_cmd)

def main():
    rospy.init_node('draw_shape_node')
    rospy.on_shutdown(safe_stop)
    
    length_of_line = 2.0  # Example length in meters, can be adjusted
    drive_line(length_of_line)
    
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown("Shape completed or max duration reached."), oneshot=True)
    rospy.spin()

if __name__ == '__main__':
    main()