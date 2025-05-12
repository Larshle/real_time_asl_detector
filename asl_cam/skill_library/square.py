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

"""
Shape: Line
Timestamp: 2023-10-12T10:00:00
Max Duration: 60s
Instruction: Line
"""

def drive_line():
    # Initialize node
    rospy.init_node('line_driver')
    
    # Create publisher
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Define straight line motion
    move_cmd = Twist()
    move_cmd.linear.x = min(SHAPE_SCALE * WHEEL_BASE_M, MAX_LINEAR_MPS)
    move_cmd.angular.z = 0

    # Define duration
    duration = 5  # seconds
    end_time = rospy.Time.now() + rospy.Duration(duration)

    # Publish move command for specified duration
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

def safe_stop():
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    stop_cmd = Twist()
    pub.publish(stop_cmd)  # sending zero velocities

def main():
    rospy.on_shutdown(safe_stop)
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown('Max duration reached'))
    
    drive_line()

    return None

if __name__ == '__main__':
    main()