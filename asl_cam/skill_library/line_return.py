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

# Shape: Line
# Timestamp: 2023-10-09T12:00:00
# Duration limit: 60 seconds
# User Instruction: Line

def drive_line(distance_m):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    distance_traveled = 0.0
    start_time = rospy.Time.now()
    
    move_cmd = Twist()
    move_cmd.linear.x = min(MAX_LINEAR_MPS, distance_m)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        duration = (current_time - start_time).to_sec()
        if distance_traveled >= distance_m or duration > MAX_DURATION_S:
            break

        pub.publish(move_cmd)
        distance_traveled += move_cmd.linear.x * 0.1
        rate.sleep()

def safe_stop():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_cmd = Twist()
    pub.publish(stop_cmd)

def main():
    rospy.init_node('draw_line', anonymous=True)
    rospy.on_shutdown(safe_stop)
    
    if len(sys.argv) < 2:
        rospy.signal_shutdown("Usage: line.py <distance_m>")
        return None
    
    distance_m = float(sys.argv[1])
    drive_line(distance_m)
    return None

if __name__ == '__main__':
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown("Max duration reached"), oneshot=True)
    main()