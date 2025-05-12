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
Shape: LIMN
Timestamp: 2023-10-05T15:32:00Z
Duration limit: 60 s
Instruction: LIMN
"""

def drive_L():
    rospy.loginfo("Driving in the shape of L")
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(10)
    
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(3.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

    move.linear.x = 0
    move.angular.z = min(MAX_ANGULAR_RPS, math.pi / 2)
    for _ in range(int((math.pi / 2) / (0.1 * move.angular.z))):
        cmd_vel.publish(move)
        rate.sleep()

    move.angular.z = 0
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

def drive_I():
    rospy.loginfo("Driving in the shape of I")
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(10)
    
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

def drive_M():
    rospy.loginfo("Driving in the shape of M")
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(10)

    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

    move.linear.x = 0
    move.angular.z = min(MAX_ANGULAR_RPS, -math.pi / 4)
    for _ in range(int((math.pi / 4) / (0.1 * abs(move.angular.z)))):
        cmd_vel.publish(move)
        rate.sleep()

    move.angular.z = 0
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

    move.linear.x = 0
    move.angular.z = min(MAX_ANGULAR_RPS, math.pi / 4)
    for _ in range(int((math.pi / 4) / (0.1 * move.angular.z))):
        cmd_vel.publish(move)
        rate.sleep()

    move.angular.z = 0
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

def drive_N():
    rospy.loginfo("Driving in the shape of N")
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    rate = rospy.Rate(10)
    
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

    move.linear.x = 0
    move.angular.z = min(MAX_ANGULAR_RPS, -math.pi / 4)
    for _ in range(int((math.pi / 4) / (0.1 * abs(move.angular.z)))):
        cmd_vel.publish(move)
        rate.sleep()

    move.angular.z = 0
    move.linear.x = min(MAX_LINEAR_MPS, 0.2)
    for _ in range(int(1.0 / 0.1)):
        cmd_vel.publish(move)
        rate.sleep()

def safe_stop():
    rospy.loginfo("Stopping the robot")
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move = Twist()
    cmd_vel.publish(move)
    rospy.sleep(1)

def main():
    rospy.init_node('draw_limn', anonymous=True)
    rospy.on_shutdown(safe_stop)
    rospy.Timer(rospy.Duration(MAX_DURATION_S), lambda event: rospy.signal_shutdown("Max duration exceeded"), oneshot=True)
    
    drive_L()
    drive_I()
    drive_M()
    drive_N()

    sys.exit(0)

if __name__ == '__main__':
    main()