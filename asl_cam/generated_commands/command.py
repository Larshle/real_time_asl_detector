#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def colline():
    rospy.init_node('colline_movement', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    move_cmd = Twist()
    
    # Move forward for a small distance
    move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
    move_cmd.angular.z = 0.0  # No rotation
    duration = 2  # Move forward for 2 seconds
    start_time = rospy.Time.now()

    while rospy.Time.now() - start_time < rospy.Duration(duration):
        pub.publish(move_cmd)
        rate.sleep()

    # Turn to create a line
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.sleep(0.5)  # Pause briefly

    # Move sideways to complete the line
    move_cmd.linear.x = 0.2  # Move forward at 0.2 m/s
    move_cmd.angular.z = 0.0
    duration = 2  # Move sideways for 2 seconds
    start_time = rospy.Time.now()

    while rospy.Time.now() - start_time < rospy.Duration(duration):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot after completing the command
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        colline()
    except rospy.ROSInterruptException:
        pass