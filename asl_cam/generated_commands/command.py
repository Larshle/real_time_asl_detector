#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def drive_in_B():
    rospy.init_node('drive_in_B', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    move_cmd = Twist()

    # Segment 1: Move vertically up
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0
    t0 = rospy.Time.now().to_sec()
    distance_covered = 0
    while distance_covered < 2.0:
        pub.publish(move_cmd)
        t1 = rospy.Time.now().to_sec()
        distance_covered = 0.2 * (t1 - t0)
        rate.sleep()

    # Segment 2: Half circle top
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = 0.5
    t0 = rospy.Time.now().to_sec()
    angle_covered = 0
    while angle_covered < 3.14:
        pub.publish(move_cmd)
        t1 = rospy.Time.now().to_sec()
        angle_covered = 0.5 * (t1 - t0)
        rate.sleep()

    # Segment 3: Move vertically middle down
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rospy.sleep(1)

    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0
    t0 = rospy.Time.now().to_sec()
    distance_covered = 0
    while distance_covered < 1.0:
        pub.publish(move_cmd)
        t1 = rospy.Time.now().to_sec()
        distance_covered = 0.2 * (t1 - t0)
        rate.sleep()

    # Segment 4: Half circle bottom
    move_cmd.linear.x = 0.1
    move_cmd.angular.z = 0.5
    t0 = rospy.Time.now().to_sec()
    angle_covered = 0
    while angle_covered < 3.14:
        pub.publish(move_cmd)
        t1 = rospy.Time.now().to_sec()
        angle_covered = 0.5 * (t1 - t0)
        rate.sleep()

    # Segment 5: Move vertically lower end
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        drive_in_B()
    except rospy.ROSInterruptException:
        pass