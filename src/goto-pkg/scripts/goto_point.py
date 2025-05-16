#!/usr/bin/env python
import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def goto_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server…")
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # convert yaw → quaternion
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    rospy.loginfo("Sending goal: x=%.2f y=%.2f yaw=%.2f", x, y, yaw)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('goto_point')
    # read from ROS params (so you can pass via rosrun/roslaunch)
    x   = rospy.get_param('~goal_x',   1.0)
    y   = rospy.get_param('~goal_y',   1.0)
    yaw = rospy.get_param('~goal_yaw', 0.0)
    result = goto_goal(x, y, yaw)
    if result:
        rospy.loginfo("Arrived at goal!")
    else:
        rospy.logwarn("Failed to reach goal.")
