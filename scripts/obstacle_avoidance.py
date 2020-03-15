#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist


def nav_vel_cb(msg):
    if abs(msg.angular.z) > angular_threshold:
        cmd_vel_pub.publish(msg)

if __name__ == '__main__': 
    rospy.init_node('obstacle_avoidance')

    angular_threshold = rospy.get_param('~angular_threshold', 0.1)

    cmd_vel_pub = rospy.Publisher("~cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("nav_vel", Twist, nav_vel_cb)

    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.pose.position.x = 4
    goal.target_pose.pose.orientation.w = 1
    while not rospy.is_shutdown():
        rospy.loginfo("send move_base goal")
        client.send_goal_and_wait(goal)