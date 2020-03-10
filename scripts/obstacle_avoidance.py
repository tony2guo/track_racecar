#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


if __name__ == '__main__': 
    rospy.init_node('obstacle_avoidance')
    
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.pose.position.x = 3
    goal.target_pose.pose.orientation.w = 1
    while not rospy.is_shutdown():
        rospy.loginfo("send move_base goal")
        client.send_goal_and_wait(goal)