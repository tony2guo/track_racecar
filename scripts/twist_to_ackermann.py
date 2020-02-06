#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
import math

def cmd_vel_cb(msg):
    pub_msg = AckermannDrive()
    pub_msg.speed = msg.linear.x
    if msg.linear.x != 0 and msg.angular.z != 0:
        turn_radius = msg.linear.x / msg.angular.z
        pub_msg.steering_angle = math.atan(front_rear_distance/turn_radius)
    ackermann_pub.publish(pub_msg)

if __name__ == '__main__': 
    rospy.init_node('twist_to_ackermann')
    
    front_rear_distance = rospy.get_param('~front_rear_distance', 1.0)
    ackermann_pub = rospy.Publisher("ackermann", AckermannDrive, queue_size = 1)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_cb)
    rospy.spin()