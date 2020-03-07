#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, String
from darknet_ros_msgs.msg import BoundingBoxes
import dynamic_reconfigure.client

def timer_cb(event=None):
    global stop_count, no_stop_count, already_stopped, last_speed, speed_limit

    if "speed_limit_30" in flags:
        speed_limit = 30
    elif "speed_limit_60" in flags:
        speed_limit = 60
    speed_limit_pub.publish(speed_limit)

    speed = speed_limit
    state = "none"
    if "traffic_light_red" in flags:
        speed = 0
        state = "traffic light red"
    elif "traffic_light_yellow" in flags:
        speed = speed_limit * 0.5
        state = "traffic light yellow"
    elif "traffic_light_green" in flags:
        state = "traffic light green"
    
    if "stop_sign" in flags:
        if stop_count < stop_second * 10 and not already_stopped:
            stop_count += 1
            speed = 0
            state = "stop sign"
        else:
            stop_count = 0
            already_stopped = True
    else:
        if no_stop_count < ignore_stop_second * 10 and stop_count != 0:
            stop_count += 1
            no_stop_count += 1
            speed = 0
            state = "stop sign"
        else:
            stop_count = 0
            no_stop_count = 0
            already_stopped = False

    
    state_pub.publish(state)
    
    if speed != last_speed:
        last_speed = speed
        speed_lim_vx = speed * coefficient
        client.update_configuration({"speed_lim_vx": speed_lim_vx})

def bounding_boxes_cb(msg):
    global flags
    
    flags = {}
    for item in msg.bounding_boxes:
        if item.xmin >= detection_constraint[item.Class]['xmin'] and item.xmax <= detection_constraint[item.Class]['xmax'] and item.ymin >= detection_constraint[item.Class]['ymin'] and item.ymax <= detection_constraint[item.Class]['ymax']:
            if detection_constraint[item.Class]['area_min'] <= (item.xmax - item.xmin) * (item.ymax - item.ymin) <= detection_constraint[item.Class]['area_max']:
                flags.setdefault(item.Class, 0)
                flags[item.Class] += 1
    
    timer_cb()
    
if __name__ == '__main__': 
    rospy.init_node('class_to_speed')
    coefficient = rospy.get_param('~coefficient', 1000.0 / 60.0 / 60.0)
    speed_limit = rospy.get_param('~speed_limit', 100)
    stop_second = rospy.get_param('~stop_second', 6)
    ignore_stop_second = rospy.get_param('~ignore_stop_second', 3)
    detection_constraint = rospy.get_param('~detection_constraint')
    flags = {}
    last_speed = speed_limit
    stop_count = 0
    no_stop_count = 0
    already_stopped = False
    client = dynamic_reconfigure.client.Client("cob_base_velocity_smoother", timeout=30)
    speed_limit_pub = rospy.Publisher("~speed_limit", Int16, queue_size=1, latch=True)
    state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_cb)
    rospy.Timer(rospy.Duration(0.1), timer_cb)
    rospy.spin()