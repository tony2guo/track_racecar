#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def image_cb(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    pub_vel = Twist()
    height = cv_image.shape[0]
    width = cv_image.shape[1]

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    blur_image = cv2.GaussianBlur(gray_image, (13, 13), 0)
    canny_image = cv2.Canny(blur_image, 50, 150)
    polygons = np.array([
        [(0, height), (width, height), (width, height//2), (0, height//2)]
        ])
    mask = np.zeros_like(canny_image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(canny_image, mask)

    lines = cv2.HoughLinesP(masked_image, 1, np.pi / 180, 150, minLineLength = 20, maxLineGap = 20)
    if lines is not None:
        intersections = []
        line_count = len(lines)
        for index, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            (A, B) = (x1, y1), (x2, y2)
            cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            for compare_index in range(index+1, line_count):
                x1, y1, x2, y2 = lines[compare_index][0]
                (C, D) = (x1, y1), (x2, y2)
                intersection = line_intersection((A, B), (C, D))
                if intersection is not None:
                    if intersection[0] < width and intersection[0] > 0 and intersection[1] < height and intersection[1] > 0:
                        cv2.circle(cv_image, (int(intersection[0]), int(intersection[1])), 5, (0, 255, 0), 2)
                        intersections.append(intersection)
        if len(intersections) > 0:
            x, y = np.mean(intersections, axis=0)
            cv2.arrowedLine(cv_image, (width//2, height), (int(x), int(y)), (255, 0, 0), 2)
            pub_vel.linear.x = 1 - y / height
            pub_vel.angular.z = 0.5 - x / width
    else:
        rospy.logwarn("no lines")
    
    cmd_vel_pub.publish(pub_vel)
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        rospy.logerr(e)

if __name__ == '__main__': 
    rospy.init_node('line_follower')
    bridge = CvBridge()
    image_pub = rospy.Publisher("~image_raw", Image, queue_size=1)
    cmd_vel_pub = rospy.Publisher("~cmd_vel", Twist, queue_size=1)
    image_sub = rospy.Subscriber("/cv_camera/image_raw", Image, image_cb)
    rospy.spin()