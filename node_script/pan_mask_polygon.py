#!/usr/bin/env python
import numpy as np
import cv2

import rospy
import cv_bridge
from geometry_msgs.msg import PolygonStamped, Point32
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image
import message_filters

tmp = {"img": None, "mask": None}

rospy.init_node("create_pan_mask_image")
pub = rospy.Publisher("pan_mask_polygon", PolygonStamped, queue_size=1)

def convert_rect_to_polygon(rect, header):
    poly_msg = PolygonStamped()
    poly_msg.header = header
    pt0 = Point32(x=rect.x, y=rect.y)
    pt1 = Point32(x=rect.x, y=rect.y + rect.height)
    pt2 = Point32(x=rect.x + rect.width, y=rect.y + rect.height)
    pt3 = Point32(x=rect.x + rect.width, y=rect.y)
    poly_msg.polygon.points.append(pt0)
    poly_msg.polygon.points.append(pt1)
    poly_msg.polygon.points.append(pt2)
    poly_msg.polygon.points.append(pt3)
    return poly_msg

def callback(msg_class, msg_rectarr, msg_image):
    if len(msg_rectarr.rects)!=1:
        # TODO use class or image to extract corresponding rect
        return
    rect = msg_rectarr.rects[0]
    poly_msg = convert_rect_to_polygon(rect, msg_image.header)
    pub.publish(poly_msg)

msg_class_name = "/edgetpu_object_detector/output/class"
msg_rectarr_name = "/edgetpu_object_detector/output/rects"
msg_image_name ="/kinect_head/rgb/image_rect_color"
msg_names = [msg_class_name, msg_rectarr_name, msg_image_name]
type_names = [ClassificationResult, RectArray, Image]
subs = [message_filters.Subscriber(name, T) for name, T in zip(msg_names, type_names)]
ts = message_filters.ApproximateTimeSynchronizer(subs, 200, 0.2)
ts.registerCallback(callback)
rospy.spin()
