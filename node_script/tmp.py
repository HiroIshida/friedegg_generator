#!/usr/bin/env python
import numpy as np
import cv2

import rospy
import cv_bridge
from geometry_msgs.msg import PolygonStamped, Point32
from sensor_msgs.msg import PointCloud2
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import BoundingBoxArray
from sensor_msgs.msg import Image
import message_filters

tmp = {"msg1": None, "msg2": None}

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

def filter_by_label(msg_boxes, msg_class):
    boxes = []
    for a, b in zip(msg_boxes.boxes, msg_class.label_names):
        if b == "dish":
            boxes.append(msg_boxes)
    return boxes

def callback(msg_boxes, msg_class):
    boxes_filtered = filter_by_label(msg_boxes, msg_class)

msg_boxes_name = "/segmentation_decomposer_ssd/boxes"
msg_cloud_name = "/tf_transform_cloud/output"
msg_class_name = "/edgetpu_object_detector/output/class"

msg_boxes_type = BoundingBoxArray
msg_cloud_type = PointCloud2
msg_class_type = ClassificationResult

msg_names = [msg_boxes_name, msg_class_name]
type_names = [msg_boxes_type, msg_class_type]
subs = [message_filters.Subscriber(name, T) for name, T in zip(msg_names, type_names)]
ts = message_filters.ApproximateTimeSynchronizer(subs, 200, 0.2)
ts.registerCallback(callback)
#rospy.spin()
