#!/usr/bin/env python
import time
import numpy as np
from scipy.spatial import ConvexHull
import ros_numpy
import cv2

import rospy
import cv_bridge
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped
from sensor_msgs.msg import PointCloud2
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import RectArray
from jsk_recognition_msgs.msg import BoundingBoxArray
from sensor_msgs.msg import Image
import message_filters

tmp = {"msg1": None, "msg2": None}

rospy.init_node("create_pan_surface_polygon")
pub = rospy.Publisher("pan_surface_polygon", PolygonStamped, queue_size=1)
pub_center = rospy.Publisher("pan_surface_center", PointStamped, queue_size=1)

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

def fileter_box_by_label(msg_boxes, msg_class):
    logicals = [label_name=="dish" for label_name in msg_class.label_names]
    indexes = np.where(logicals)[0].tolist()
    return [msg_boxes.boxes[i] for i in indexes]

def find_bigest_box(boxes, threshold=0.004):
    # threshold = 0.2 * 0.2 * 0.1 by default
    get_size = lambda box: box.dimensions.x * box.dimensions.y * box.dimensions.z
    sizes = [get_size(box) for box in boxes]
    return boxes[np.argmax(sizes)]

def assert_orientation_condition(box):
    quat_ = box.pose.orientation
    quat = [quat_.x, quat_.y, quat_.z, quat_.w]
    assert quat == [1., 0., 0., 0.]

def filter_point_inside_box(pts, box):
    assert_orientation_condition(box) # TODO currently only handle 1, 0, 0, 0 case
    pos_ = box.pose.position
    pos = np.array([pos_.x, pos_.y, pos_.z])
    dim_ = box.dimensions
    dim = np.array([dim_.x, dim_.y, dim_.z])
    low = pos - dim * 0.5
    hi = pos + dim * 0.5
    indexes_inside = np.all(np.logical_and(low <= pts, pts <= hi), axis=1)
    return pts[indexes_inside]

def compute_pan_surface_polygon(pts):
    z_array = pts[:, 2]
    z_mean = np.mean(z_array)
    z_std = np.std(z_array)
    logicals = np.array([pt[2] > z_mean + z_std for pt in pts])
    pts_filtered = pts[logicals]

    hull = ConvexHull(pts_filtered[:, :2])
    verts_ = hull.points[hull.vertices]
    z_filtered_mean = np.mean(pts_filtered[:, 2])
    verts = np.hstack([verts_, np.ones((len(verts_), 1))*z_filtered_mean])
    return verts

def polygon_size_and_com(verts):
    s = 0.0
    n = len(verts)
    origin = verts[0]
    com = np.zeros(3)
    for i in range(n):
        v0 = verts[i] - origin
        v1 = verts[(i+1)%n] - origin
        s_triangle = np.linalg.norm(np.outer(v0, v1)) * 0.5
        com += s_triangle * (v0 + v1)/3.0
        s += s_triangle # green's theorem
    com = com/s + origin
    return s, com

def callback(msg_boxes, msg_class, msg_cloud):
    if len(msg_boxes.boxes) != len(msg_class.label_names):
        # NOTE workaround
        # TODO maybe this occurs due to sync error??
        rospy.loginfo("numbers of box and labels does not match (maybe async error)")
        return

    boxes_filtered = fileter_box_by_label(msg_boxes, msg_class)
    if len(boxes_filtered)==0:
        rospy.loginfo("No dish found")
        return
    box = find_bigest_box(boxes_filtered)

    pts = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg_cloud)
    pts_filtered = filter_point_inside_box(pts, box)

    msg_polygon = PolygonStamped(header=msg_cloud.header)
    verts = compute_pan_surface_polygon(pts_filtered)
    size, com = polygon_size_and_com(verts) # TODO check size is starnage!
    rospy.loginfo("computed pan size is {0}".format(size))
    rospy.loginfo("computed pan com is {0}".format(com.tolist()))
    if size < 0.3: # TODO adhoc parameter. we should fix size computation
        rospy.loginfo("publish canceled as it's too small")
        return  
    for v in verts:
        pt = Point32(x=v[0], y=v[1], z=v[2])
        msg_polygon.polygon.points.append(pt)
    pub.publish(msg_polygon)

    msg_point = PointStamped(header=msg_cloud.header)
    msg_point.point.x = com[0]
    msg_point.point.y = com[1]
    msg_point.point.z = com[2]
    pub_center.publish(msg_point)

msg_boxes_name = "/segmentation_decomposer_ssd/boxes"
msg_cloud_name = "/tf_transform_cloud/output"
msg_class_name = "/edgetpu_object_detector/output/class"

msg_boxes_type = BoundingBoxArray
msg_cloud_type = PointCloud2
msg_class_type = ClassificationResult

msg_names = [msg_boxes_name, msg_class_name, msg_cloud_name]
type_names = [msg_boxes_type, msg_class_type, msg_cloud_type]
subs = [message_filters.Subscriber(name, T) for name, T in zip(msg_names, type_names)]
ts = message_filters.ApproximateTimeSynchronizer(subs, 200, 0.2)
ts.registerCallback(callback)
rospy.spin()

