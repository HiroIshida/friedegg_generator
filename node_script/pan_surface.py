#!/usr/bin/env python
import numpy as np
from scipy.spatial import ConvexHull
import ros_numpy
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

tmp = {"hull": None}

pub = rospy.Publisher("pan_surface", PolygonStamped, queue_size=1)

def callback(msg_cloud):
    pts_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg_cloud)
    z_array = pts_array[:, 2]
    z_mean = np.mean(z_array)
    z_std = np.std(z_array)
    logicals = np.array([pt[2] > z_mean + z_std for pt in pts_array])
    pts_filtered = pts_array[logicals]

    hull = ConvexHull(pts_filtered[:, :2])
    V = hull.points[hull.vertices]
    z_filtered_mean = np.mean(pts_filtered[:, 2])

    msg_polygon = PolygonStamped(header=msg_cloud.header)
    for v in V:
        pt = Point32(x=v[0], y=v[1], z=z_filtered_mean)
        msg_polygon.polygon.points.append(pt)

    pub.publish(msg_polygon)

rospy.init_node("pan_surface_detector")
rospy.Subscriber("/tf_transform_cloud/output", PointCloud2, callback)
rospy.spin()


