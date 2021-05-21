#!/usr/bin/env python
import time

from scipy.spatial.transform import Rotation as R

import rospy
import ros_numpy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
import sensor_msgs.point_cloud2 as pc2

class GraspPoseDetector(object):
    frame_id = "base_link"

    def __init__(self):
        pcloud_edge_name = "/organized_edge_detector/output"
        boxes_name = "/HSI_color_filter/boxes"
        sub1 = message_filters.Subscriber(pcloud_edge_name, PointCloud2)
        sub2 = message_filters.Subscriber(boxes_name, BoundingBoxArray)
        ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 200, 0.2)
        ts.registerCallback(self.callback)

        self.pub = rospy.Publisher("grasp_pose", PoseStamped, queue_size=1)

        # varibales 
        self._header = None
        self._grasp_pose = None

        # variables for dbeugging
        self._pts_array = None

    def callback(self, pcloud_edge, boxes):
        assert pcloud_edge.header.frame_id == GraspPoseDetector.frame_id
        assert boxes.header.frame_id == GraspPoseDetector.frame_id
        self._header = pcloud_edge.header

        if len(boxes.boxes) != 1:
            return
        box = boxes.boxes[0]
        x_center, y_center, z_center = box.pose.position.x, box.pose.position.y, box.pose.position.z
        x_width, y_width, z_width = box.dimensions.x, box.dimensions.y, box.dimensions.z
        b_min = np.array([x_center - x_width * 0.5, y_center - y_width * 0.5, z_center - z_width * 0.5])
        b_max = np.array([x_center + x_width * 0.5, y_center + y_width * 0.5, z_center + z_width * 0.5])
        dish_center = np.array([box.pose.position.x, box.pose.position.y, b_min[2]])

        pts_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcloud_edge)
        def predicate(pt):
            b_width = b_max - b_min
            b_min_margined = b_min - b_width * 0.1
            b_max_margined = b_max + b_width * 0.1
            for i in range(2):
                if pt[i] < b_min_margined[i]:
                    return False
                if pt[i] > b_max_margined[i]:
                    return False
            if pt[2] < b_max[2] - z_width * 0.3:
                return False
            return True

        logicals = np.array([predicate(pt) for pt in pts_array])
        upper_edge_points = pts_array[logicals]

        idx_y_min = np.argmin(upper_edge_points[:, 1])

        grasp_pose = upper_edge_points[idx_y_min]
        self._grasp_pose = grasp_pose # TODO actually it's a position

        self._pts_array = pts_array[logicals]
        self._publish_grasp_pose()

    def _publish_grasp_pose(self):
        msg = PoseStamped()
        msg.header = self._header
        pos = msg.pose.position
        pos.x, pos.y, pos.z = self._grasp_pose

        r = R.from_euler('xyz', [90, 45, 90], degrees=True)
        rot = msg.pose.orientation
        rot.x, rot.y, rot.z, rot.w = r.as_quat()
        self.pub.publish(msg)

    def debug_show_points(self):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = fig.add_subplot(211 , projection='3d')

        pts = _pts_array
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
        plt.show()


if __name__=="__main__":
    rospy.init_node("grasp_pose_detector")
    rate = rospy.Rate(10)
    gpd = GraspPoseDetector()
    rospy.spin()
