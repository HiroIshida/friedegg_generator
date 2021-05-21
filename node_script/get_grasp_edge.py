import time

import rospy
import ros_numpy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
import sensor_msgs.point_cloud2 as pc2

class GraspPointDetector(object):
    frame_id = "base_link"

    def __init__(self):
        pcloud_edge_name = "/organized_edge_detector/output"
        boxes_name = "/HSI_color_filter/boxes"
        sub1 = message_filters.Subscriber(pcloud_edge_name, PointCloud2)
        sub2 = message_filters.Subscriber(boxes_name, BoundingBoxArray)
        ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 200, 0.2)
        ts.registerCallback(self.callback)

        self.pub = rospy.Publisher("grasp_point", PointStamped, queue_size=1)

        # varibales 
        self._header = None
        self._grasp_point = None

        # variables for dbeugging
        self._pts_array = None

    def callback(self, pcloud_edge, boxes):
        assert pcloud_edge.header.frame_id == GraspPointDetector.frame_id
        assert boxes.header.frame_id == GraspPointDetector.frame_id
        self._header = pcloud_edge.header

        if len(boxes.boxes) != 1:
            return
        box = boxes.boxes[0]
        z_center = box.pose.position.z
        z_width = box.dimensions.z
        z_lo = z_center - z_width * 0.5
        z_hi = z_center + z_width * 0.5

        pts_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcloud_edge)
        predicate = lambda pt : pt[2] > z_hi - z_width * 0.3
        logicals = np.array([predicate(pt) for pt in pts_array])
        upper_edge_points = pts_array[logicals]

        idx_x_min = np.argmin(upper_edge_points[:, 0])
        grasp_point = upper_edge_points[idx_x_min]
        self._grasp_point = grasp_point

        self._pts_array = pts_array[logicals]
        self._publish_grasp_point()

    def _publish_grasp_point(self):
        msg = PointStamped()
        msg.header = self._header
        msg.point.x, msg.point.y, msg.point.z = self._grasp_point
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
    rospy.init_node("grasp_point_detector")
    rate = rospy.Rate(10)
    gpd = GraspPointDetector()
    rospy.spin()
