#!/usr/bin/env python
import rospy
from posedetection_msgs.msg import ObjectDetection

rospy.init_node('dummy_listener', anonymous=True)

topic_name_detection = "/kitchen_finder/ObjectDetection"
def cb_object_detection(msg):
    print(msg)
sub = rospy.Subscriber(topic_name_detection, ObjectDetection, cb_object_detection)
rospy.spin()


