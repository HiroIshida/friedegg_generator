# use to test trajectory saver

import rospy 
from geometry_msgs.msg import PoseStamped

rospy.init_node("dummy_pan_publisher")
pub_center = rospy.Publisher("pan_surface_center", PoseStamped, queue_size=1)

rate = rospy.Rate(10) # 10hz
while True:
    rate.sleep()
    msg_pose = PoseStamped()
    msg_pose.pose.orientation.w = 1.0
    point = msg_pose.pose.position
    point.x = 0.3
    point.y = 0.0
    point.z = 0.5
    pub_center.publish(msg_pose)
