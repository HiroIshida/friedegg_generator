import numpy as np
import rospy
import skrobot

from geometry_msgs.msg import PoseStamped

class Demo(object):
    def __init__(self):

        robot_model = skrobot.models.PR2()
        robot_model.reset_manip_pose()
        robot_model.reset_pose()

        link_names = ["r_shoulder_pan_link", "r_shoulder_lift_link", \
                "r_upper_arm_roll_link", "r_elbow_flex_link", \
                "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"]

        self.robot_model = robot_model
        self.link_list = [robot_model.__dict__[name] for name in link_names]
        self.joint_list = [link.joint for link in self.link_list]
        self.grasp_pose = None

        rospy.Subscriber("grasp_pose", PoseStamped, self._callback)

        self._target_pose = None

    def _callback(self, msg):
        print("rec")
        assert msg.header.frame_id == "base_link"
        pos = msg.pose.position
        ori = msg.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        orientation = np.array([ori.w, ori.x, ori.y, ori.z])
        rotmat = skrobot.coordinates.math.rotation_matrix_from_quat(orientation)
        target_pose = skrobot.coordinates.Coordinates(position, rotmat)
        self._target_pose = target_pose

        rarm_end_coords = skrobot.coordinates.CascadedCoords(
                parent=self.robot_model.r_gripper_tool_frame, 
                name='rarm_end_coords')
        res = self.robot_model.inverse_kinematics(target_pose, link_list=self.link_list, move_target=rarm_end_coords, rotation_axis=True)

    def debug_show(self):
        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(self.robot_model)
        viewer.show()


rospy.init_node("demo")
demo = Demo()
rospy.spin()




