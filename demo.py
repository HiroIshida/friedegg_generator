import time
import copy
import numpy as np
import rospy
import skrobot
from skrobot.model.primitives import Axis

from geometry_msgs.msg import PoseStamped

class Demo(object):
    def __init__(self):

        robot_model = skrobot.models.PR2()
        robot_model.reset_manip_pose()

        link_names = ["r_shoulder_pan_link", "r_shoulder_lift_link", \
                "r_upper_arm_roll_link", "r_elbow_flex_link", \
                "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"]
        ri = skrobot.interfaces.ros.PR2ROSRobotInterface(robot_model)

        self.robot_model = robot_model
        self.ri = ri
        self.link_list = [robot_model.__dict__[name] for name in link_names]
        self.joint_list = [link.joint for link in self.link_list]
        self.grasp_pose = None


        rospy.Subscriber("grasp_pose", PoseStamped, self._callback)

        self._target_grasp_pose = None
        self._target_pose = None

    def _callback(self, msg):
        assert msg.header.frame_id == "base_footprint"
        pos = msg.pose.position
        ori = msg.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        orientation = np.array([ori.w, ori.x, ori.y, ori.z])
        rotmat = skrobot.coordinates.math.rotation_matrix_from_quat(orientation)
        tmp = skrobot.coordinates.Coordinates(position, rotmat)

        target_grasp_pose = copy.deepcopy(tmp)
        target_grasp_pose.translate([0.02, 0, 0])
        self._target_grasp_pose = target_grasp_pose

        target_pose = copy.deepcopy(tmp)
        target_pose.translate([-0.05, 0, 0])
        self._target_pose = target_pose

    def reset_robot(self):
        self.robot_model.reset_manip_pose()
        self.robot_model.r_shoulder_lift_joint.joint_angle(-0.5)
        self.robot_model.l_shoulder_lift_joint.joint_angle(-0.5)
        self.robot_model.torso_lift_joint.joint_angle(0.25)
        self.robot_model.head_tilt_joint.joint_angle(0.9)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.5, time_scale=1.0)

    def grasp(self, debug=False):
        self.ri.move_gripper("rarm", pos=0.04)

        rarm_end_coords = skrobot.coordinates.CascadedCoords(
                parent=self.robot_model.r_gripper_tool_frame, 
                name='rarm_end_coords')
        res = self.robot_model.inverse_kinematics(self._target_pose, link_list=self.link_list, move_target=rarm_end_coords, rotation_axis=True)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.5, time_scale=1.0)
        self.ri.wait_interpolation()

        res = self.robot_model.inverse_kinematics(self._target_grasp_pose, link_list=self.link_list, move_target=rarm_end_coords, rotation_axis=True)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=1.0, time_scale=1.0)
        self.ri.wait_interpolation()
        self.ri.move_gripper("rarm", pos=0.0)

        if debug:
            viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
            viewer.add(self.robot_model)
            viewer.add(Axis.from_coords(self._target_grasp_pose))
            viewer.show()

rospy.init_node("demo")
demo = Demo()
#rospy.spin()




