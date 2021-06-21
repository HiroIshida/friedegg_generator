import time
import copy
import numpy as np
import rospy
import skrobot
import math
from skrobot.model.primitives import Axis
from skrobot.planner.utils import set_robot_config, get_robot_config
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

def translate_robot(robot_model, joint_list, trans):
    assert len(trans)==2
    av_current = get_robot_config(robot_model, joint_list, with_base=True)
    av_next = copy.copy(av_current)
    av_next[-3:-1] += np.array(trans)
    set_robot_config(robot_model, joint_list, av_next, with_base=True)
    return av_current, av_next

class Demo(object):
    def __init__(self):

        robot_model = skrobot.models.PR2()
        robot_model.reset_manip_pose()

        link_names = ["r_shoulder_pan_link", "r_shoulder_lift_link", \
                "r_upper_arm_roll_link", "r_elbow_flex_link", \
                "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"]
        ri = skrobot.interfaces.ros.PR2ROSRobotInterface(robot_model)

        rarm_end_coords = skrobot.coordinates.CascadedCoords(
                parent=robot_model.r_gripper_tool_frame, 
                name='rarm_end_coords')

        self.rarm_end_coords = rarm_end_coords
        self.robot_model = robot_model
        self.ri = ri
        self.link_list = [robot_model.__dict__[name] for name in link_names]
        self.joint_list = [link.joint for link in self.link_list]
        self.grasp_pose = None


        rospy.Subscriber("grasp_pose", PoseStamped, self._callback_grasp_pose)
        rospy.Subscriber("pan_surface_center", PointStamped, self._callback_point)

        # set in _callback_grasp_pose
        self._target_grasp_pose = None
        self._target_pose = None

        # set in _callback_point
        self._pan_surface_center = None

    def _callback_grasp_pose(self, msg):
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

    def _callback_point(self, msg):
        point = np.array([msg.point.x, msg.point.y, msg.point.z])
        self._pan_surface_center = point

    def obtain_current_angle_vector(self):
        self.robot_model.angle_vector(self.ri.angle_vector())
        return [j.joint_angle() for j in self.joint_list]

    def reset_robot(self):
        self.robot_model.reset_manip_pose()
        self.robot_model.r_shoulder_lift_joint.joint_angle(-0.5)
        self.robot_model.l_shoulder_lift_joint.joint_angle(-0.5)
        self.robot_model.torso_lift_joint.joint_angle(0.35)
        self.robot_model.head_tilt_joint.joint_angle(0.9)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.5, time_scale=1.0)

    def set_rarm(self, angles):
        assert len(angles) == 7
        joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        for (jn, an) in zip(joint_names, angles):
            joint = self.robot_model.__dict__[jn]
            joint.joint_angle(an)

    def grasp(self, debug=False):
        self.ri.move_gripper("rarm", pos=0.04)
        angles = np.array([-28.31, -5.975, -111.2, -56.61, -53.92, -63.41, -172.148]) * math.pi/180.0
        self.set_rarm(angles)
        res = self.robot_model.inverse_kinematics(self._target_pose, link_list=self.link_list, move_target=self.rarm_end_coords, rotation_axis=True)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.5, time_scale=1.0)
        self.ri.wait_interpolation()

        res = self.robot_model.inverse_kinematics(self._target_grasp_pose, link_list=self.link_list, move_target=self.rarm_end_coords, rotation_axis=True)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=1.0, time_scale=1.0)
        self.ri.wait_interpolation()
        self.ri.move_gripper("rarm", pos=0.0)

        if debug:
            viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
            viewer.add(self.robot_model)
            viewer.add(Axis.from_coords(self._target_grasp_pose))
            viewer.show()

    def move_to_pan(self, use_real_robot=False, debug=False, angles=[90, 45, 90]):
        # DEBUG!!!!!
        self._pan_surface_center = np.array([0.7908201515525104, 0.23566020955566158, 0.9293659055940292])

        assert self._pan_surface_center is not None, "reaching target is not set yet"

        rarm_angles = [0.12, 0.044, -1.28, -1.198, -1.5, -1.67, -3.38]
        (j.joint_angle(a) for j, a in zip(self.joint_list, rarm_angles))

        offset = np.array([0.0, 0.1, 0.1])
        position = self._pan_surface_center + offset

        if use_real_robot:
            #demo.ri.go_pos_unsafe_no_wait(*[0.0, 0.5, 0.0])
            pass
        else:
            translate_robot(self.robot_model, self.joint_list, [0.1, 0.5])

        def solve_ik(angles):
            r = R.from_euler('xyz', angles, degrees=True)
            x, y, z, w = r.as_quat() # TODO can be simpler by using as_matrix
            orientation = np.array([w, x, y, z])
            rotmat = skrobot.coordinates.math.rotation_matrix_from_quat(orientation)
            prepare_pose = skrobot.coordinates.Coordinates(position, rotmat)


            res = self.robot_model.inverse_kinematics(prepare_pose, link_list=self.link_list, move_target=self.rarm_end_coords, rotation_axis=True)

        for a in [60, 30, 0, -30, -60, -90]:
            solve_ik([a, 45, 30])

            if use_real_robot:
                self.ri.angle_vector(self.robot_model.angle_vector(), time=0.5, time_scale=1.0)
                time.sleep(0.3)

        solve_ik([-90, -10, 30])
        if use_real_robot:
            self.ri.angle_vector(self.robot_model.angle_vector(), time=0.5, time_scale=1.0)
            self.ri.wait_interpolation()

        if debug:
            viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
            viewer.add(self.robot_model)
            viewer.add(Axis.from_coords(prepare_pose))
            viewer.show()

try:
    demo
except:
    rospy.init_node("demo")
    demo = Demo()

demo.reset_robot()
#demo.grasp()
#demo.move_to_pan(use_real_robot=True)

"""
demo.move_to_pan(angles=[90, 45, 30], slide=True, debug=False)
demo.ri.angle_vector(demo.robot_model.angle_vector(), time=2.5, time_scale=1.0)
time.sleep(3)

for a in [60, 30, 0, -30, -60, -90]:
    demo.move_to_pan(angles=[a, 45, 30], slide=False, debug=False)
    demo.ri.angle_vector(demo.robot_model.angle_vector(), time=0.5, time_scale=1.0)
    time.sleep(0.3)

demo.move_to_pan(angles=[-90, 0, 30], slide=False, debug=False)
demo.ri.angle_vector(demo.robot_model.angle_vector(), time=2.5, time_scale=1.0)
time.sleep(3)
"""