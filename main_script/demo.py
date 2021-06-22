import time
import copy
import numpy as np
import rospy
import skrobot
import math
from skrobot.model.primitives import Axis
from skrobot.planner.utils import set_robot_config, get_robot_config
from skrobot.coordinates import make_coords, rpy_matrix, matrix2quaternion
from skrobot.coordinates.math import wxyz2xyzw, xyzw2wxyz
from skrobot.model import Axis

from scipy.spatial.transform import Rotation as R


from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped

from reproduce_trajectory import Reproducer, convert

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

        tmp = ["_shoulder_pan_link", "_shoulder_lift_link", \
                "_upper_arm_roll_link", "_elbow_flex_link", \
                "_forearm_roll_link", "_wrist_flex_link", "_wrist_roll_link"]
        link_names = ["r" + e for e in tmp]
        larm_link_names = ["l" + e for e in tmp]
        ri = skrobot.interfaces.ros.PR2ROSRobotInterface(robot_model)

        rarm_end_coords = skrobot.coordinates.CascadedCoords(
                parent=robot_model.r_gripper_tool_frame, 
                name='rarm_end_coords')
        larm_end_coords = skrobot.coordinates.CascadedCoords(
                parent=robot_model.l_gripper_tool_frame, 
                name='larm_end_coords')

        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(robot_model)

        self.rarm_end_coords = rarm_end_coords
        self.larm_end_coords = larm_end_coords
        self.robot_model = robot_model
        self.viewer = viewer
        self.ri = ri
        self.link_list = [robot_model.__dict__[name] for name in link_names] # TODO to rarm_link_list
        self.larm_link_list = [robot_model.__dict__[name] for name in larm_link_names]
        self.joint_list = [link.joint for link in self.link_list]
        self.grasp_pose = None

        rospy.Subscriber("grasp_pose", PoseStamped, self._callback_grasp_pose)
        rospy.Subscriber("pan_surface_center", PoseStamped, self._callback_pan_center)
        rospy.Subscriber("/kitchen_finder/object_pose", PoseStamped, self._callback_switch_pose)

        # set in _callback_grasp_pose
        self._target_grasp_pose = None
        self._target_pose = None

        # set in _callback_point
        self._pan_surface_center = None

        # set in _callback_switch_pose
        self._switch_coords = None

    def _callback_grasp_pose(self, msg):
        assert msg.header.frame_id == "base_footprint"
        pos = msg.pose.position
        ori = msg.pose.orientation
        position = np.array([pos.x, pos.y, pos.z])
        orientation = np.array([ori.w, ori.x, ori.y, ori.z])
        rotmat = skrobot.coordinates.math.rotation_matrix_from_quat(orientation)
        tmp = skrobot.coordinates.Coordinates(position, rotmat)

        target_grasp_pose = copy.deepcopy(tmp)
        target_grasp_pose.translate([0.03, 0, 0])
        self._target_grasp_pose = target_grasp_pose

        target_pose = copy.deepcopy(tmp)
        target_pose.translate([-0.05, 0, 0])
        self._target_pose = target_pose

    def _callback_pan_center(self, msg):
        pose = msg.pose 
        pos = [pose.position.x, pose.position.y, pose.position.z + 0.1]
        ori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        self._pan_surface_center = [pos, ori]

    def _callback_switch_pose(self, msg):
        pos, ori = msg.pose.position, msg.pose.orientation
        T_switch_to_opt = [[pos.x, pos.y, pos.z], [ori.x, ori.y, ori.z, ori.w]]

        # cause skrobt's coordinte transform is bit complicated, I use my own 
        # NOTE mine is xyzw but skrobot's is wxyz
        opt = self.robot_model.__dict__["narrow_stereo_optical_frame"].copy_worldcoords()
        rot = wxyz2xyzw(matrix2quaternion(opt.rotation))
        T_opt_to_base = [opt.translation, rot.tolist()]
        T_switch_to_base = convert(T_switch_to_opt, T_opt_to_base)

        # convert to skrobot coords
        pos, rot = T_switch_to_base
        pos[2] = 0.74 # Because we know the height of the switch hahaha
        rotmat = skrobot.coordinates.math.rotation_matrix_from_quat(xyzw2wxyz(rot))
        coords = skrobot.coordinates.Coordinates(pos, rotmat)
        self._switch_coords = coords

    def obtain_current_angle_vector(self):
        self.robot_model.angle_vector(self.ri.angle_vector())
        return [j.joint_angle() for j in self.joint_list]

    def reset_robot(self, torso_angle=0.35):
        self.robot_model.reset_manip_pose()
        self.robot_model.r_shoulder_lift_joint.joint_angle(-0.5)
        self.robot_model.l_shoulder_lift_joint.joint_angle(-0.5)
        self.robot_model.torso_lift_joint.joint_angle(torso_angle)
        #self.robot_model.head_tilt_joint.joint_angle(0.9)
        self.robot_model.head_tilt_joint.joint_angle(42.396 * math.pi/180.0)
        self.robot_model.head_pan_joint.joint_angle(6.6787 * math.pi/180.0)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.5, time_scale=1.0)
        self.ri.move_gripper("rarm", pos=0.07)
        self.ri.move_gripper("larm", pos=0.0)
        #6.6487 42.396

    def set_rarm(self, angles):
        assert len(angles) == 7
        joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        for (jn, an) in zip(joint_names, angles):
            joint = self.robot_model.__dict__[jn]
            joint.joint_angle(an)

    def set_larm(self, angles):
        assert len(angles) == 7
        joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        for (jn, an) in zip(joint_names, angles):
            joint = self.robot_model.__dict__[jn]
            joint.joint_angle(an)

    def grasp(self, debug=False):
        self.ri.move_gripper("rarm", pos=0.07)
        angles = np.array([-28.31, -5.975, -111.2, -56.61, -53.92, -63.41, -172.148]) * math.pi/180.0
        self.set_rarm(angles)
        res = self.robot_model.inverse_kinematics(self._target_pose, link_list=self.link_list, move_target=self.rarm_end_coords, rotation_axis=True)
        assert res is not False

        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.5, time_scale=1.0)
        self.ri.wait_interpolation()

        res = self.robot_model.inverse_kinematics(self._target_grasp_pose, link_list=self.link_list, move_target=self.rarm_end_coords, rotation_axis=True)
        assert res is not False
        self.ri.angle_vector(self.robot_model.angle_vector(), time=1.0, time_scale=1.0)
        self.ri.wait_interpolation()
        self.ri.move_gripper("rarm", pos=0.0, effort=1000)

        if debug:
            viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
            viewer.add(self.robot_model)
            viewer.add(Axis.from_coords(self._target_grasp_pose))
            viewer.show()

    def place_egg_on_pan(self):
        assert self._pan_surface_center is not None
        repro = Reproducer(self.robot_model, "../aux_script/trajectory.json", use_torso=False)
        try_count = 0
        while True:
            try:
                angles_list, joint_names = repro(self._pan_surface_center)
                break
            except:
                try_count += 1
                time.sleep(0.5)
                print(try_count)
                if try_count == 30:
                    raise Exception
                print("failure. try again")


        joints = [self.robot_model.__dict__[jn] for jn in joint_names]
        angles_now = [j.joint_angle() for j in joints]
        angles_list.insert(0, angles_now)

        for angles in angles_list:
            for jn, a in zip(joint_names, angles):
                self.robot_model.__dict__[jn].joint_angle(a)
            dur = 1.0
            self.ri.angle_vector(self.robot_model.angle_vector(),
                    time=dur, time_scale=1.0)
            self.ri.wait_interpolation()

        angles_list.reverse()
        for angles in angles_list:
            for jn, a in zip(joint_names, angles):
                self.robot_model.__dict__[jn].joint_angle(a)
            dur = 1.0
            self.ri.angle_vector(self.robot_model.angle_vector(),
                    time=dur, time_scale=1.0)
            self.ri.wait_interpolation()

        # open!
        self.ri.move_gripper("rarm", pos=0.07)

    def turnon_oven(self, debug=False):
        self.reset_robot(0.0) # to be able to see the switch well

        assert self._switch_coords is not None

        target_coords = copy.deepcopy(self._switch_coords)

        def solve_ik_and_send(dur=2.0):
            res = self.robot_model.inverse_kinematics(
                    target_coords,
                    link_list=self.larm_link_list, 
                    move_target=self.larm_end_coords, 
                    rotation_axis=True)
            self.ri.angle_vector(self.robot_model.angle_vector(), time=dur, time_scale=1.0)
            self.ri.wait_interpolation()
            assert res is not None


        target_coords.translate([-0.05, 0.02, -0.02]) # -0.01 for carib error
        if debug:
            ax = Axis(axis_radius=0.01, axis_length=0.1, 
                    pos=target_coords.translation, rot=target_coords.rotation)
            self.viewer.add(ax)

        # pre-push motion
        angles = np.array([15.1908, 43.69, 80.62, -102.1, -146.3, -67.54, 460.74]) * math.pi/180.0
        self.set_larm(angles)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.0, time_scale=1.0)
        self.ri.wait_interpolation()
        solve_ik_and_send()

        # push motion
        target_coords.translate([0.045, 0, 0.0]) 
        solve_ik_and_send(1.0)

        # pull motion
        target_coords.translate([-0.09, 0, 0])
        solve_ik_and_send(1.0)

        # grasp motion
        self.ri.move_gripper("larm", pos=0.07)
        target_coords.translate([0.08, 0, 0])
        solve_ik_and_send()
        self.ri.move_gripper("larm", pos=0.01)

        # rotate motion
        rot_joint = self.robot_model.l_wrist_roll_joint
        rot_joint.joint_angle(rot_joint.joint_angle() + math.pi * 0.5)
        self.ri.angle_vector(self.robot_model.angle_vector(), time=2.0, time_scale=1.0)
        self.ri.wait_interpolation()

        # come-back motion
        self.ri.move_gripper("larm", pos=0.05)
        target_coords.translate([-0.06, 0, 0])
        solve_ik_and_send(1.0)
        self.ri.move_gripper("larm", pos=0.0)

        # SUPER AD-HOC
        self._target_coords = target_coords
        self._lambda = solve_ik_and_send

    def turnoff_oven(self):
        target_coords = self._target_coords
        target_coords.translate([0.08, 0, 0.0]) 
        self._lambda(1.0)

        target_coords.translate([-0.08, 0, 0.0]) 
        self._lambda(1.0)

try:
    demo
except:
    rospy.init_node("demo")
    demo = Demo()

#demo.reset_robot()
#demo.grasp()
#demo.place_egg_on_pan()
#demo.ri.go_pos_unsafe(-0.4, 0, 0)
#demo.reset_robot(0.0)
#demo.turnon_oven()
