#!/usr/bin/env python  
import json
import numpy as np
import skrobot
import tf
import rospy

def qv_mult(q1, v1_):
    length = np.linalg.norm(v1_)
    v1 = v1_/length
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    v_converted = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1))[:3]
    return v_converted * length

def convert(tf_12, tf_23):
    tran_12, rot_12 = [np.array(e) for e in tf_12]
    tran_23, rot_23 = [np.array(e) for e in tf_23]
    rot_13 = tf.transformations.quaternion_multiply(rot_12, rot_23)
    tran_13 = tran_23 + qv_mult(rot_23, tran_12)
    return list(tran_13), list(rot_13)


class DataManager(object):
    def __init__(self, arm, object_frame):
        robot_model = skrobot.models.PR2()
        ri = skrobot.interfaces.ros.PR2ROSRobotInterface(robot_model)
        self.robot_model = robot_model
        self.ri = ri
        self.listener = tf.TransformListener()
        self.object_frame = object_frame

        tmp = ["_shoulder_pan_joint", "_shoulder_lift_joint", "_upper_arm_roll_joint", "_elbow_flex_joint","_forearm_roll_joint", "_wrist_flex_joint", "_wrist_roll_joint"]

        if arm == "rarm":
            self.arm_joint_names = [("r" + e) for e in tmp]
            self.tool_frame = 'r_gripper_tool_frame'
        else:
            self.arm_joint_names = [("l" + e) for e in tmp]
            self.tool_frame = 'l_gripper_tool_frame'
        self.arm_joint_names.append("torso_lift_joint")

        # initialize data to be dumped
        self.data = {"joint_names" : self.arm_joint_names, "end_effector": self.tool_frame}
        self.data["angles_seq"] = []
        self.data["pose_seq"] = []

        self.T_base_to_target = None

    def _reflect_current_state(self):
        self.robot_model.angle_vector(self.ri.angle_vector())

    def get_arm_angles(self):
        self._reflect_current_state()
        return np.array([self.robot_model.__dict__[jn].joint_angle() for jn in self.arm_joint_names]).tolist()

    def get_target_pose(self):
        # get relative pose of the arm from the focusing objective
        while True:
            try:
                T_base_to_target = self.listener.lookupTransform(self.object_frame, "base_link", rospy.Time(0))
                return T_base_to_target
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def get_relative_pose(self):
        while True:
            try:
                T_gripper_to_base = self.listener.lookupTransform("base_link", self.tool_frame, rospy.Time(0))
                return convert(T_gripper_to_base, self.T_base_to_target)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def push_waypoint(self):
        rel_pose = self.get_relative_pose()
        angles = self.get_arm_angles()
        self.data["angles_seq"].append(angles)
        self.data["pose_seq"].append(rel_pose)

    def record(self):
        self.T_base_to_target = self.get_target_pose()

        print("start recording. Please hit \"q\" if want to terminate.")
        while True:
            string = raw_input()
            if string=='q':
                print("finish recording")
                return
            self.push_waypoint()
            print("successfully recorded!")

    def dump(self, filename="trajectory.json"):
        with open(filename, mode='w') as f:
            json.dump(self.data, f, indent=2)

if __name__ == '__main__':
    #dm = DataManager("rarm", "base_link")
    dm = DataManager("rarm", "/pan_surface_center")
    dm.record()
    dm.dump()
