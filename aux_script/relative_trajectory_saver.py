#!/usr/bin/env python  
import json
import numpy as np
import skrobot
import tf
import rospy

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

    def _reflect_current_state(self):
        self.robot_model.angle_vector(self.ri.angle_vector())

    def get_arm_angles(self):
        self._reflect_current_state()
        return np.array([self.robot_model.__dict__[jn].joint_angle() for jn in self.arm_joint_names]).tolist()

    def get_relative_pose(self):
        # get relative pose of the arm from the focusing objective
        while True:
            try:
                tf_relative = self.listener.lookupTransform(self.object_frame, self.tool_frame, rospy.Time(0))
                return tf_relative
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def push_waypoint(self):
        rel_pose = self.get_relative_pose()
        angles = self.get_arm_angles()
        self.data["angles_seq"].append(angles)
        self.data["pose_seq"].append(rel_pose)

    def record(self):
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
    dm = DataManager("rarm", "base_link")
    dm.record()
    dm.dump()
