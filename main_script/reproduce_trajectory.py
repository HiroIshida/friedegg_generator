#!/usr/bin/env python  
import time
import copy
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

class Reproducer(object):
    def __init__(self, robot_model, filename):
        self.robot_model = copy.deepcopy(robot_model) # ref is ok?
        with open(filename, mode='r') as f:
            self.data = json.load(f)
        self.move_joints = [self.robot_model.__dict__[jn] for jn in self.data["joint_names"]]
        self.move_links = [j.child_link for j in self.move_joints]

    def __call__(self, T_target_to_base):
        move_target = self.robot_model.__dict__[self.data["end_effector"]]
        angles_list = [] 
        for T_gripper_to_target, angles in zip(self.data["pose_seq"], self.data["angles_seq"]):
            T_gripper_to_base = convert(T_gripper_to_target, T_target_to_base)

            # coverting raw transform representation to skrobot coords
            pos, ori = T_gripper_to_base
            position = np.array(pos)
            orientation = np.array([ori[3], ori[0], ori[1], ori[2]])
            rotmat = skrobot.coordinates.math.rotation_matrix_from_quat(orientation)
            coords = skrobot.coordinates.Coordinates(position, rotmat)
            for j, a in zip(self.move_joints, angles):
                j.joint_angle(a)

            is_success = self.robot_model.inverse_kinematics(
                    coords,
                    link_list=self.move_links,
                    move_target=move_target,
                    rotation_axis=True)
            assert (is_success is not False)
            angles_list.append([j.joint_angle() for j in self.move_joints])
        return angles_list, self.data["joint_names"]

if __name__ == '__main__':
    try:
        robot_model
    except:
        robot_model = skrobot.models.PR2()
        viewer = skrobot.viewers.TrimeshSceneViewer(resolution=(640, 480))
        viewer.add(robot_model)
        viewer.show()
    repro = Reproducer(robot_model, "../aux_script/trajectory.json")

    T_target_to_base = [[0.0, 0, 0.0], [0, 0, 0, 1.]]
    angles_list, joint_names = repro(T_target_to_base)
    joints = [robot_model.__dict__[jn] for jn in joint_names]

    for angles in angles_list:
        time.sleep(1.0)
        for j, a in zip(joints, angles):
            j.joint_angle(a)
        viewer.redraw()
