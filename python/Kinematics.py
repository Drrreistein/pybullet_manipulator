from numpy import random
from numpy.core.defchararray import join
import pybullet as p
from utils.pybullet_tools import utils
from Robot import Robot
from IPython import embed
import numpy as np

class Kinematics():
    def __init__(self, robot_model:Robot):

        self.robot = robot_model.robot
        self.rm = robot_model

        self.num_joints = self.rm.num_joints
        self.end_effector_link = self.rm.end_effector_link
        self.joint_limits = self.rm.joint_limits
        self.movable_joints = self.rm.movable_joints

    def forward_kinematics(self, joints, target_link=None):
        # TODO
        pass
    
    def get_joint_positions(self):
        return self.rm.get_joint_positions()

    def set_joint_positions_by_indices(self, joints, joint_inds):
        for i, ind in enumerate(joint_inds):
            p.resetJointState(self.robot, ind, joints[i], targetVelocity=0)

    def set_joint_positions(self, joints_value):
        assert len(self.movable_joints) == len(joints_value), 'joints length not legal'
        assert self.inside_joint_limits(joints_value), 'target joints value out of joint limits'
        for i in range(len(joints_value)):
            p.resetJointState(self.robot, self.movable_joints[i], joints_value[i], targetVelocity=0)

    def get_tcp_pose(self):
        return self.rm.get_tcp_pose()

    def get_fk_by_setting(self, joints):
        current_joints = self.rm.get_joint_positions()
        self.set_joint_positions(joints)
        tcp_pose = self.rm.get_tcp_pose(joints)
        self.set_joint_positions(current_joints)
        return tcp_pose

    def inverse_kinematics(self, pose, target_link=None):
        # TODO, how to deal with the situation when ik out of joint limits
        if target_link is None:
            target_link = self.end_effector_link
        position, orientation = pose
        kinematic_conf = p.calculateInverseKinematics(self.robot, target_link, position, orientation)
        return kinematic_conf
    
    def inverse_kinematics_random(self, pose, target_link=None, max_tries=20):
        init_joints = self.get_joint_positions()
        for i in range(max_tries):
            random_joints = self.sample_joints()
            self.set_joint_positions(random_joints)
            tmp_conf = self.inverse_kinematics(pose, target_link)
            if self.inside_joint_limits(tmp_conf):
                break
            else:
                tmp_conf = None
        self.set_joint_positions(init_joints)
        return tmp_conf

    def inverse_kinematics_fix_joint(self, fix_joint, pose):
        pass

    def get_joint_limits(self):
        return self.rm.get_joint_limits()

    def set_joint_limits(self, joint_limits):
        self.joint_limits = joint_limits

    def get_jacobian(self):
        pass

    def sample_joints(self):
        return [np.random.uniform(self.joint_limits[i,0],self.joint_limits[i,1],1)[0] for i in range(len(self.movable_joints))]

    def sample_pose(self):
        random_joints = self.sample_joints()
        return self.get_fk_by_setting(random_joints)

    def inside_joint_limits(self, joints, joints_value):
        assert len(joints) == len(joints_value), 'must be in the same length'
        return all(self.joint_limits[np.array(joints),0] <= joints_value) and all(joints_value <= self.joint_limits[np.array(joints),1])

    def out_of_joint_limits(self, joints, joints_value):
        return not self.inside_join_limits(joints, joints_value)

    def equal_pose(self, pose1, pose2):
        return np.linalg.norm(np.array(pose1[0])-np.array(pose2[0])) + np.linalg.norm(np.array(pose1[1])-np.array(pose2[1])) < 1e-4

    def equal_joints(self, joints1, joints2):
        return np.linalg.norm(np.array(joints1)-np.array(joints2))<1e-5