import pybullet 
from collections import namedtuple
import numpy as np
from IPython import embed

class Robot(object):
    class link():
        def __init__(self, ind, name, pose):
            self.index = ind
            self.name = name
            self.pose = pose

    JointInfo = namedtuple('JointInfo', ['joint_index', 'joint_name', 'joint_lower_limit', 'joint_upper_limit'])
    LinkInfo = namedtuple('LinkInfo', ['link_index','link_name'])
    LinkState = namedtuple('LinkInfo', ['link_name','link_rt_world'])
    
    def __init__(self, robot, arms=None, base=None):
        # TODO: add arms and base for mobile robot mainpulator
        self.robot = robot
        (base_name, robot_name) = pybullet.getBodyInfo(self.robot)
        self.base_name = base_name.decode()
        self.robot_name = robot_name.decode()

        self.num_joints = pybullet.getNumJoints(self.robot)
        self.__get_joint_info()
        
        self.end_effector_link = self.get_ee_link().link_index
        self.end_to_tool = ((0,0,0),(0,0,0,1))

    def get_base_name(self):
        return self.base_name

    def get_robot_name(self):
        return self.robot_name

    def __get_joint_info(self):

        self.link_names = []
        self.link_names.append(pybullet.getBodyInfo(self.robot)[0].decode())
        self.link_info = []
        self.link_info.append(Robot.LinkInfo(-1, link_name=self.link_names[0]))

        self.joint_info = []
        self.joint_limits = []
        self.movable_joints = []
        for i in range(self.num_joints):
            j_info = pybullet.getJointInfo(self.robot, i)
            self.link_names.append(j_info[-5].decode())
            self.link_info.append(Robot.LinkInfo(i, link_name=self.link_names[-1]))

            self.joint_info.append(Robot.JointInfo(j_info[0],joint_name=j_info[1], joint_lower_limit=j_info[8], joint_upper_limit=j_info[9]))
            if j_info[8]<j_info[9]:
                self.movable_joints.append(i)
                self.joint_limits.append([j_info[8], j_info[9]])

        self.joint_limits = np.array(self.joint_limits)

    def get_link_names(self):
        return self.link_names

    def get_link_info(self):
        return self.link_info

    def get_link_states(self):
        ## TODO, what are self.link[-1] and self.link[-2]
        link_states = []
        link_states.append(Robot.LinkState(self.link_names[0], link_rt_world=pybullet.getBasePositionAndOrientation(self.robot)))
        for i in range(self.num_joints):
            ls = pybullet.getLinkState(self.robot, i)
            link_states.append(Robot.LinkState(self.link_names[i+1], link_rt_world=(ls[0], ls[1])))
        return link_states

    def get_joint_info(self):
        return self.joint_info

    def get_ee_link(self):
        ee_link_index = self.num_joints-2
        return self.link_info[ee_link_index]

    def get_base_link(self):
        # TODO self.link[0] or self.link[1]
        return self.link_info[0]

    def get_joint_limits(self):
        return self.joint_limits

    def get_tcp_pose(self):
        ls = pybullet.getLinkState(self.robot, self.end_effector_link)
        return (ls[0], ls[1])

    def get_joint_positions(self):
        return [pybullet.getJointState(self.robot, i)[0] for i in self.movable_joints]

    def get_link_pose(self, link):
        pass

    def set_end_to_tool(self, pose):
        pass

    def get_end_to_tool(self):
        pass