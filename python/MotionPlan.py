import pybullet as pb
from motion_planners.smoothing import smooth_path
from motion_planners.rrt import TreeNode, configs
from motion_planners.utils import irange, argmin, RRT_ITERATIONS, RRT_RESTARTS, RRT_SMOOTHING
from CollisionCheck import CollisionChecker
import numpy as np
import utils

class Planner(object):
    pass

class OMPLPlanner(Planner):
    pass

class SimplePlanner(Planner):
    def __init__(self, robot):
        self.robot = robot
        self.configured = False
        self.collision_checker = CollisionChecker()

    def config(self, planning_timeout=5, planner_type='rrt_connect', iterations=100, planning_joints=None, 
                distance_weight=None, joint_limits=None, extend_resolution=None):
        self.planning_timeout = planning_timeout
        self.planner_type = planner_type
        self.max_sampling_iterations = iterations 
        self.configured = True

        self.planning_joints = utils.get_movable_joints(self.robot) if planning_joints is None else planning_joints
        self.joints_length = len(self.planning_joints)
        self.distance_weight = np.ones(self.joint_length) if distance_weight is None else np.array(distance_weight)
        self.joint_limits = np.array(utils.get_joint_limits(self.robot)) if joint_limits is None else np.array(joint_limits)
        self.extend_resolution = 0.1 if extend_resolution is None else extend_resolution

    def solve(self, start, end):
        path = []

        if not self.configured:
            return False, None, None
        res = True
        if self.planner_type=='rrt_connect':
            path, collision_pairs = self.__rrt_connect(start, end)
            if path is None: 
                res = False
        return res, path, collision_pairs

    def __rrt(self, q1, q2):
        pass

    def __rrt_connect(self, q1, q2):
        collision_pairs = set()

        # check initial pos
        if self.collision_checker.check_collision(q1)[0] or self.collision_checker.check_collision(q2)[0]:
            print(f"initial position collision")
            return None, set()

        root1, root2 = TreeNode(q1), TreeNode(q2)
        nodes1, nodes2 = [root1], [root2]
        for _ in irange(self.max_sampling_iterations):
            if len(nodes1) > len(nodes2):
                nodes1, nodes2 = nodes2, nodes1
            s = self.sample()

            last1 = argmin(lambda n: self.distance(n.config, s), nodes1)
            for q in self.extend(last1.config, s):
                res, col_pairs = self.collision_checker.check_collision(q)
                if res:
                    collision_pairs.update(col_pairs)
                    break
                last1 = TreeNode(q, parent=last1)
                nodes1.append(last1)

            last2 = argmin(lambda n: self.distance(n.config, last1.config), nodes2)
            for q in self.extend(last2.config, last1.config):
                res, col_pairs = self.collision_checker.check_collision(q)
                if res:
                    collision_pairs.update(col_pairs)
                    break
                last2 = TreeNode(q, parent=last2)
                nodes2.append(last2)
            else:
                path1, path2 = last1.retrace(), last2.retrace()
                if path1[0] != root1:
                    path1, path2 = path2, path1
                return configs(path1[:-1] + path2[::-1]), collision_pairs
        return None, collision_pairs

    def __rrt_star(self, q1, q2):
        pass

    def extend(self, q1, q2):
        steps = np.abs(np.divide(q1 - q2, self.extend_resolution))
        n = int(np.max(steps)) + 1
        q = q1
        for i in range(n+1):
            q = tuple(i*np.array(q2 - q1)/n + q1)
            yield q

    def sample(self):
        return np.random.uniform(self.joint_limits)

    def distance(self, q1, q2):
        return np.sqrt((self.distance_weight * np.array(q1-q2)**2).sum())