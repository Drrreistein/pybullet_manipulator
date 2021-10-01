import collections
import pybullet
import numpy as np
from collections import namedtuple
import itertools

collision_pair = namedtuple('col_pair', ['body_link1','body_link2'])
class body_link(object):
    def __init__(self, body, link):
        self.body = body
        self.link = link

    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.body, self.link)

class CollisionMatrix(object):
    def __init__(self):
        pass

class CollisionChecker(object):
    def __init__(self):
        self.distance_threshold = 1e-3

        self.__get_collision_objects()
        self.__get_collision_matrix()

    def __get_collision_objects(self):
        num_bodies = pybullet.getNumBodies()
        self.collision_objects = []
        for b in range(num_bodies):
            num_link = pybullet.getNumJoints(b)
            if num_link>0:
                for l in range(num_link):
                    self.collision_objects.append(body_link(b,l))
            else:
                self.collision_objects.append(body_link(b,-1))

    def get_collision_objects(self):
        return self.collision_objects

    def get_distance_threshold(self):
        return self.distance_threshold

    def set_distance_threshold(self, threshold):
        self.distance_threshold  = threshold

    def __get_collision_matrix(self):
        self.collision_matrix = tuple(itertools.combinations(self.collision_objects, 2))

    def get_collision_matrix(self):
        return self.collision_matrix

    def get_self_collision_matrix(self, body):
        num_link = pybullet.getNumJoints(body)
        if num_link:
            return None
        else:
            body_links = []
            for i in range(num_link):
                body_links.append(body_link(body, i))
            return itertools.combinations(body_links, 2)

    def set_collision_matrix(self, collision_matrix):
        self.collision_matrix = collision_matrix

    def check_collision(self):
        for p in self.collision_matrix:
            if len(pybullet.getClosestPoints(p[0].body, p[1].body, self.distance_threshold, p[0].link, p[1].link)):
                return True, p
        return False, None

    def check_pair_collision(self, body_link1, body_link2):
        if len(pybullet.getClosestPoints(body_link1.body, body_link2.body, self.distance_threshold, body_link1.link, body_link2.link)):
            return True
        else:
            return False
        
    def show_collision_area(self):
        pass
