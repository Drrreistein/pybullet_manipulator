import collections
from typing import Iterator
from logging_utils import ColoredFormatter
import pybullet
import numpy as np
from IPython import embed

class Visualizer(object):
    def __init__(self):
        self.handles = []
        pass

    def __add_segments(self, points, closed=False, color=(1,0,0), width=1):
        lines = []
        for v1, v2 in zip(points, points[1:]):
            lines.append(pybullet.addUserDebugLine(v1, v2, lineColorRGB=color, lineWidth=width))
        if closed:
            lines.append(pybullet.addUserDebugLine(points[-1], points[0], lineColorRGB=color, lineWidth=width))
        return lines

    def plt_frame(self, pose, length_in_mm=100):
        origin_world = pose[0]
        handles = []
        for k in range(3):
            axis = np.zeros(3)
            axis[k] = 1
            axis_world = pybullet.multiplyTransforms(pose[0],pose[1], tuple(axis*length_in_mm/1000), (0,0,0,1))[0]
            handles.append(self.__add_segments([origin_world, axis_world], color=axis))
        return handles

    def plt_frames(self, poses, length_in_mm=100):
        return [self.plt_frame(p, length_in_mm=length_in_mm) for p in poses]

    def plt_pose(self, pose, length_in_mm=100):
        return self.plt_frames(pose, length_in_mm=length_in_mm)

    def plt_poses(self, poses, length_in_mm=100):
        return self.plt_frames(poses, length_in_mm=length_in_mm)

    def plt_line(self, point1, point2, color=(1,0,0), width_in_mm=1):
        return self.__add_segments([point1, point2], color=color, width=width_in_mm)

    def plt_lines(self, points1, points2, color=(1,0,0), width_in_mm=1):
        return [self.plt_line([p1, p2], color=color, width_in_mm=width_in_mm) for p1, p2 in zip(points1, points2)]

    def plt_point(self, point, size_in_mm=10, color=(1,0,0)):
        points = []
        for i in range(len(point)):
            axis = np.zeros(len(point))
            axis[i] = 1.0
            points.append(np.array(point) - size_in_mm / 1000 / 2 * axis)
            points.append(np.array(point) + size_in_mm / 1000 / 2 * axis)
        return self.__add_segments([points], closed=True, color=color, width=size_in_mm)

    def plt_points(self, points, size_in_mm=10, color=(1,0,0)):
        return [self.plt_point(p, color=color, size_in_mm=size_in_mm) for p in points]

    def plt_text(self, points, text):
        pass

    def plt_box(self, pose, size:Iterator):
        pass

    def plt_cylinder(self, pose, size:Iterator):
        pass

    def plt_sphere(self, pose, radius):
        pass

    def del_vis(self, handles:list):
        pass