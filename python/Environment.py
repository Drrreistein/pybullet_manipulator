from copy import deepcopy

import pybullet
from utility import HideOutput, MultiBody

class MultiBody(object):
    def __init__(self) -> None:
        self.ind = None
        self.pose = None
        self.aabb = None

    def InitFromShape(self):
        pass

    def InitFromURDF(self, urdf_file):
        pass

    def get_base_transform(self):
        return self.pose

    def set_base_transform(self, pose):
        
        self.pose = pose
        pass

class Env(object):
    def __init__(self):
        NotImplemented

class Environment(Env):
    def __init__(self):
        method = pybullet.GUI
        with HideOutput():
            # options="--width=1024 --height=768"
            #  --window_backend=2 --render_device=0'
            sim_id = pybullet.connect(method)
            # sim_id = pybullet.connect(pybullet.GUI, options="--opengl2") if use_gui else pybullet.connect(pybullet.DIRECT)
        assert 0 <= sim_id

        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False, physicsClientId=sim_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, False, physicsClientId=sim_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, False, physicsClientId=sim_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, False, physicsClientId=sim_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False, physicsClientId=sim_id)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SHADOWS, True, physicsClientId=sim_id)

        self.sim_id = sim_id
        self.all_bodies = []

    def add_body_from_shape(self):
        pass

    def add_body_from_urdf(self, urdf_file):
        pass

    def remove_body(self):
        pass

    def remove_bodies(self):
        pass

    def get_all_robots(self):
        pass

    def get_all_multibodies(self):
        pass

    def copy(self):
        """
        return a copy of enviroment
        """
        return deepcopy(self)