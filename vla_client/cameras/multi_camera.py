from .base_camera import BaseCamera
from .realsense import RealSenseCamera
from .zed import ZEDCamera

from typing import List


CAMERA_REGISTRY = {
    'zed': ZEDCamera,
    'realsense': RealSenseCamera,
}


class MultiCamera:
    def __init__(self, camera_specs):
        self.cameras: List[BaseCamera] = []
        for camera_spec in camera_specs.split(':'):
            camera_type, *args = camera_spec.split(',')
            self.cameras.append(CAMERA_REGISTRY[camera_type](*args))
        self.num_frames = sum(c.get_num_frames() for c in self.cameras)

    def get_frames(self):
        ret = []
        for camera in self.cameras:
            ret.extend(camera.get_frames())
        return ret
    
    @staticmethod
    def list_cameras():
        raise NotImplementedError()
    
    def get_num_frames(self):
        return self.num_frames
