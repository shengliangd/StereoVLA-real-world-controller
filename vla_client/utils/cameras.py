import cv2
import numpy as np


import rospy
import threading
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

from ..cameras import MultiCamera


class CameraVisualizer:
    def __init__(self, multi_camera: MultiCamera, hz=10, size=(512, 512)):
        self.multi_camera = multi_camera
        self.cv_bridge = CvBridge()

        self.hz = hz
        self.size = size
        self.bboxes = {}
        self.captions = {}

        self.video_writer = None
        self.video_writer_lock = threading.Lock()

        self.publishers = []
        for idx in range(self.multi_camera.get_num_frames()):
            self.publishers.append(rospy.Publisher(f'/cameras/view_{idx}', ImageMsg, queue_size=0))

        def fn():
            rate = rospy.Rate(self.hz)
            while not rospy.is_shutdown():
                self._step()
                rate.sleep()
        self.step_thread = threading.Thread(target=fn, daemon=True)
        self.step_thread.start()

    def _step(self):
        frames = []
        for idx, color_frame in enumerate(cv2.cvtColor(cv2.resize(f.copy(), self.size), cv2.COLOR_RGB2BGR) for f in self.multi_camera.get_frames()):
            if idx in self.bboxes:
                color_frame = self.draw_bbox(color_frame, *self.bboxes[idx])
            self.draw_cross_(color_frame)
            if idx in self.captions is not None:
                cv2.putText(color_frame, self.captions[idx], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 1, cv2.LINE_AA)
            frames.append(color_frame)
            try:
                self.publishers[idx].publish(self.cv_bridge.cv2_to_imgmsg(color_frame, "bgr8"))
            except:
                raise
                pass
        with self.video_writer_lock:
            if self.video_writer is not None:
                self.video_writer.write(np.concatenate(frames, axis=1))

    def draw_bbox(self, frame, bbox, image_size):
        original_size = frame.shape[:2]
        # directly resize the bbox
        bbox = np.array(bbox)
        image_size = np.array(image_size)
        bbox = (bbox * np.array([*(original_size/image_size)]*2)).astype(int)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
        return frame

    def draw_cross_(self, frame):
        cross_length = 50
        cv2.line(frame, (frame.shape[0]//2-cross_length, frame.shape[1]//2), (frame.shape[0]//2+cross_length, frame.shape[1]//2), color=(0, 255, 0), thickness=1)
        cv2.line(frame, (frame.shape[0]//2, frame.shape[1]//2-cross_length), (frame.shape[0]//2, frame.shape[1]//2+cross_length), color=(0, 255, 0), thickness=1)

    def start_recording(self, path):
        with self.video_writer_lock:
            assert self.video_writer is None
            self.video_writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), self.hz, (self.size[0] * self.multi_camera.get_num_frames(), self.size[1]))

    def stop_recording(self):
        with self.video_writer_lock:
            assert self.video_writer is not None
            self.video_writer.release()
            self.video_writer = None

    def set_caption(self, camera_name, caption):
        self.captions[camera_name] = caption

    def set_bbox(self, camera_name, bbox):
        self.bboxes[camera_name] = bbox

    def clear(self):
        self.bboxes.clear()
        self.captions.clear()
