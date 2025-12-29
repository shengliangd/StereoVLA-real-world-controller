import cv2
import numpy as np
import cv2
import pyrealsense2 as rs
from .base_camera import BaseCamera


class _RealSenseCamera:
    def __init__(
        self,
        serial_number: str = None,
        use_infrared=False,
        width=640,
        height=480,
        fps=15,
    ):
        self.width = width
        self.height = height
        self.use_infrared = use_infrared
        # Configure depth and color streams
        self.config = rs.config()
        if serial_number is not None:
            self.config.enable_device(serial_number)
        self.config.enable_stream(
            rs.stream.depth, self.width, self.height, rs.format.z16, fps
        )
        self.config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.rgb8, fps
        )
        if self.use_infrared:
            self.config.enable_stream(
                rs.stream.infrared, 1, self.width, self.height, rs.format.y8, fps
            )
            self.config.enable_stream(
                rs.stream.infrared, 2, self.width, self.height, rs.format.y8, fps
            )

        # Start RealSense pipeline
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)
        self.profile = self.pipeline.start(self.config)

        intrinsics_matrix = self.get_camera_intrinsics_matrix()
        self.fx = intrinsics_matrix[0, 0]
        self.fy = intrinsics_matrix[1, 1]
        self.cx = intrinsics_matrix[0, 2]
        self.cy = intrinsics_matrix[1, 2]

        # try
        for _ in range(15):
            self.get_frames()

    def get_aligned_frames(self):
        # Wait for a coherent pair of frames: aligned depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = np.asanyarray(aligned_frames.get_color_frame().get_data())
        depth_frame = (
            np.float32(np.asanyarray(aligned_frames.get_depth_frame().get_data()))
            / 1000.0
        )
        if self.use_infrared:
            ir_l_image = np.asanyarray(aligned_frames.get_infrared_frame(1).get_data())
            ir_r_image = np.asanyarray(aligned_frames.get_infrared_frame(2).get_data())
        else:
            ir_l_image = None
            ir_r_image = None
        return color_frame, depth_frame, ir_l_image, ir_r_image

    def get_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        color_frame = np.asanyarray(frames.get_color_frame().get_data())
        depth_frame = (
            np.float32(np.asanyarray(frames.get_depth_frame().get_data())) / 1000.0
        )
        return color_frame, depth_frame

    def get_camera_intrinsics(self):
        # depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intrin = (
            self.profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        return color_intrin

    def get_camera_intrinsics_matrix(self):
        color_intrin = self.get_camera_intrinsics()
        # Construct the intrinsics matrix
        intrinsics_matrix = np.array(
            [
                [color_intrin.fx, 0, color_intrin.ppx],
                [0, color_intrin.fy, color_intrin.ppy],
                [0, 0, 1],
            ]
        )
        return intrinsics_matrix

    def get_rgb_intrinsics_matrix(self):
        color_intrin = self.get_camera_intrinsics()
        # Construct the intrinsics matrix
        intrinsics_matrix = np.array(
            [
                [color_intrin.fx, 0, color_intrin.ppx],
                [0, color_intrin.fy, color_intrin.ppy],
                [0, 0, 1],
            ]
        )
        return intrinsics_matrix

    def get_ir_intrinsics_matrix(self, ir_id):
        ir_intrin = (
            self.profile.get_stream(rs.stream.infrared, ir_id)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        # Construct the intrinsics matrix
        intrinsics_matrix = np.array(
            [
                [ir_intrin.fx, 0, ir_intrin.ppx],
                [0, ir_intrin.fy, ir_intrin.ppy],
                [0, 0, 1],
            ]
        )
        return intrinsics_matrix

    def get_projection_matrix(self):
        import torch

        intrinsic_rgb = self.get_rgb_intrinsics_matrix()
        intrinsic_ir_1 = self.get_ir_intrinsics_matrix(1)
        intrinsic_ir_2 = self.get_ir_intrinsics_matrix(2)
        pose_ir1 = np.array(
            [
                [1.0, 0.0, 0.0, -0.0151],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0, 0, 0, 1],
            ]
        )
        pose_ir2 = np.array(
            [
                [1.0, 0.0, 0.0, -0.0701],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0, 0, 0, 1],
            ]
        )
        pose_rgb = np.eye(4, dtype=np.float32)

        poses = [pose_rgb, pose_ir1, pose_ir2]
        calib_rgb = intrinsic_rgb.copy()
        calib_rgb[:2] *= 0.5
        calib_rgb[0:2, 2] -= 0.5
        calib_ir_1 = intrinsic_ir_1.copy()
        calib_ir_1[:2] *= 0.5
        calib_ir_1[0:2, 2] -= 0.5
        calib_ir_2 = intrinsic_ir_2.copy()
        calib_ir_2[:2] *= 0.5
        calib_ir_2[0:2, 2] -= 0.5
        intrinsics = [calib_rgb, calib_ir_1, calib_ir_2]

        poses = np.stack(poses, 0).astype(np.float32)
        intrinsics = np.stack(intrinsics, 0).astype(np.float32)

        poses = torch.from_numpy(poses)
        intrinsics = torch.from_numpy(intrinsics)

        proj = poses.clone()
        proj[:, :3, :4] = torch.matmul(intrinsics, poses[:, :3, :4])
        return proj

    def show_frames(self):
        color_frame, depth_frame = self.get_frames()
        cv2.imshow("color_frame", cv2.cvtColor(color_frame, cv2.COLOR_RGB2BGR))
        cv2.imshow("depth_frame", depth_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            exit(0)

    def _show_frames(self):
        import matplotlib.pyplot as plt

        depth_frame, color_frame = self.get_frames()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Plot the depth and color images
        plt.subplot(1, 2, 1)
        plt.imshow(depth_image, cmap="gray")
        plt.title("Depth Image")
        plt.subplot(1, 2, 2)
        plt.imshow(color_image)
        plt.title("Color Image")
        plt.show()

    def __del__(self):
        cv2.destroyAllWindows()
        print("camera stoped")
        self.pipeline.stop()


class RealSenseCamera(BaseCamera):
    def __init__(self, serial_number):
        self.camera = _RealSenseCamera(serial_number, fps=30)
        self.k_real = self.camera.get_camera_intrinsics_matrix()
        self.k_sim = np.array([
            [605, 0, 320],
            [0, 605, 240],
            [0, 0, 1],
        ], dtype=np.float32)
        print(f'camera intrinsics {self.k_real}')
        super().__init__(30)

    def _get_frames(self):
        rgb, depth, _, _ = self.camera.get_aligned_frames()
        rgb = self._align_camera(rgb, self.k_real, self.k_sim)
        return [rgb]

    @staticmethod
    def list_cameras():
        context = rs.context()
        devices = context.query_devices()
        return devices

    def get_num_frames(self):
        return 1

    def _align_camera(self, image, k_real, k_sim):
        assert image.shape[0] == 480 and image.shape[1] == 640

        # convert image from real camera to sim uncropped camera, still original size
        transform_mat = k_sim @ np.linalg.inv(k_real)
        image = cv2.warpAffine(image, transform_mat[:2], (640, 480), flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_REPLICATE)

        # center crop and resize
        size_y, size_x, _ = image.shape
        start_x = size_x//2 - size_y//2
        start_y = size_y//2 - size_y//2
        new_image = image[start_y : start_y + size_y, start_x : start_x + size_y, :]
        res = cv2.resize(new_image, dsize=(256, 256), interpolation=cv2.INTER_CUBIC)

        return res
