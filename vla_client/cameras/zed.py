########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

from typing import List, Tuple

import pyzed.sl as sl
import numpy as np
import cv2
from .base_camera import BaseCamera


OUTPUT_SIZE = 256


class ZEDCamera(BaseCamera):
    RESOLUTION = (720, 1280) # H, W
    K_SIM = np.array([
        [730.0, 0., 640.],
        [0., 730.0, 360.],
        [0., 0., 1.],
    ], dtype=np.float32)
    FPS = 15
    def __init__(self, serial_number=None):
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.sdk_verbose = 1
        init_params.camera_fps = self.FPS
        init_params.optional_settings_path = "res/zed_settings/"

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {err}")

        self.k_real_left = self._get_k('left')
        self.k_real_right = self._get_k('right')
        print(f"k_left:\n{self.k_real_left}")
        print(f"k_right:\n{self.k_real_right}")
        stereo_translation = self.zed.get_camera_information().camera_configuration.calibration_parameters.stereo_transform.get_translation().get()
        print(f"stereo_translation:\n{stereo_translation}")

        self.buffer_left = sl.Mat()
        self.buffer_right = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()

        super().__init__(self.FPS)

    def _get_k(self, side):
        calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters
        if side == 'left':
            cam = calibration_params.left_cam
        elif side == 'right':
            cam = calibration_params.right_cam
        else:
            raise ValueError(f"Invalid side: {side}. Must be 'left' or 'right'")
        return np.array([
            [cam.fx, 0, cam.cx],
            [0, cam.fy, cam.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        

    def _get_frames(self):
        assert self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS
        self.zed.retrieve_image(self.buffer_left, sl.VIEW.LEFT)
        image_left = self.buffer_left.get_data()[:, :, :3][:, :, ::-1]
        self.zed.retrieve_image(self.buffer_right, sl.VIEW.RIGHT)
        image_right = self.buffer_right.get_data()[:, :, :3][:, :, ::-1]
        # check image size
        assert image_left.shape[:2] == self.RESOLUTION
        assert image_right.shape[:2] == self.RESOLUTION

        image_left = self._adjust_image(image_left, self.k_real_left, self.K_SIM)
        image_right = self._adjust_image(image_right, self.k_real_right, self.K_SIM)

        return [image_left, image_right]

    @staticmethod
    def list_cameras(self):
        raise NotImplementedError()

    def get_num_frames(self):
        return 2

    def _adjust_image(self, image: np.ndarray, k_real, k_sim) -> np.ndarray:
        # convert image from real camera to sim uncropped camera, still original size
        transform_mat = k_sim @ np.linalg.inv(k_real)
        image = cv2.warpAffine(image, transform_mat[:2], (image.shape[1], image.shape[0]), flags=cv2.INTER_CUBIC | cv2.WARP_INVERSE_MAP, borderMode=cv2.BORDER_REPLICATE)

        # center crop and resize
        size_y, size_x, _ = image.shape
        start_x = size_x//2 - size_y//2
        start_y = size_y//2 - size_y//2
        new_image = image[start_y : start_y + size_y, start_x : start_x + size_y, :]
        res = cv2.resize(new_image, dsize=(OUTPUT_SIZE, OUTPUT_SIZE), interpolation=cv2.INTER_CUBIC)

        return np.array(res)


def main():
    zed_camera = ZEDCamera()
    while True:
        left_img, right_img = zed_camera.get_frames()
        combined_img = np.hstack((left_img, right_img))
        cv2.imshow("ZED Camera - Left and Right", combined_img[:, :, ::-1])
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break
        elif cv2.waitKey(50) & 0xFF == ord('s'):
            print("Saving images...")
            import time
            timestamp = int(time.time() * 1000)
            import os
            os.makedirs(f"data/{OUTPUT_SIZE}_{timestamp}", exist_ok=True)
            cv2.imwrite(f"data/{OUTPUT_SIZE}_{timestamp}/left.png", cv2.cvtColor(left_img, cv2.COLOR_RGB2BGR))
            cv2.imwrite(f"data/{OUTPUT_SIZE}_{timestamp}/right.png", cv2.cvtColor(right_img, cv2.COLOR_RGB2BGR))
            print(f"Images saved to data/{OUTPUT_SIZE}_{timestamp}/")
    zed_camera.zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
