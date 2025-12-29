from ..controllers import common
from ..utils.timer import Timer
from ..utils import input_choice, input_typed, TrajectoryRecorder
from ..utils.cameras import CameraVisualizer
from ..cameras import MultiCamera

import pynput
from pynput import keyboard
import transforms3d as t3d
import controllers
import random
import zmq
import datetime
import numpy as np
import io
import PIL.Image
import termios
import sys
import time
import threading


class GraspMode:
    def __init__(self, args, only_grasp=True):
        self.args = args
        self.only_grasp = only_grasp

        self.multi_camera = MultiCamera(args.cameras)

        self.camera_visualizer = CameraVisualizer(self.multi_camera)

        self.blocking = self.args.controller == "blocking"
        self.robot_controller = controllers.FrankaROSController("logical" if self.blocking else "physical", self.args.extended_finger)

        self.robot_lock = threading.RLock()
        self.env_id = random.randint(0, 1000000000)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(f"tcp://{args.server_ip}:{args.server_port}")

    def run(self):
        while True:
            self._goto_init()
            self.robot_controller.reset_history()

            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            if self.only_grasp:
                user_input = input_typed('Enter target object caption (empty string for exit): ', str)
            else:
                user_input = input_typed('Enter instruction (empty string for exit): ', str)
            if user_input == '':
                break
            instruction = f'pick up {user_input}' if self.only_grasp else user_input
            self._run_once(instruction)

    def _run_once(self, instruction):
        self.finished = False
        self.paused = False
        timer = Timer()
        folder = f'{self.args.save_to}-{datetime.datetime.now().strftime("%Y%m%d%H%M%S")}'
        recorder = TrajectoryRecorder(folder=folder)
        self.camera_visualizer.set_caption(0, instruction)
        self.camera_visualizer.start_recording(f'{folder}/video.mp4')
        with pynput.keyboard.Listener(on_press=self._on_keyboard_press, on_release=self._on_keyboard_release):
            while not self.finished:
                time.sleep(0.005)
                if self.paused or self.robot_controller.abnormal:
                    time.sleep(0.1)
                    continue

                with self.robot_lock:
                    if self.paused:
                        continue
                    step_info = {}
                    # if eef closed to 0, force open
                    if self.robot_controller.get_eef_width() < 0.0013:
                        eef_pose = self.robot_controller.get_eef_pose()
                        current_position = eef_pose[:3]
                        current_orientation = eef_pose[3:6]
                        current_orientation_mat = t3d.euler.euler2mat(*current_orientation)
                        try:
                            self.robot_controller.execute_waypoints([([*current_position[:2], max(0.30, current_position[2])], current_orientation_mat, 1)])
                        except RuntimeError as e:
                            continue
                        try:
                            self.robot_controller.wait_for_reach(common.POS_TOL, common.ROT_TOL, 3)
                        except RuntimeError as e:
                            continue
                        except TimeoutError as e:
                            continue
                        self.robot_controller.reset_history()
                        continue

                    # hold the lock to avoid other threads to control the robot (especially the keyboard thread)
                    images = self.multi_camera.get_frames()
                    assert len(images) == 2, 'currently only support two-camera setting'

                    depth = np.array([[0.]], dtype=np.float32)  # for compatibility
                    bytes_io = io.BytesIO()
                    PIL.Image.fromarray(depth[:, :, None].view(np.uint8)).save(bytes_io, format="PNG", compress_level=9)
                    compressed_depth = bytes_io.getvalue()

                    prev_eef_pose = self.robot_controller.get_eef_pose(0.3)
                    eef_pose = self.robot_controller.get_eef_pose()

                    message = {
                        "text": instruction,
                        # this order follows the data generation
                        "image_wrist_array": [self._compress_image(images[0])],  # left camera
                        "image_array": [self._compress_image(images[1])],  # right camera
                        "depth_array": [compressed_depth],
                        "proprio_array": [prev_eef_pose, prev_eef_pose, prev_eef_pose, eef_pose],
                        "env_id": self.env_id,
                        "traj_metadata": None,
                        "compressed": True,
                    }

                    with timer("request"):
                        response = self.socket.send_pyobj(message)
                        response = self.socket.recv_pyobj()
                    print(f"policy: request time {timer.get_time('request'):.2f}s")

                    step_info.update({
                        "request": message,
                        "response": response,
                        "request_delay": timer.get_time("request"),
                        "qpos": self.robot_controller.get_qpos(),
                    })

                    assert response["info"] == "success", "invalid response from server"
                    debug = response["debug"]
                    if "pose" in debug:
                        self.robot_controller.update_tf('grasp_pose', debug["pose"][0], debug["pose"][1])
                    if "proprio" in debug:
                        self.robot_controller.update_tf('proprio', debug["proprio"][0], debug["proprio"][1])
                        self.robot_controller.update_tf('proprio_gt', debug["proprio_gt"][0], debug["proprio_gt"][1])
                    if debug.get("bbox") is not None:
                        # the order matches simulation
                        self.camera_visualizer.set_bbox(0, (debug["bbox"][0], (224, 224)))
                        self.camera_visualizer.set_bbox(1, (debug["bbox"][1], (224, 224)))

                    delta_actions = response["result"]
                    current_position = eef_pose[:3]
                    current_orientation_mat = t3d.euler.euler2mat(*eef_pose[3:6])
                    abs_actions = []
                    cnt = 0
                    for action in delta_actions:
                        assert action[6] in [-1., 1., 0.], "please quantize gripper actions before passing it to me"
                        target_position = current_position + action[:3]
                        target_orientation_mat = t3d.euler.euler2mat(*action[3:6]) @ current_orientation_mat
                        abs_actions.append((target_position, target_orientation_mat, action[6]))
                        self.robot_controller.update_tf('target_' + str(cnt), abs_actions[cnt][0], t3d.euler.mat2euler(abs_actions[cnt][1]))
                        cnt = cnt + 1
                        current_position = target_position
                        current_orientation_mat = target_orientation_mat

                    step_info["actions"] = abs_actions
                    try:
                        self.robot_controller.execute_waypoints(abs_actions)
                    except RuntimeError:
                        continue

                    recorder.submit_step(step_info)
        self.camera_visualizer.stop_recording()
        self.camera_visualizer.clear()
        recorder.finish()

        # put down
        eef_pose = self.robot_controller.get_eef_pose()
        current_position = eef_pose[:3]
        current_orientation_mat = t3d.euler.euler2mat(*eef_pose[3:6])
        # release the object and lift
        self.robot_controller.execute_waypoints([([*current_position[:2], current_position[2]], current_orientation_mat, 0)])

    def _on_keyboard_press(self, key):
        if hasattr(key, "char"):
            # open gripper
            if key.char == '=':
                print('keyboard: force open gripper...')
                with self.robot_lock:
                    self.paused = True
                    self.robot_controller.move_gripper('open')
                    print('keyboard: gripper opened, and paused')
            # close gripper
            elif key.char == '-':
                print('keyboard: force close gripper...')
                with self.robot_lock:
                    self.paused = True
                    self.robot_controller.move_gripper('close')
                    print('keyboard: gripper closed, and paused')
            # pause/resume
            elif key.char == 'p':
                print('keyboard: ', 'pause...' if not self.paused else 'resume...')
                with self.robot_lock:
                    self.paused = not self.paused
                    print('keyboard : ', 'paused' if self.paused else 'resumed')
            # quit
            elif key.char == 'q':
                print('keyboard: finish trajectory')
                self.finished = True
            elif key.char == 'o':
                print('keyboard: place object...')
                with self.robot_lock:
                    self.paused = True
                    self._place_object()
                print('keyboard: finished placing, you can resume (p) or finish (q)')
            else:
                pass
        else:
            if key == keyboard.Key.space:
                with self.robot_lock:
                    self.paused = not self.paused
                print('keyboard: ', 'pause' if self.paused else 'resume')
            elif key == keyboard.Key.esc:
                exit(1)

    def _on_keyboard_release(self, key):
        import sys
        if hasattr(key, "char") or key in [keyboard.Key.space]:
            sys.stdout.write('\b \b')
            sys.stdout.flush()

    def _place_object(self):
        eef_pose = self.robot_controller.get_eef_pose()

        # lift to 0.5
        waypoint = ((eef_pose[0], eef_pose[1], 0.5), t3d.euler.euler2mat(-np.pi, 0., np.pi/4), -1)
        self.robot_controller.execute_waypoints([waypoint])

        # go to left point
        waypoint = ((0.20, 0.40, 0.5), t3d.euler.euler2mat(-np.pi, 0., np.pi/2), -1)
        self.robot_controller.execute_waypoints([waypoint])

        # # go down, release object
        waypoint = ((0.20, 0.40, 0.20), t3d.euler.euler2mat(-np.pi, 0., np.pi/2), 1)
        self.robot_controller.execute_waypoints([waypoint])

        # # go up
        waypoint = ((0.20, 0.40, 0.5), t3d.euler.euler2mat(-np.pi, 0., np.pi/2), 1)
        self.robot_controller.execute_waypoints([waypoint])

        # reset
        self._goto_init()

    def _goto_init(self):
        self.robot_controller.move_gripper('open')
        init_pos = [0.36307235, -0.00142398, 0.48912046]
        init_ori_mat = t3d.euler.euler2mat(-np.pi, 0., np.pi/4)
        self.robot_controller.execute_waypoints([(init_pos, init_ori_mat, 1)])
        self.robot_controller.wait_for_reach(common.POS_TOL, common.ROT_TOL, 5)

    def _compress_image(self, image):
        bytes_io = io.BytesIO()
        PIL.Image.fromarray(image).save(bytes_io, format="JPEG")
        return bytes_io.getvalue()
