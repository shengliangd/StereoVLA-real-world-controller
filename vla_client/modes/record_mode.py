from ..utils import input_typed, TrajectoryRecorder
from ..utils.cameras import CameraVisualizer
from ..cameras import MultiCamera
from ..controllers import common

import pynput
from pynput import keyboard
import transforms3d as t3d
import controllers
import random
 
import datetime
import numpy as np
import io
import PIL.Image
import termios
import sys
import time
import threading
import rospy
import os


class RecordMode:
    HZ = 10
    def __init__(self, args, only_grasp=True):
        self.args = args
        self.only_grasp = only_grasp

        self.multi_camera = MultiCamera(args.cameras)

        self.camera_visualizer = CameraVisualizer(self.multi_camera)

        self.blocking = self.args.controller == "blocking"
        self.robot_controller = controllers.FrankaROSController("logical" if self.blocking else "physical", args.extended_finger, handle_abnormal=False)

        self.robot_lock = threading.RLock()

    def run(self):
        while True:
            self.robot_controller._try_recover_from_error()
            self._goto_random_init()
            self.robot_controller.reset_history()

            termios.tcflush(sys.stdin, termios.TCIOFLUSH)
            task_instruction = input_typed('Enter task instruction (empty string for exit): ', str)
            if task_instruction == '':
                break
            rate = rospy.Rate(self.HZ)
            # wait for user to put the robot into guide mode
            print("please guide the robot, recording will start automatically")
            while self.robot_controller.robot_mode not in [3]:
                rate.sleep()
            print("recording...")
            # record joint states for the task
            commands = []
            while self.robot_controller.robot_mode in self.robot_controller.ROBOT_MODES_UNCONTROLLABLE:
                joint_state = self.robot_controller.get_qpos()
                eef_pose = self.robot_controller.get_latest_eef_pose()
                eef_position = eef_pose[:3]
                eef_orientation = eef_pose[3:6]
                commands.append((joint_state, (eef_position, eef_orientation)))
                rate.sleep()
            self.robot_controller._try_recover_from_error()
            self.robot_controller.gripper_status = None

            # start recording
            self._replay_and_record(task_instruction, commands)
            print('GO TO PREPARE FOR RELEASE!!!!!')
            for i in reversed(range(5)):
                print(f'REMAINING TIME: {i}')
                time.sleep(1)
            print('RELEASED!!!!!')

    def _replay_and_record(self, instruction, commands):
        print(f"start replaying and recording the episode... controller mode: {self.robot_controller.robot_mode} len of commands:{len(commands)}")
        # first, go to the initial position
        if len(commands) == 0:
            return
        
        # move to first command position
        first_pos, first_ori = commands[0][1]
        first_ori_mat = t3d.euler.euler2mat(*first_ori)
        self.robot_controller.execute_waypoints([(first_pos, first_ori_mat, 1)])
        self.robot_controller.wait_for_reach(common.POS_TOL, common.ROT_TOL, 10)

        print(f'start replay!!!')

        self.finished = False
        self.paused = False
        folder = f'{self.args.save_to}-{datetime.datetime.now().strftime("%Y%m%d%H%M%S")}'
        recorder = TrajectoryRecorder(folder=folder)
        rate = rospy.Rate(self.HZ)
        with pynput.keyboard.Listener(on_press=self._on_keyboard_press, on_release=self._on_keyboard_release):
            for joint_command, (eef_command_position, eef_command_orientation) in commands:
                if self.paused:
                    time.sleep(0.1)
                    continue

                self.robot_controller.execute_waypoints([(eef_command_position, t3d.euler.euler2mat(*eef_command_orientation), 0)])
                rate.sleep()
                if sum(joint_command[-2:]) < 0.075 and self.robot_controller.gripper_status == 'open':
                    self.robot_controller.move_gripper('close')
                elif sum(joint_command[-2:]) > 0.075 and self.robot_controller.gripper_status == 'close':
                    self.robot_controller.move_gripper('open')

                # record data
                images = self.multi_camera.get_frames()
                assert len(images) == 2, 'currently only support two-camera setting'
                joint_positions = self.robot_controller.get_qpos()
                eef_pose = self.robot_controller.get_latest_eef_pose()
                step_info = {
                    "left_rgb": self._compress_image(images[0]),
                    "right_rgb": self._compress_image(images[1]),
                    "joint_position": joint_positions,
                    "eef_position": eef_pose[:3],
                    "eef_orientation": eef_pose[3:6],
                    "eef_width": sum(joint_positions[-2:]),
                    "command_joint_position": joint_command,
                    "command_eef_position": eef_command_position,
                    "command_eef_orientation": eef_command_orientation,
                    "command_eef_width": sum(joint_command[-2:]),
                    "gripper_status": {'open': 1, 'close': -1}[self.robot_controller.gripper_status],
                }
                recorder.submit_step(step_info)

                # update tf for debugging
                self.robot_controller.update_tf('proprio', eef_command_position, eef_command_orientation)

        
        recorder.submit_metadata({
            "instruction": instruction,
            "hz": self.HZ,
            "timestamp": datetime.datetime.now().strftime("%Y%m%d%H%M%S"),
        })
        recorder.finish()
        keep = input_typed('keep? (y/n): ', bool)
        if not keep:
            os.rename(folder, folder+'-invalid')

    def _on_keyboard_press(self, key):
        if hasattr(key, "char"):
            # open gripper
            if key.char == '=':
                print('force open gripper...')
                with self.robot_lock:
                    self.paused = True
                    self.robot_controller.move_gripper('open')
                    print('gripper opened, and paused')
            # close gripper
            elif key.char == '-':
                print('force close gripper...')
                with self.robot_lock:
                    self.paused = True
                    self.robot_controller.move_gripper('close')
                    print('gripper closed, and paused')
            else:
                pass
        else:
            if key == keyboard.Key.space:
                with self.robot_lock:
                    self.paused = not self.paused
                print('pause' if self.paused else 'resume')
            elif key == keyboard.Key.esc:
                exit(1)

    def _on_keyboard_release(self, key):
        import sys
        if hasattr(key, "char") or key in [keyboard.Key.space]:
            sys.stdout.write('\b \b')
            sys.stdout.flush()

    def _goto_random_init(self):
        self.robot_controller.move_gripper('open')
        init_pos = np.array([0.36307235, -0.00142398, 0.48912046]) + np.random.uniform(low=[-0.10, -0.20, -0.10], high=[0.10, 0.20, 0.10], size=3)
        init_ori_euler = np.array([-np.pi, 0., 0.]) + np.random.uniform(low=[-10/180*np.pi, -10/180*np.pi, -np.pi/3], high=[10/180*np.pi, 10/180*np.pi, np.pi/3])
        init_ori_mat = t3d.euler.euler2mat(*init_ori_euler)
        self.robot_controller.execute_waypoints([(init_pos, init_ori_mat, -1)])
        self.robot_controller.wait_for_reach(common.POS_TOL, common.ROT_TOL, 5)
        self.robot_controller.execute_waypoints([(init_pos, init_ori_mat, 1)])
        self.robot_controller.wait_for_reach(common.POS_TOL, common.ROT_TOL, 5)

    def _compress_image(self, image):
        bytes_io = io.BytesIO()
        PIL.Image.fromarray(image).save(bytes_io, format="JPEG")
        return bytes_io.getvalue()
