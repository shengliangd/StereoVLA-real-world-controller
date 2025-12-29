import rospy
import sys
sys.path.append('./vla_client')
from vla_client.modes import GraspMode, RecordMode
import argparse

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mode", default="grasp", choices=["grasp", "instruct", "record"])
arg_parser.add_argument("--controller", default="blocking", choices=["blocking", "non-blocking"])
arg_parser.add_argument("--cameras", type=str, required=True, help="format: camera_type,camera_id[:camera_type,camera_id]...")
arg_parser.add_argument("--save-to", type=str, help="directory for saving the experiment record, will auto add timestamp")
arg_parser.add_argument("--server-ip", type=str, required=True)
arg_parser.add_argument("--server-port", type=int, required=True)
arg_parser.add_argument("--robot-ip", type=str, default="172.16.0.2")
arg_parser.add_argument("--extended-finger", type=int, required=True, help="use extended finger or not")


if __name__ == "__main__":
    args = arg_parser.parse_args()

    rospy.init_node('vla_controller', anonymous=True, disable_signals=True)

    if args.mode == "grasp":
        mode = GraspMode(args, only_grasp=True)
    elif args.mode == "instruct":
        mode = GraspMode(args, only_grasp=False)
    elif args.mode == "record":
        mode = RecordMode(args)
    else:
        raise NotImplementedError(f"mode {args.mode} not implemented")

    mode.run()

    rospy.signal_shutdown('done')
