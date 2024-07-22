import h5py
import glob
import os
import threading
import time
from enum import Enum, auto
from pathlib import Path
from pynput import keyboard
import numpy as np

from yourdfpy import URDF
from xarm7_ability import XArm7Ability
from bunny_teleop.init_config import BimanualAlignmentMode
from bunny_teleop.bimanual_teleop_client import TeleopClient

TASK_NAME = "bimanual_grasp"
USE_REAL_HAND = True
USE_REAL_ARM = True

SAVE_DATA = False


class DataStatus(Enum):
    NOT_STARTED = auto()
    STARTED = auto()
    FINISHED = auto()
    SAVED = auto()
    ABORTED = auto()


# Demonstration collection
if SAVE_DATA:
    DATA_DIR = Path(__file__).parent.parent / "teleop_data" / TASK_NAME / "robot_data_raw"
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    CURR_EPISODE_NUM, CURR_EPISODE = None, None

    def set_new_episode():
        global CURR_EPISODE_NUM, CURR_EPISODE, DATA_DIR
        OLD_EPISODES = sorted(glob.glob(str(DATA_DIR / "episode_*")), key=os.path.getmtime)
        CURR_EPISODE_NUM = int(OLD_EPISODES[-1].split("/")[-1][8:]) + 1 if OLD_EPISODES else 0
        CURR_EPISODE = f"episode_{CURR_EPISODE_NUM}"

        print("Current task:", TASK_NAME)
        print("Current episode:", CURR_EPISODE)

        print("Data will be saved in:", DATA_DIR / CURR_EPISODE)
        print("Press 's' to start saving data, 'q' to finish saving data.")

    set_new_episode()

    data_status = DataStatus.NOT_STARTED

    def on_press(key):
        global data_status
        try:
            if key.char == "n" and data_status == DataStatus.SAVED:
                print(">>> Start new episode")
                set_new_episode()
                data_status = DataStatus.NOT_STARTED
            if key.char == "s" and data_status == DataStatus.NOT_STARTED:
                print(">>> Start capturing data")
                data_status = DataStatus.STARTED
            if key.char == "q" and data_status == DataStatus.STARTED:
                print(">>> Finish capturing data")
                data_status = DataStatus.FINISHED
                print("Press 'n' to start a new episode.")
            if key.char == "a" and data_status == DataStatus.STARTED:
                print(">>> Abort this episode")
                data_status = DataStatus.ABORTED
                print("Press 'n' to start a new episode.")
            print(f"Pressed {key.char}")
        except AttributeError:
            pass  # Ignore special keys

    def start_listener():
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    # Start the keyboard listener on a separate thread
    listener_thread = threading.Thread(target=start_listener)
    listener_thread.start()


def main():
    asset_path = (Path(__file__).parent.parent / "examples/assets").resolve()

    # Load a yourdfpy instance only for forward kinematics computation
    left_urdf_path = asset_path / "urdf/assembly/xarm7_ability/xarm7_ability_left_hand.urdf"
    left_robot = URDF.load(str(left_urdf_path))

    right_urdf_path = asset_path / "urdf/assembly/xarm7_ability/xarm7_ability_right_hand.urdf"
    right_robot = URDF.load(str(right_urdf_path))

    # Robot initial state
    robots = [left_robot, right_robot]
    joint_names = tuple(robot.actuated_joint_names for robot in robots)

    # Initial joint positions of the robots
    left_init_qpos = np.concatenate([np.array([-26, -14.4, 1.8, 13.2, 180, 52.9, -30.5]) / 180 * np.pi, np.zeros(10)])
    right_init_qpos = np.concatenate([np.array([48.1, 5.2, 1.9, 13.1, 174.6, 91.4, 46.8]) / 180 * np.pi, np.zeros(10)])

    bimanual_init_qpos = (left_init_qpos, right_init_qpos)

    client = TeleopClient(port=5500, cmd_dims=(17, 17), host="localhost")
    client.send_init_config(
        robot_base_pose=(
            np.array([0, 0.4, 0, 1, 0, 0, 0]),
            np.array([0, -0.4, 0, 1, 0, 0, 0]),
        ),
        init_qpos=bimanual_init_qpos,
        joint_names=joint_names,
        bimanual_alignment_mode=BimanualAlignmentMode.ALIGN_CENTER,  # ALIGN_CENTER, ALIGN_LEFT, ALIGN_RIGHT, ALIGN_SEPARETELY
        align_gravity_dir=True,
    )

    print("============= Waiting for server to start...")
    client.wait_for_server_start()
    print("============= Server started.")

    if USE_REAL_HAND or USE_REAL_ARM:
        use_servo_control = False

        # Create real robot interface for real control
        left_robot = XArm7Ability(
            use_arm=USE_REAL_ARM,
            use_hand=USE_REAL_HAND,
            arm_ip="192.168.1.xxx",
            hand_tty_index="usb-xxx",
            is_right=False,
            use_servo_control=use_servo_control,
        )
        right_robot = XArm7Ability(
            use_arm=USE_REAL_ARM,
            use_hand=USE_REAL_HAND,
            arm_ip="192.168.1.xxx",
            hand_tty_index="usb-xxx",
            is_right=True,
            use_servo_control=use_servo_control,
        )

        # Special handling the motion from initialization
        left_error, right_error = 1e5, 1e5
        error_threshold = np.deg2rad(2)
        left_robot.reset()
        right_robot.reset()

        if USE_REAL_ARM:
            # For safety, we set the velocity to a lower value during initialization
            if not use_servo_control:
                original_velocity_limit = left_robot.max_arm_velocity
                new_velocity_limit = original_velocity_limit / 3
                left_robot.max_arm_velocity = new_velocity_limit
                right_robot.max_arm_velocity = new_velocity_limit
            else:
                original_max_velocity = left_robot.arm_velocity_limit
                new_velocity_limit = original_max_velocity / 3
                left_robot.arm_velocity_limit = new_velocity_limit
                right_robot.arm_velocity_limit = new_velocity_limit
            qpos_list = client.get_teleop_cmd()
            left_robot.control_arm_qpos(qpos_list[0][:7])
            right_robot.control_arm_qpos(qpos_list[1][:7])
            left_robot.start()
            right_robot.start()

            print("============== Initializing robot arm...")
            while np.linalg.norm(left_error) > error_threshold or np.linalg.norm(right_error) > error_threshold:
                # print(np.linalg.norm(left_error), np.linalg.norm(right_error), error_threshold)
                left_robot.wait_until_next_control_signal()
                qpos_list = client.get_teleop_cmd()
                left_robot.control_arm_qpos(qpos_list[0][0:7])
                right_robot.control_arm_qpos(qpos_list[1][0:7])
                left_error = qpos_list[0][0:7] - left_robot.get_arm_qpos()
                right_error = qpos_list[1][0:7] - right_robot.get_arm_qpos()

            print("============== Initialization finished.")

            # Set back the robot velocity to original
            if not use_servo_control:
                left_robot.max_arm_velocity = original_velocity_limit
                right_robot.max_arm_velocity = original_velocity_limit
            else:
                left_robot.arm_velocity_limit = original_max_velocity
                right_robot.arm_velocity_limit = original_max_velocity

        print(f"Initialization finished with error: {left_error}, {right_error}")
        # left_robot.stop()

    # Initialization of data collection
    data = {"action": [], "hand0": [], "hand1": [], "arm0": [], "arm1": []}
    timestamps = []
    global data_status, CURR_EPISODE_NUM, CURR_EPISODE

    try:
        left_robot.hand.start_process()
        right_robot.hand.start_process()

        while True:
            if SAVE_DATA and data_status == DataStatus.STARTED:
                saving_data = True
            else:
                saving_data = False

            qpos_list = client.get_teleop_cmd()
            if saving_data:
                data["action"].append(qpos_list[0].tolist() + qpos_list[1].tolist())
                timestamps.append([time.time()])

            if USE_REAL_ARM or USE_REAL_HAND:
                if saving_data:
                    data["hand0"].append(left_robot.get_hand_data())
                    timestamps[-1].append(time.time())
                    data["hand1"].append(right_robot.get_hand_data())
                    timestamps[-1].append(time.time())
                    left_arm_data = left_robot.get_arm_data()
                    data["arm0"].append(left_arm_data)
                    timestamps[-1].append(time.time())
                    right_arm_data = right_robot.get_arm_data()
                    data["arm1"].append(right_arm_data)
                    timestamps[-1].append(time.time())

                # Wait until next control signal, we only need to call this function for a single arm even with two
                left_robot.wait_until_next_control_signal()

                # Control hand
                left_robot.control_hand_qpos(qpos_list[0][7:])
                right_robot.control_hand_qpos(qpos_list[1][7:])

                # Left robot
                left_robot.control_arm_qpos(qpos_list[0][0:7])

                # Right robot
                right_robot.control_arm_qpos(qpos_list[1][0:7])

            if SAVE_DATA and data_status == DataStatus.FINISHED:
                os.makedirs(DATA_DIR / CURR_EPISODE, exist_ok=True)
                with h5py.File(DATA_DIR / CURR_EPISODE / "data.h5", "w") as f:
                    for key, value in data.items():
                        f.create_dataset(key, data=value)
                    f.create_dataset("timestamps", data=timestamps)
                np.save(DATA_DIR / CURR_EPISODE / "timestamps.npy", np.array(timestamps))
                print("Data saved in:", DATA_DIR / CURR_EPISODE)
                data_status = DataStatus.SAVED
                data = {"action": [], "hand0": [], "hand1": [], "arm0": [], "arm1": []}
                timestamps = []

            if SAVE_DATA and data_status == DataStatus.ABORTED:
                data_status = DataStatus.SAVED
                data = {"action": [], "hand0": [], "hand1": [], "arm0": [], "arm1": []}
                timestamps = []
                print("Episode aborted.")

    except KeyboardInterrupt:
        print("Keyboard interrupt, shutting down.\n")
        left_robot.stop()
        right_robot.stop()

        # Wait for the listener to finish
        listener_thread.join()


if __name__ == "__main__":
    main()
