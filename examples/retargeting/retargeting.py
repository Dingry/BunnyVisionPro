import pickle
from pathlib import Path
from typing import List

import numpy as np
import tqdm
import tyro
from dex_retargeting.constants import (
    RobotName,
    RetargetingType,
    HandType,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
from dex_retargeting.seq_retarget import SeqRetargeting

OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ]
)

OPERATOR2AVP_RIGHT = OPERATOR2MANO_RIGHT

OPERATOR2AVP_LEFT = np.array(
    [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]
)


def three_mat_mul(left_rot: np.ndarray, mat: np.ndarray, right_rot: np.ndarray):
    result = np.eye(4)
    rotation = left_rot @ mat[:3, :3] @ right_rot
    pos = left_rot @ mat[:3, 3]
    result[:3, :3] = rotation
    result[:3, 3] = pos
    return result


def two_mat_batch_mul(batch_mat: np.ndarray, left_rot: np.ndarray):
    result = np.tile(np.eye(4), [batch_mat.shape[0], 1, 1])
    result[:, :3, :3] = np.matmul(left_rot[None, ...], batch_mat[:, :3, :3])
    result[:, :3, 3] = batch_mat[:, :3, 3] @ left_rot.T
    return result


def joint_avp2hand(finger_mat: np.ndarray):
    finger_index = np.array([0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24])
    finger_mat = finger_mat[finger_index]
    return finger_mat


def filter_data(data: List, fps, duration):
    init_time = data[0]["time"]
    all_times = np.array([d["time"] for d in data]) - init_time
    step = 1.0 / fps
    new_data = []
    for i in range(fps * duration):
        current_time = i * step
        diff = np.abs(all_times - current_time)
        best_match = np.argmin(diff)
        new_data.append(data[best_match])
    return new_data


def retarget_video(
    left_retargeting: SeqRetargeting,
    right_retargeting: SeqRetargeting,
    data_path: str,
    output_path: str,
    config_paths: List[str],
):
    data = np.load(data_path, allow_pickle=True)
    data = filter_data(data, fps=30, duration=15)
    length = len(data)

    left_qpos_list = []
    right_qpos_list = []
    left_pose_list = []
    right_pose_list = []

    with tqdm.tqdm(total=length) as pbar:
        for i in range(length):
            single_data = data[i]
            for hand_num, retargeting in enumerate([left_retargeting, right_retargeting]):
                if hand_num == 0:
                    joint_pose = two_mat_batch_mul(single_data["left_fingers"], OPERATOR2AVP_LEFT.T)
                else:
                    joint_pose = two_mat_batch_mul(single_data["right_fingers"], OPERATOR2AVP_RIGHT.T)

                joint_pos = joint_avp2hand(joint_pose)[:, :3, 3]

                indices = retargeting.optimizer.target_link_human_indices
                origin_indices = indices[0, :]
                task_indices = indices[1, :]
                ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]

                qpos = retargeting.retarget(ref_value)

                if hand_num == 0:
                    left_qpos_list.append(qpos)
                    wrist_pose = single_data["left_wrist"][0].copy()
                    wrist_pose[:3, :3] = wrist_pose[:3, :3] @ OPERATOR2AVP_LEFT
                    left_pose_list.append(wrist_pose.copy())
                else:
                    right_qpos_list.append(qpos)
                    wrist_pose = single_data["right_wrist"][0].copy()
                    wrist_pose[:3, :3] = wrist_pose[:3, :3] @ OPERATOR2AVP_RIGHT
                    right_pose_list.append(wrist_pose.copy())

            pbar.update(1)

        meta_data = dict(
            config_paths=config_paths,
            dofs=[
                len(left_retargeting.optimizer.robot.dof_joint_names),
                len(right_retargeting.optimizer.robot.dof_joint_names),
            ],
            joint_names=[
                left_retargeting.optimizer.robot.dof_joint_names,
                right_retargeting.optimizer.robot.dof_joint_names,
            ],
        )

        result = {
            "left_qpos": np.stack(left_qpos_list),
            "right_qpos": np.stack(right_qpos_list),
            "left_pose": np.stack(left_pose_list),
            "right_pose": np.stack(right_pose_list),
        }

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("wb") as f:
            pickle.dump(dict(data=result, meta_data=meta_data), f)


def main(
    robot_name: RobotName,
    data_path: str,
    output_path: str,
):
    """
    Detects the human hand pose from a video and translates the human pose trajectory into a robot pose trajectory.

    Args:
        robot_name: The identifier for the robot. This should match one of the default supported robots.
        data_path: The file path for the input offline dump from Apple VisionPro in .pkl format.
        output_path: The file path for the output data in .mp4 format.
    """

    robot_dir = Path(__file__).absolute().parent / "dex-urdf" / "robots" / "hands"
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    left_config_path = get_default_config_path(robot_name, RetargetingType.dexpilot, HandType.left)
    right_config_path = get_default_config_path(robot_name, RetargetingType.dexpilot, HandType.right)
    left_retargeting = RetargetingConfig.load_from_file(left_config_path).build()
    right_retargeting = RetargetingConfig.load_from_file(right_config_path).build()
    retarget_video(
        left_retargeting,
        right_retargeting,
        data_path,
        output_path,
        [str(left_config_path), str(right_config_path)],
    )


if __name__ == "__main__":
    tyro.cli(main)
