import dataclasses
from enum import Enum
from typing import List, Tuple

import numpy as np


class BimanualAlignmentMode(Enum):
    ALIGN_CENTER = 0
    ALIGN_LEFT = 1
    ALIGN_RIGHT = 2
    ALIGN_SEPARATELY = 3


@dataclasses.dataclass
class InitializationConfig:
    robot_base_pose: Tuple[np.ndarray, np.ndarray]
    init_qpos: Tuple[np.ndarray, np.ndarray]
    joint_names: Tuple[List[str], List[str]]

    # Initialization type
    align_gravity_dir: bool = True
    bimanual_alignment_mode: BimanualAlignmentMode = BimanualAlignmentMode.ALIGN_CENTER

    def __post_init__(self):
        for i in range(len(self.init_qpos)):
            self.init_qpos[i][:] = self.init_qpos[i].astype(np.float64)[:]
        for i in range(len(self.robot_base_pose)):
            self.robot_base_pose[i][:] = self.robot_base_pose[i].astype(np.float64)[:]

    def validate(self, dof_left: int, dof_right: int):
        if len(self.robot_base_pose) != 7:
            raise ValueError(f"robot_base_pose should be a 7d vector")
        if self.init_qpos[0].shape[0] != dof_left:
            raise ValueError(
                f"init_qpos[0] should be a {dof_left}d vector, the same dim as dof"
            )
        if self.init_qpos[1].shape[0] != dof_right:
            raise ValueError(
                f"init_qpos[1] should be a {dof_right}d vector, the same dim as dof"
            )
        if len(self.joint_names[0]) != dof_left:
            raise ValueError(
                f"joint_names[0] should be a {dof_left}d vector, the same dim as dof"
            )
        if len(self.joint_names[1]) != dof_right:
            raise ValueError(
                f"joint_names[0] should be a {dof_right}d vector, the same dim as dof"
            )

    def get_joint_index_mapping(
            self, server_side_joint_names, hand_index: int
    ) -> np.ndarray:
        # Build index mapping.
        # Note that different URDF parser of the same robot may result in different joint order,
        # Especially for multi-finger robot which is a kinematic tree rather than a single kinematics chain
        # lula_qpos[index_lula2optimizer] = retargeted_qpos
        # server_side_qpos[index_server2client] = client_side_qpos
        index_client2server = [
            self.joint_names[hand_index].index(name) for name in server_side_joint_names
        ]
        index_client2server = np.array(index_client2server, dtype=int)
        return index_client2server

    def to_dict(self):
        return dict(
            robot_base_pose=[
                self.robot_base_pose[0].tolist(),
                self.robot_base_pose[1].tolist(),
            ],
            init_qpos=(self.init_qpos[0].tolist(), self.init_qpos[1].tolist()),
            joint_names=self.joint_names,
            align_gravity_dir=self.align_gravity_dir,
            bimanual_alignment_mode=self.bimanual_alignment_mode.value,
        )

    @classmethod
    def from_dict(cls, config):
        return InitializationConfig(
            robot_base_pose=(
                np.array(config["robot_base_pose"][0]),
                np.array(config["robot_base_pose"][1]),
            ),
            init_qpos=(
                np.array(config["init_qpos"][0]),
                np.array(config["init_qpos"][1]),
            ),
            joint_names=config["joint_names"],
            align_gravity_dir=config["align_gravity_dir"],
            bimanual_alignment_mode=BimanualAlignmentMode(config["bimanual_alignment_mode"]),
        )
