import threading
import time
import warnings
from pathlib import Path
from typing import Dict

import numpy as np
import pinocchio as pin


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._prev_err = None
        self._cum_err = 0

    def reset(self):
        self._prev_err = None
        self._cum_err = 0

    def control(self, err, dt):
        if self._prev_err is None:
            self._prev_err = err

        value = (
                self.kp * err
                + self.kd * (err - self._prev_err) / dt
                + self.ki * self._cum_err
        )

        self._prev_err = err
        self._cum_err += dt * err

        return value


class XArm7Ability:
    def __init__(
            self,
            use_arm=True,
            use_hand=True,
            arm_ip="192.168.1.242",
            hand_tty_index=0,
            control_dt=0.02,
            internal_control_dt=1.0 / 250,
            is_right=True,
            use_servo_control=False,
            enable_haptics=False,
            haptics_port=None,
    ):
        self.use_arm = use_arm
        self.use_hand = use_hand
        self.control_dt = control_dt
        self.inner_control_dt = internal_control_dt
        self.is_right = is_right
        self.use_servo_control = use_servo_control
        self.xarm_mode = 1 if use_servo_control else 4

        # Setup arm
        if use_arm:
            from xarm import XArmAPI

            self.arm = XArmAPI(arm_ip, is_radian=True)
            if not self.use_servo_control:

                default_kp = np.array([2, 2, 1, 1, 1, 1, 1]) * 5
                default_kd = default_kp / 20
                default_ki = np.zeros(7)
                self.max_arm_velocity = np.array([0.8, 0.8, 0.8, 0.8, 1.0, 1.0, 1.5])
                self.arm_pid = PIDController(
                    kp=default_kp,
                    ki=default_ki,
                    kd=default_kd,
                )
            else:
                self.arm_velocity_limit = 3.0

            # Setup control thread
            self._arm_thread = threading.Thread(target=self._internal_control_arm_qpos)
            self._arm_lock = threading.Lock()
            self._arm_pos_target = None
            self._arm_should_stop = False

        else:
            self.arm = None

        if use_hand:
            from ability_hand.hand_control import RealAbilityHand

            self.hand = RealAbilityHand(
                usb_port=f"/dev/serial/by-id/{hand_tty_index}",
                reply_mode=0x21,
                hand_address=0x50,
                plot_touch=False,
                verbose=False,
            )
        else:
            self.hand = None

        self.reset()

        self.last_control_time = None

        # Computation model
        urdf_base_path = (
                Path(__file__).parent.parent / "assets/curobo/assets/urdf"
        ).absolute()
        if self.is_right:
            robot_path = (
                    urdf_base_path / "assembly/xarm7_ability/xarm7_ability_right_hand.urdf"
            )
        else:
            robot_path = (
                    urdf_base_path / "assembly/xarm7_ability/xarm7_ability_left_hand.urdf"
            )
        robot_path = robot_path.resolve().absolute()
        self.pin_model: pin.Model = pin.buildModelFromUrdf(str(robot_path))
        self.pin_data: pin.Data = self.pin_model.createData()
        self.frame_id_mapping: Dict[str, int] = {}
        for i, frame in enumerate(self.pin_model.frames):
            self.frame_id_mapping[frame.name] = i
        ee_frame_name = "ee_link"
        self.ee_frame_id = self.frame_id_mapping[ee_frame_name]

    def compute_ee_pose(self, qpos) -> pin.SE3:
        if qpos.shape[0] == 7:
            qpos = np.concatenate([qpos, np.zeros(10)])
        pin.forwardKinematics(self.pin_model, self.pin_data, np.array(qpos))
        ee_pose: pin.SE3 = pin.updateFramePlacement(
            self.pin_model, self.pin_data, self.ee_frame_id
        )
        return ee_pose

    def compute_ik(self, ee_pose: pin.SE3, init_qpos):
        oMdes = ee_pose
        qpos = init_qpos

        if qpos.shape[0] == 7:
            qpos = np.concatenate([qpos, np.zeros(10)])

        for k in range(100):
            pin.forwardKinematics(self.pin_model, self.pin_data, qpos)
            ee_pose = pin.updateFramePlacement(
                self.pin_model, self.pin_data, self.ee_frame_id
            )
            J = pin.computeFrameJacobian(
                self.pin_model, self.pin_data, qpos, self.ee_frame_id
            )
            iMd = ee_pose.actInv(oMdes)
            err = pin.log(iMd).vector
            if np.linalg.norm(err) < 1e-3:
                # print(k, np.linalg.norm(err))
                break

            v = J.T.dot(np.linalg.solve(J.dot(J.T) + 1e-5, err))
            qpos = pin.integrate(self.pin_model, qpos, v * 0.05)
        return qpos

    def reset(self):
        if self.use_arm:
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(self.xarm_mode)
            self.arm.set_state(state=0)
            self.arm.set_mode(self.xarm_mode)
            self.arm.set_state(state=0)

        if self.use_hand:
            joint_angle = [15, 15, 15, 15, -15, 15]
            joint_angle = np.deg2rad(np.array(joint_angle))
            self.hand.set_joint_angle(joint_angle, reply_mode=0x10)
            self.hand.set_joint_angle(joint_angle, reply_mode=0x10)

        time.sleep(0.5)

    def clip_arm_next_qpos(self, target_qpos, velocity_limit=0.8):
        if self.use_arm:
            current_qpos = self.get_arm_qpos()
            error = target_qpos - current_qpos
            motion_scale = np.max(np.abs(error)) / (velocity_limit * self.inner_control_dt)
            safe_control_qpos = current_qpos + error / motion_scale
            return safe_control_qpos

    def clip_arm_velocity(self, arm_qvel: np.ndarray):
        velocity_overshot = np.abs(arm_qvel) / self.max_arm_velocity
        max_overshot = np.max(velocity_overshot)
        if max_overshot > 1 + 1e-4:
            safe_velocity = arm_qvel / max_overshot
            bottleneck_joint = np.argmax(velocity_overshot)
            print(f"Bottleneck joint for velocity clip: joint-{bottleneck_joint + 1} with overshoot {max_overshot}")
        else:
            safe_velocity = arm_qvel
        return safe_velocity

    def control_arm_qpos(self, arm_qpos: np.ndarray):
        with self._arm_lock:
            self._arm_pos_target = arm_qpos

    def _internal_control_arm_qpos(self):
        while True:
            time.sleep(self.inner_control_dt)
            if self._arm_should_stop:
                break

            if self.use_arm:
                with self._arm_lock:
                    arm_qpos = self._arm_pos_target

                code, state = self.arm.get_state()
                if code is not 0:
                    print(f"*" * 100)
                    print(f"Arm error: {code}")
                    print(f"*" * 100)
                    self.use_arm = False

                if self.use_servo_control:
                    safe_control_qpos = self.clip_arm_next_qpos(
                        arm_qpos, velocity_limit=self.arm_velocity_limit
                    )
                    self.arm.set_servo_angle_j(angles=safe_control_qpos)
                else:
                    code, xarm_state = self.arm.get_joint_states(is_radian=True)
                    arm_current_qpos = xarm_state[0]
                    error = arm_qpos - arm_current_qpos
                    qvel = self.arm_pid.control(error, self.inner_control_dt)
                    safe_qvel = self.clip_arm_velocity(qvel)
                    code = self.arm.vc_set_joint_velocity(safe_qvel)

    def control_hand_qpos(self, hand_qpos: np.ndarray):
        self.hand.set_joint_angle(hand_qpos, reply_mode=0x21)

    def wait_until_next_control_signal(self):
        if self.last_control_time is None:
            self.last_control_time = time.perf_counter()
        else:
            dt = time.perf_counter() - self.last_control_time
            if dt < self.control_dt:
                time.sleep(self.control_dt - dt)
            else:
                warnings.warn(
                    f"Control dt: {self.control_dt} can not be reached, actual dt: {dt}"
                )
            self.last_control_time = time.perf_counter()

    def get_arm_qpos(self):
        if self.use_arm:
            code, xarm_state = self.arm.get_joint_states(is_radian=True)
            return xarm_state[0]

    def get_arm_data(self):
        if self.use_arm:
            code, xarm_state = self.arm.get_joint_states(is_radian=True)
            code, xarm_eef_pose = self.arm.get_position(is_radian=True)
            return np.concatenate(
                [np.array([xarm_state]).flatten(), np.array(xarm_eef_pose)]
            )

    def get_hand_data(self):
        return self.hand.get_data().tolist()

    def stop(self):
        if self.use_hand:
            self.hand.stop_process()
        if self.use_arm:
            self.arm.vc_set_joint_velocity(np.zeros(7))
            self._arm_should_stop = True
            self._arm_thread.join()
            # self.arm.motion_enable(enable=False)

    def start(self):
        if self.use_arm:
            self._arm_thread.start()

    # Test Function
    def generate_circle_motion(self, radius, init_qpos, steps):
        pin.forwardKinematics(self.pin_model, self.pin_data, np.array(init_qpos))
        ee_pose: pin.SE3 = pin.updateFramePlacement(
            self.pin_model, self.pin_data, self.ee_frame_id
        )

        center_pos = ee_pose.translation + np.array([0, 0, radius])
        last_qpos = np.array(init_qpos)
        qpos_list = []
        ee_pos_list = []
        for i in np.arange(-np.pi / 2, np.pi / 2 * 3, 2 * np.pi / steps):
            ee_pos = np.array([0, np.cos(i), np.sin(i)]) * radius + center_pos
            new_ee_pose: pin.SE3 = ee_pose.copy()
            new_ee_pose.translation = ee_pos
            new_qpos = self.compute_ik(new_ee_pose, last_qpos)
            last_qpos = new_qpos
            qpos_list.append(new_qpos[:7])
            ee_pos_list.append(ee_pos)

        return np.array(qpos_list), np.array(ee_pos_list)
