## Installation
install the necessary package with the following command:

```bash
pip install -e .[real_control]
```

install the XArm Python SDK:

```bash
git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
cd xArm-Python-SDK
python setup.py install
```

install the Ability hand python wrapper:
```bash
git clone https://github.com/Dingry/AbilityHandWrapper 
cd AbilityHandWrapper
pip install -e .
```

## Usage

First, make sure the server is started following [this instruction](https://dingry.github.io/BunnyVisionPro/getstarted/usage/#bunny-teleoperation-server-setup).

Then, run the following command to start the teleoperation client:

```bash
python teleop_bimanual_xarm7_ability.py
```
Please note that you need to specify both the IP address of your XArm7 and the USB port used for connecting the Ability Hand in the teleop_bimanual_xarm7_ability.py file.
```python
    left_robot = XArm7Ability(
        use_arm=USE_REAL_ARM,
        use_hand=USE_REAL_HAND,
        arm_ip="192.168.1.xxx",  # Change to your XArm7 IP address
        hand_tty_index="usb-xxx",  # Change to your Ability Hand USB port
        is_right=False,
        use_servo_control=use_servo_control,
    )
    right_robot = XArm7Ability(
        use_arm=USE_REAL_ARM,
        use_hand=USE_REAL_HAND,
        arm_ip="192.168.1.xxx",  # Change to your XArm7 IP address
        hand_tty_index="usb-xxx",  # Change to your Ability Hand USB port
        is_right=True,
        use_servo_control=use_servo_control,
    )
```



#### Configuration Options
You can customize the key configurations in the `teleop_bimanual_xarm7_ability.py` file.
- Bimanual alignment mode during Initialization [[Documentation]](https://dingry.github.io/BunnyVisionPro/advanced/initialization/#bimanual-alignment-modes):

```python
client.send_init_config(
    robot_base_pose=(
        np.array([0, 0.4, 0, 1, 0, 0, 0]),
        np.array([0, -0.4, 0, 1, 0, 0, 0]),
    ),
    init_qpos=bimanual_init_qpos,
    joint_names=joint_names,
    bimanual_alignment_mode=BimanualAlignmentMode.ALIGN_CENTER,  # Options: ALIGN_CENTER, ALIGN_LEFT, ALIGN_RIGHT, ALIGN_SEPARETELY
    align_gravity_dir=True,
)
```

- Initial QPose of the robots:

```python
left_init_qpos = np.concatenate([np.array([-26, -14.4, 1.8, 13.2, 180, 52.9, -30.5]) / 180 * np.pi, np.zeros(10)])
right_init_qpos = np.concatenate([np.array([48.1, 5.2, 1.9, 13.1, 174.6, 91.4, 46.8]) / 180 * np.pi, np.zeros(10)])
```

- Saving data during teleoperation:

```python
TASK_NAME = "task_name"
SAVE_DATA = True
```
