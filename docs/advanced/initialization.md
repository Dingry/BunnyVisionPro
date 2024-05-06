# Teleoperation Initialization

Teleoperation systems map human movements to robotic actions. For instance, advancing your right hand by 0.1 meters
should correspondingly move the robot's right end effector forward by the same distance. Establishing a clear definition
of coordinate system for both human and robot through an initialization step is crucial to accurately translating
human motion to robot in three-dimensional space.

## Calibration Between Human and Robot

### Initialization Process

During the initialization phase, we establish a three-dimensional frame in both human and robot coordinate systems,
referred to as the **initial human frame** and the **initial robot frame**, respectively.
Movements by the human operator are measured relative to the initial human frame, and the robot mimics these relative
movements within its corresponding initial robot frame.

### System Configuration

The positions and orientations of these initial frames are configured via the `TeleopClient` API as shown below:

```python
import numpy as np
from bunny_teleop.init_config import BimanualAlignmentMode


class TeleopClient:
  def send_init_config(
    self,
    *,
    robot_base_pose: Tuple[np.ndarray, np.ndarray],
    init_qpos: Tuple[np.ndarray, np.ndarray],
    joint_names: Tuple[List[str], List[str]],
    align_gravity_dir=True,
    bimanual_alignment_mode=BimanualAlignmentMode.ALIGN_CENTER,
  ):
    # Implementation skipped for doc purpose
    pass
```

!!! tip "Checking Initialization Status"

    Use `teleop_client.started` or `teleop_client.wait_for_server_start()` to confirm if the initialization has completed.

## Initializing the Robot Frame

Upon successful operation of the `bunny_teleop_server`, the teleop initialization is triggered by the `send_init_config`
API call. This function requires the initial pose of the robot (`init_qpos`), which represents the joint positions for
both the left and right robotic arms. The teleop server computes the end effector poses using forward kinematics to
determine the initial robot frame.

!!! note "Be Careful about Joint Orders"

    Ensure the joint names are correctly specified as the teleop server may parse the robot model differently, affecting the
    joint order.

## Initializing the Human Frame

Post the API call, the VisionPro system begins tracking the operator's hand poses. To mitigate noise in hand pose
estimation, an average pose is calculated from 200 frames following the initialization call. The initialization
completes once these poses are processed, allowing the teleop server to begin streaming commands.

!!! note "Good Hand Poses are Important for Initialization"

    The frame counter resets to 0 if the operator's hands are not held still or if the fingers are not spread out flat.
    If the teleop_server has not started within 5 seconds after initialization, check the position and condition of both 
    hands. 

    An example of a proper hand pose is shown below:

    ![good init pose](../assets/images/good_init_pose.jpg){: style="width:50%"}

## Bimanual Alignment Modes

### Overview

In the tutorial above, we skip how to determine the initial frames for the left and right hands.
Essentially, we can treat each hand as a separate entity with its own frame, similar to individual robots,
or use a single frame for controlling both hands in bimanual operations.
Depending on the task, different configurations might be preferable.
Our system allows users to select the desired mode through the `bimanual_alignment_mode` variable in the
`send_init_config` function, offering various options to best suit specific task requirements.

```python
class BimanualAlignmentMode(Enum):
  ALIGN_CENTER = 0
  ALIGN_LEFT = 1
  ALIGN_RIGHT = 2
  ALIGN_SEPARATELY = 3
```

### Mode Descriptions

- **ALIGN_SEPARATELY**: Treats each arm as an independent entity, ideal for tasks requiring distinct movements.
- **ALIGN_CENTER**: Calculates a center between the two robot end effectors, aligning this with the midpoint between
  the human hands.
- **ALIGN_LEFT** and **ALIGN_RIGHT**: Focuses on one hand's position and orientation, useful for tasks prioritizing one
  side.

!!! tip "Global Pose of Robot Base"

    To provide the teleoperation server with the positions of the two robot arms, the `robot_base_pose` variable is used. 
    This variable, a tuple consisting of two 7-dimensional numpy arrays, specifies the position and orientation (using the
    quaternion wxyz convention) of each robot's base.

!!! danger

    For modes except `ALIGN_SEPARATELY` Robot arms may move instantly after initialization is finished even if the human operator keeps static.This can be
    dangers on real robot if you are not aware what will happen after initialization. Try first on our simulation version
    before deploying on the real robot.

