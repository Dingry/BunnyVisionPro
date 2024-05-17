from pathlib import Path
from typing import Dict, List, Union, Optional

import cv2
import numpy as np
import sapien
import tqdm
import tyro
from dex_retargeting.retargeting_config import RetargetingConfig
from pytransform3d import rotations
from sapien import Pose
from sapien.asset import create_dome_envmap
from sapien.utils import Viewer


# Convert to webp
# ffmpeg -i teaser.mp4 -vcodec libwebp -lossless 1 -loop 0 -preset default  -an -vsync 0 teaser.webp


def render_by_sapien(
    meta_data: Dict,
    data: List[Union[List[float], np.ndarray]],
    output_video_path: Optional[str] = None,
):
    # Config is loaded only to find the urdf path and robot name
    config_paths = meta_data["config_paths"]
    left_config = RetargetingConfig.load_from_file(config_paths[0])
    right_config = RetargetingConfig.load_from_file(config_paths[1])

    record_video = output_video_path is not None
    headless = record_video
    if not record_video:
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")
    else:
        sapien.render.set_camera_shader_dir("rt")
        sapien.render.set_ray_tracing_samples_per_pixel(64)
        sapien.render.set_ray_tracing_path_depth(8)
        sapien.render.set_ray_tracing_denoiser("oidn")

    scene = sapien.Scene()

    ground_pose = sapien.Pose(q=[-0.707, 0.707, 0, 0], p=[-1, 0, 0])
    scene.add_directional_light(np.array([-0.2, -1, -0.2]), np.array([2, 2, 2]), shadow=True)
    scene.set_environment_map(
        create_dome_envmap(sky_color=[0.2, 0.2, 0.2], ground_color=[0.2, 0.2, 0.2])
    )

    scene.add_area_light_for_ray_tracing(
        sapien.Pose(q=[-0.707, 0.707, 0, 0], p=[-1, 0, 0])
        * sapien.Pose([2, 1, 2], [0.707, 0, 0.707, 0]),
        np.array([1, 1, 1]),
        2,
        2,
    )

    # Ground
    render_mat = sapien.render.RenderMaterial()
    render_mat.base_color = [0.06, 0.08, 0.12, 1]
    render_mat.metallic = 0.0
    render_mat.roughness = 0.9
    render_mat.specular = 0.8
    ground = scene.add_ground(
        0, render_material=render_mat, render_half_size=[1000, 1000]
    )
    ground.set_pose(ground_pose)

    # Load robot and set it to a good pose to take picture
    loader = scene.create_urdf_loader()
    left_robot = loader.load(left_config.urdf_path.replace(".urdf", "_glb.urdf"))
    right_robot = loader.load(right_config.urdf_path.replace(".urdf", "_glb.urdf"))

    camera = scene.add_camera(
        name="Cheese!", width=960, height=540, fovy=1, near=0.1, far=10
    )
    camera.set_local_pose(
        Pose(
            [-0.03, 0.8, 0.83],
            [0.707, 0, 0, -0.707],
        )
    )

    camera_pose = camera.get_local_pose()
    pos = camera_pose.p
    if "svh" in left_config.urdf_path or "ability" in left_config.urdf_path:
        pos[2] += 0.1
    if "allegro" in left_config.urdf_path:
        pos[2] += 0.05
    camera_pose.set_p(pos)
    camera.set_local_pose(camera_pose)

    # Setup onscreen viewer if not headless
    if not headless:
        viewer = Viewer()
        viewer.set_scene(scene)
        viewer.set_camera_pose(camera.get_local_pose())
        viewer.paused = True
    else:
        viewer = None

    # Setup video recorder
    if record_video:
        writer = cv2.VideoWriter(
            output_video_path, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (camera.get_width(), camera.get_height())
        )

    # Different robot loader may have different orders for joints
    sapien_left_joint_names = [
        joint.get_name() for joint in left_robot.get_active_joints()
    ]
    sapien_right_joint_names = [
        joint.get_name() for joint in right_robot.get_active_joints()
    ]
    retargeting_joint_names = meta_data["joint_names"]

    left_retargeting_to_sapien = np.array(
        [retargeting_joint_names[0].index(name) for name in sapien_left_joint_names]
    ).astype(int)
    right_retargeting_to_sapien = np.array(
        [retargeting_joint_names[1].index(name) for name in sapien_right_joint_names]
    ).astype(int)

    for i in tqdm.trange(0, len(data["left_qpos"]), 2):
        left_robot.set_qpos(np.array(data["left_qpos"][i])[left_retargeting_to_sapien])
        right_robot.set_qpos(
            np.array(data["right_qpos"][i])[right_retargeting_to_sapien]
        )

        left_pose = data["left_pose"][i]
        right_pose = data["right_pose"][i]
        left_quat = rotations.quaternion_from_matrix(left_pose[:3, :3])
        right_quat = rotations.quaternion_from_matrix(right_pose[:3, :3])
        left_robot.set_pose(sapien.Pose(left_pose[:3, 3], left_quat))
        right_robot.set_pose(sapien.Pose(right_pose[:3, 3], right_quat))

        if not headless:
            for _ in range(1):
                viewer.render()

        if record_video:
            scene.update_render()
            camera.take_picture()
            rgb = camera.get_picture_cuda("Color").torch()[..., :3].cpu().numpy()
            rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
            writer.write(rgb[..., ::-1])

    if record_video:
        writer.release()

    scene = None


def main(
    pickle_path: str,
    output_video_path: Optional[str] = None,
):
    """
    Loads the preserved robot pose data and renders it either on screen or as an mp4 video.

    Args:
        pickle_path: Path to the .pickle file, created by `detect_from_video.py`.
        output_video_path: Path where the output video in .mp4 format would be saved.
            By default, it is set to None, implying no video will be saved.
    """
    robot_dir = Path(__file__).absolute().parent / "dex-urdf" / "robots" / "hands"
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))

    pickle_data = np.load(pickle_path, allow_pickle=True)
    meta_data, data = pickle_data["meta_data"], pickle_data["data"]

    render_by_sapien(meta_data, data, output_video_path)


if __name__ == "__main__":
    tyro.cli(main)
