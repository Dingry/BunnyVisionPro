from pathlib import Path

import numpy as np
import sapien
from sapien.utils import Viewer

from bunny_teleop.bimanual_teleop_client import TeleopClient
from bunny_teleop.init_config import BimanualAlignmentMode


def main():
    asset_path = (Path(__file__).parent.parent / "assets").resolve().absolute()
    scene = sapien.Scene()

    # Load robot into the scene
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    loader.load_multiple_collisions_from_file = True
    left_urdf_path = asset_path / "urdf/assembly/xarm7_ability/xarm7_ability_left_hand.urdf"
    right_urdf_path = asset_path / "urdf/assembly/xarm7_ability/xarm7_ability_right_hand.urdf"
    left_robot = loader.load(str(left_urdf_path))
    right_robot = loader.load(str(right_urdf_path))

    # Setup light
    scene.set_ambient_light(np.array([0.6, 0.6, 0.6]))
    scene.add_directional_light(np.array([1, 1, -1]), np.array([1, 1, 1]))
    scene.add_point_light(np.array([2, 2, 2]), np.array([1, 1, 1]), shadow=False)
    scene.add_point_light(np.array([2, -2, 2]), np.array([1, 1, 1]), shadow=False)

    # Setup ground
    visual_material = sapien.render.RenderMaterial()
    visual_material.set_base_color(np.array([0.5, 0.5, 0.5, 1]))
    visual_material.set_roughness(0.7)
    visual_material.set_metallic(1)
    visual_material.set_specular(0.04)
    scene.add_ground(-1, render_material=visual_material)

    # Set robot initial pose and qpos
    left_robot_base_pose = sapien.Pose([0, 0.5, 0])
    right_robot_base_pose = sapien.Pose([0, -0.5, 0])
    left_robot.set_pose(left_robot_base_pose)
    right_robot.set_pose(right_robot_base_pose)
    init_qpos = [-0.03141593, 0.13439035, 0.03141593, 0.23911011, 3.14159265, 1.46433124, -0.00349066] + [0] * 10
    init_qpos = np.array(init_qpos)
    left_robot.set_qpos(init_qpos)
    right_robot.set_qpos(init_qpos)
    left_joint_names = [joint.name for joint in left_robot.get_active_joints()]
    right_joint_names = [joint.name for joint in right_robot.get_active_joints()]

    for robot in [left_robot, right_robot]:
        for joint in robot.get_active_joints():
            # These parameters are same as real robot, just for example purpose
            joint.set_drive_properties(10000, 500)

    # Load a table
    table = create_table(scene)

    # Create viewer and start simulate the robots
    viewer = Viewer()
    viewer.set_scene(scene)
    viewer.set_camera_pose(sapien.Pose([1.43346, 0.0135722, 0.772064], [0.00547725, 0.297903, 0.00167024, -0.954579]))

    # Teleoperation client
    port_num = 5500
    server_address = "localhost"
    teleop_client = TeleopClient(port=port_num, cmd_dims=(left_robot.dof, right_robot.dof), host=server_address)

    # Teleop initialization
    teleop_client.send_init_config(
        robot_base_pose=(sapien_pose_to_vector(left_robot_base_pose), sapien_pose_to_vector(right_robot_base_pose)),
        init_qpos=(init_qpos, init_qpos),
        joint_names=(left_joint_names, right_joint_names),
        align_gravity_dir=True,
        bimanual_alignment_mode=BimanualAlignmentMode.ALIGN_SEPARATELY,
    )
    print(f"Begin teleoperation initialization")
    print(f"Place your two hands under your Apple Vision Pro. Make sure you all your fingers are in flatten pose.")

    while not teleop_client.started:
        viewer.render()

    while not viewer.closed:
        # Gravity compensation
        left_robot.set_qf(left_robot.compute_passive_force())
        right_robot.set_qf(right_robot.compute_passive_force())

        # Get teleop command computed from server
        teleop_cmd = teleop_client.get_teleop_cmd()
        left_robot.set_qpos(teleop_cmd[0])
        right_robot.set_qpos(teleop_cmd[1])
        # scene.step()
        viewer.render()


def create_table(scene: sapien.Scene, table_height=1.0, table_half_size=(0.8, 0.8, 0.025)):
    builder = scene.create_actor_builder()

    # Top
    top_pose = sapien.Pose([0, 0, -table_half_size[2]])
    top_material = scene.create_physical_material(1, 0.5, 0.01)
    builder.add_box_collision(pose=top_pose, half_size=table_half_size, material=top_material)

    # Leg
    asset_dir = Path(__file__).parent.parent / "assets"
    table_map_path = asset_dir / "misc" / "table_map.jpg"
    table_cube_path = asset_dir / "misc" / "cube.obj"

    table_visual_material = sapien.render.RenderMaterial()
    texture = sapien.render.RenderTexture2D(str(table_map_path))
    table_visual_material.set_base_color_texture(texture)
    table_visual_material.set_metallic(0.0)
    table_visual_material.set_specular(0.3)
    table_visual_material.set_roughness(0.3)

    # Size
    leg_size = np.array([0.025, 0.025, (table_height / 2 - table_half_size[2])])
    leg_height = -table_height / 2 - table_half_size[2]
    x = table_half_size[0] - 0.1
    y = table_half_size[1] - 0.1
    builder.add_visual_from_file(
        str(table_cube_path), pose=top_pose, material=table_visual_material, scale=table_half_size, name="surface"
    )
    builder.add_box_visual(
        pose=sapien.Pose([x, y, leg_height]), half_size=leg_size, material=table_visual_material, name="leg0"
    )
    builder.add_box_visual(
        pose=sapien.Pose([x, -y, leg_height]), half_size=leg_size, material=table_visual_material, name="leg1"
    )
    builder.add_box_visual(
        pose=sapien.Pose([-x, y, leg_height]), half_size=leg_size, material=table_visual_material, name="leg2"
    )
    builder.add_box_visual(
        pose=sapien.Pose([-x, -y, leg_height]), half_size=leg_size, material=table_visual_material, name="leg3"
    )
    return builder.build_static("table")


def sapien_pose_to_vector(pose: sapien.Pose) -> np.ndarray:
    return np.concatenate([pose.p, pose.q])


if __name__ == '__main__':
    main()
