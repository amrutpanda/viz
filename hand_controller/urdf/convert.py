from urdf2mjcf import run

run(
    urdf_path="hand_controller.urdf",
    mjcf_path="hand_controller.mjcf",
    copy_meshes=True,
)
