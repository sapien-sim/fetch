import sapien
import numpy as np
import warnings
import os


class Fetch(sapien.Widget):
    def load(self, scene: sapien.Scene):
        loader = scene.create_urdf_loader()
        loader.fix_root_link = False

        loader.set_link_material("base_link", 0.0, 0.0, 0.0)
        loader.set_link_material("l_wheel_link", 0.0, 0.0, 0.0)
        loader.set_link_material("r_wheel_link", 0.0, 0.0, 0.0)

        self.robot = loader.load(os.path.join(os.path.dirname(__file__), "fetch.urdf"))

        for l in self.robot.links:
            if l.name != "base_link":
                l.disable_gravity = True
            # else:
            #     l.mass = l.mass * 2
            #     l.inertia = l.inertia * 2

        self.torso_joint = self.robot.find_joint_by_name("torso_lift_joint")
        self.arm_joints = [
            self.robot.find_joint_by_name(j)
            for j in [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "upperarm_roll_joint",
                "elbow_flex_joint",
                "forearm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
            ]
        ]
        self.head_joints = [
            self.robot.find_joint_by_name(j)
            for j in ["head_pan_joint", "head_tilt_joint"]
        ]

        # dummy drive for wheels
        c = sapien.physx.PhysxRigidDynamicComponent()
        c.set_kinematic(True)
        self.drive_dummy = sapien.Entity().add_component(c).add_to_scene(scene)
        self.base_drive = scene.create_drive(
            self.drive_dummy,
            sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
            self.robot.root,
            sapien.Pose(q=[0.7071068, 0, -0.7071068, 0]),
        )
        self.set_base_drive_damping(1e3, 1e3)

    def set_base_drive_damping(self, linear, angular):
        self.base_drive.set_drive_property_z(0, linear, mode="acceleration")
        self.base_drive.set_drive_property_twist(0, angular, mode="acceleration")

    def set_base_drive(self, linear, angular):
        self.base_drive.set_drive_velocity_target([0, 0, -linear], [-angular, 0, 0])

    def step(self):
        self.drive_dummy.set_pose(self.robot.root.pose)

    def unload(self, scene: sapien.Scene):
        scene.remove_articulation(self.robot)


if __name__ == "__main__":
    robot = Fetch()

    scene = sapien.Scene()
    scene.load_widget(robot)
    scene.load_widget_from_package("demo_arena", "DemoArena")

    robot.set_base_drive(1, 0.5)

    viewer = scene.create_viewer()
    while not viewer.closed:
        robot.step()

        scene.step()
        scene.update_render()
        viewer.render()
