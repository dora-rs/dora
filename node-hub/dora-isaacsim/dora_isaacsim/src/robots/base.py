import os

import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.robots import Robot


class BaseRobot:
    def __init__(
        self,
        robot_prim_path,
        name="franka",
        joints_name=[],
        ee_prim_path=None,
        gripper_joint_name=None,
        controller=None,
        control_mode="joint",
        kinematics_solver_cfg=dict(),
    ):
        self.robot_prim_path = robot_prim_path
        self.robot = None
        self.name = name
        self.joints_name = joints_name
        self.joint_indices = []
        self.ee_prim_path = ee_prim_path  # usd 中 ee 的 prim 路径

        self.init_ee_pose = None  # 初始时刻的 ee 位姿
        self.init_joint_pos = None  # 初始时刻的关节角f度

        self.gripper_joint_name = gripper_joint_name

        self.controller = controller
        self.control_mode = control_mode

        self.kinematics_solver_cfg = kinematics_solver_cfg

    def spawn(self, world):
        self.robot = world.scene.add(Robot(prim_path=self.robot_prim_path, name=self.name))

        if self.controller is not None:
            self.controller.spawn(self)

        if self.kinematics_solver_cfg:
            self.kinematics_solver, self.articulation_kinematics_solver = self.init_IK_controller(
                self.robot,
                self.kinematics_solver_cfg["end_effector_name"],
                self.kinematics_solver_cfg["kinematics_config_dir"],
                self.kinematics_solver_cfg["robot_description_path"],
                self.kinematics_solver_cfg["urdf_path"],
            )

    def init_IK_controller(
        self,
        articulation,
        end_effector_name,
        kinematics_config_dir,
        robot_description_path,
        urdf_path,
    ):
        # 获取 IK Controller
        kinematics_solver = LulaKinematicsSolver(
            robot_description_path=os.path.join(kinematics_config_dir, robot_description_path),
            urdf_path=os.path.join(kinematics_config_dir, urdf_path),
        )

        articulation_kinematics_solver = ArticulationKinematicsSolver(
            articulation, kinematics_solver, end_effector_name
        )
        return kinematics_solver, articulation_kinematics_solver

    def compute_IK(
        self,
        target_position,
        target_orientation=None,
        position_tolerance=None,
        orientation_tolerance=None,
        frame="world",
    ):
        """
        Compute inverse kinematics for the end effector frame using the current robot position as a warm start.  The result is returned
        in an articulation action that can be directly applied to the robot.

        Args:
            target_position (np.array): target translation of the target frame (in stage units) relative to the USD stage origin
            target_orientation (np.array): target orientation of the target frame relative to the USD stage global frame. Defaults to None.
            position_tolerance (float): l-2 norm of acceptable position error (in stage units) between the target and achieved translations. Defaults to None.
            orientation tolerance (float): magnitude of rotation (in radians) separating the target orientation from the achieved orientation.
                orientation_tolerance is well defined for values between 0 and pi. Defaults to None.

        Returns:
            Tuple[ArticulationAction, bool]:
            ik_result: An ArticulationAction that can be applied to the robot to move the end effector frame to the desired position.
            success: Solver converged successfully
        """
        if frame == "world":
            # set the robot base pose respect to the world frame
            robot_base_translation, robot_base_orientation = self.robot.get_world_pose()
            self.kinematics_solver.set_robot_base_pose(
                robot_base_translation, robot_base_orientation
            )

        action, success = self.articulation_kinematics_solver.compute_inverse_kinematics(
            target_position, target_orientation, position_tolerance, orientation_tolerance
        )

        if not success:
            print("IK did not converge to a solution.  No action is being taken")
        return action.joint_positions, success

    def reset(self):
        self.gripper_index = self.robot.get_dof_index(self.gripper_joint_name)
        if len(self.joints_name) > 0:
            self.joint_indices = [
                self.robot.get_dof_index(joint_name) for joint_name in self.joints_name
            ]

        if self.controller is not None:
            self.controller.reset()

    def apply_action(self, joint_positions, joint_indices=None):
        if joint_indices is None:
            joint_indices = self.joint_indices
        target_joint_action = ArticulationAction(
            joint_positions=joint_positions, joint_indices=joint_indices
        )
        articulation_controller = self.robot.get_articulation_controller()
        articulation_controller.apply_action(target_joint_action)

    def apply_gripper_width(self, gripper_width):
        gripper_index = self.gripper_index
        self.apply_action(joint_positions=[gripper_width], joint_indices=[gripper_index])

    def get_joint_position(self, joint_indices=None):
        if joint_indices is None:
            joint_indices = self.joint_indices
        joint_pos = [self.robot.get_joint_positions(joint_idx) for joint_idx in joint_indices]
        return np.column_stack(joint_pos)[0]

    def get_ee_pose(self):
        # 获取 panda_hand 的 Prim
        end_effector_path = self.ee_prim_path
        try:
            hand_prim = XFormPrim(end_effector_path)
            # 获取全局坐标系下的位姿
            state = hand_prim.get_default_state()
            return np.concatenate([state.position, state.orientation], axis=-1)
        except Exception:
            hand_prim = RigidPrim(end_effector_path)
            # 获取全局坐标系下的位姿
            return hand_prim.get_world_pose()

    def get_gripper_width(self):
        gripper_width = self.robot.get_joint_positions(self.gripper_index)
        return gripper_width

    def get_states(self):
        joint_pos = self.get_joint_position()
        ee_pose = self.get_ee_pose()
        gripper_width = self.get_gripper_width()

        states = {
            "joint_pos": joint_pos,
            "ee_pose": ee_pose,
            "gripper_width": gripper_width,
        }

        return states

    def compute_action(self):
        action = self.controller.forward()
        if self.control_mode == "end_effector_control":
            if action is not None:
                next_action, success = self.compute_IK(
                    target_position=action[:3], target_orientation=action[3:7]
                )
                return np.append(next_action, action[-1])
        return action

    def set_trajectory(self, trajectory):
        self.controller.set_trajectory(trajectory)

    def get_trajectory(self):
        return self.controller.get_trajectory()
