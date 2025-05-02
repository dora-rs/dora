"""TODO: Add docstring."""

import os
from typing import List, Optional, Tuple, Union

import numpy as np
import pyarrow as pa
import pytorch_kinematics as pk
import torch
from dora import Node
from pytorch_kinematics.transforms.rotation_conversions import matrix_to_euler_angles

TRANSFORM = np.array(os.getenv("TRANSFORM", "0. 0. 0. 0. 0. 0. 1.").split(" ")).astype(
    np.float32,
)
pos = torch.tensor([TRANSFORM[0], TRANSFORM[1], TRANSFORM[2]])
rot = torch.tensor(
    [
        TRANSFORM[3],
        TRANSFORM[4],
        TRANSFORM[5],
        TRANSFORM[6],
    ],
)
ROB_TF = pk.Transform3d(pos=pos, rot=rot)


def get_xyz_rpy_array_from_transform3d(
    transform: "pk.transforms.Transform3d", convention: str = "XYZ",
) -> torch.Tensor:
    """XYZ-RPY conversion.
    
    Extract translation (xyz) and rotation (RPY Euler angles in radians)
    from a pytorch_kinematics Transform3d object and returns them concatenated
    into a single tensor.

    Args:
        transform: A Transform3d object or a batch of Transform3d objects.
                    Expected to have a method like get_matrix() or directly
                    access attributes like .R and .T.
        convention: The Euler angle convention (e.g., "XYZ", "ZYX").
                    RPY typically corresponds to "XYZ" (roll around X,
                    pitch around Y, yaw around Z). Adjust if needed.

    Returns:
        A single tensor of shape (..., 6) where the last dimension contains
        [x, y, z, roll, pitch, yaw] in radians.

    """
    # Attempt to get the full 4x4 transformation matrix
    # Common methods are get_matrix() or accessing internal properties.
    # This might need adjustment based on the specific API version.
    matrix = transform.get_matrix()  # Shape (..., 4, 4)

    # Extract translation (first 3 elements of the last column)
    xyz = matrix[..., :3, 3]

    # Extract rotation matrix (top-left 3x3 submatrix)
    rotation_matrix = matrix[..., :3, :3]

    # Convert the rotation matrix to Euler angles in radians
    # The matrix_to_euler_angles function likely exists based on pytorch3d's structure
    rpy = matrix_to_euler_angles(
        rotation_matrix, convention=convention,
    )  # Shape (..., 3)

    # Concatenate xyz and rpy along the last dimension
    return torch.cat((xyz, rpy), dim=-1)  # Shape (..., 6)


class RobotKinematics:
    """wrapper for pytorch_kinematics FK and IK operations."""

    def __init__(
        self,
        urdf_path: str,
        end_effector_link: str,
        device: Union[str, torch.device] = "cpu",
    ):
        """Initialize the kinematic chain from a URDF.

        Args:
            urdf_path (str): Path to the URDF file.
            end_effector_link (str): Name of the end-effector link.
            device (Union[str, torch.device]): Computation device ('cpu' or 'cuda').

        """
        self.device = torch.device(device)
        try:
            # Load kinematic chain (core pytorch_kinematics object)
            self.chain = pk.build_serial_chain_from_urdf(
                open(urdf_path, mode="rb").read(), end_effector_link,
            ).to(device=self.device)
        except Exception as e:
            raise RuntimeError(f"Failed to build chain from URDF: {urdf_path}") from e

        self.num_joints = len(self.chain.get_joint_parameter_names(exclude_fixed=True))
        # Default initial guess for IK if none provided
        self._default_q = torch.zeros(
            (1, self.num_joints), device=self.device, dtype=torch.float32,
        )
        # print(f"Initialized RobotKinematicsConcise: {self.num_joints} joints, EE='{end_effector_link}', device='{self.device}'") # Optional print

    def _prepare_joint_tensor(
        self, joints: Union[List[float], torch.Tensor], batch_dim_required: bool = True,
    ) -> torch.Tensor:
        """Validate and formats joint angles input tensor."""
        if isinstance(joints, list):
            j = torch.tensor(joints, dtype=torch.float32, device=self.device)
        elif isinstance(joints, np.ndarray):
            j = torch.tensor(joints, device=self.device, dtype=torch.float32)
        elif isinstance(joints, torch.Tensor):
            j = joints.to(device=self.device, dtype=torch.float32)
        else:
            raise TypeError(
                "Joints must be a list or torch.Tensor and got: ", joints.type,
            )

        if j.ndim == 1:
            if j.shape[0] != self.num_joints:
                raise ValueError(f"Expected {self.num_joints} joints, got {j.shape[0]}")
            if batch_dim_required:
                j = j.unsqueeze(0)  # Add batch dimension if needed (e.g., shape (1, N))
        elif j.ndim == 2:
            if j.shape[1] != self.num_joints:
                raise ValueError(
                    f"Expected {self.num_joints} joints (dim 1), got {j.shape[1]}",
                )
            if batch_dim_required and j.shape[0] != 1:
                # Most common IK solvers expect batch=1 for initial guess, FK can handle batches
                # Relaxing this check slightly, but user should be aware for IK
                pass
        else:
            raise ValueError(f"Joint tensor must have 1 or 2 dimensions, got {j.ndim}")
        return j

    def compute_fk(
        self, joint_angles: Union[List[float], torch.Tensor],
    ) -> pk.Transform3d:
        """Compute Forward Kinematics (FK).

        Args:
            joint_angles (Union[List[float], torch.Tensor]): Joint angles (radians).
                Shape (num_joints,) or (B, num_joints).

        Returns:
            pk.Transform3d: End-effector pose(s). Batched if input is batched.

        """
        # robot frame

        angles_tensor = self._prepare_joint_tensor(
            joint_angles, batch_dim_required=False,
        )  # FK handles batches naturally
        # Direct call to pytorch_kinematics FK
        goal_in_rob_frame_tf = self.chain.forward_kinematics(
            angles_tensor, end_only=True,
        )
        goal_tf = ROB_TF.compose(goal_in_rob_frame_tf)
        return goal_tf

    def compute_ik(
        self,
        target_pose: pk.Transform3d,
        initial_guess: Optional[Union[List[float], torch.Tensor]] = None,
        iterations: int = 100,
        lr: float = 0.1,
        error_tolerance: float = 1e-4,
    ) -> Tuple[torch.Tensor, bool]:
        """Compute Inverse Kinematics (IK) using PseudoInverseIK.

        Args:
            target_pose (pk.Transform3d): Target end-effector pose (batch size 1).
            initial_guess (Optional): Initial joint angles guess. Uses zeros if None.
            Shape (num_joints,) or (1, num_joints).
            iterations (int): Max solver iterations.
            lr (float): Solver learning rate.
            error_tolerance (float): Solver convergence tolerance.

        Returns:
            Tuple[torch.Tensor, bool]: Solution joint angles (1, num_joints), convergence status.

        """
        if not isinstance(target_pose, pk.Transform3d):
            raise TypeError("target_pose must be a pk.Transform3d")
        target_pose = target_pose.to(device=self.device)
        if target_pose.get_matrix().shape[0] != 1:
            raise ValueError(
                "target_pose must have batch size 1 for this IK method.",
            )  # Common limitation

        # Prepare initial guess (q_init)
        q_init = self._prepare_joint_tensor(
            initial_guess if initial_guess is not None else self._default_q,
            batch_dim_required=True,
        )
        if q_init.shape[0] != 1:
            raise ValueError(
                "initial_guess must result in batch size 1 for this IK setup.",
            )  # Enforce batch=1 for guess

        q_init = q_init.requires_grad_(True)  # Need gradient for solver

        # Instantiate and run the IK solver (core pytorch_kinematics objects/methods)
        ik_solver = pk.PseudoInverseIK(
            self.chain,
            max_iterations=1_000,
            retry_configs=q_init,
            joint_limits=torch.tensor(self.chain.get_joint_limits()),
            early_stopping_any_converged=True,
            early_stopping_no_improvement="all",
            debug=False,
            lr=0.05,
            rot_tolerance=1e-4,
            pos_tolerance=1e-3,
        )
        solution_angles = ik_solver.solve(target_pose)
        if solution_angles.err_rot > 1e-3 or solution_angles.err_pos > 1e-2:
            print(
                f"IK did not converge: pos_err={solution_angles.err_pos}, rot_err={solution_angles.err_rot} for target {target_pose}",
            )
            return None
        return solution_angles.solutions.detach()


def main():
    """TODO: Add docstring."""
    node = Node()

    urdf_path = os.getenv("URDF_PATH")
    end_effector_link = os.getenv("END_EFFECTOR_LINK")
    robot = RobotKinematics(urdf_path, end_effector_link)
    last_known_state = None

    for event in node:
        if event["type"] == "INPUT":
            metadata = event["metadata"]

            match metadata["encoding"]:
                case "xyzquat":
                    # Apply Inverse Kinematics
                    if last_known_state is not None:
                        target = event["value"].to_numpy()
                        target = target.astype(np.float32)
                        target = pk.Transform3d(
                            pos=target[:3], rot=torch.tensor(target[3:7]),
                        )
                        solution = robot.compute_ik(target, last_known_state)
                        metadata["encoding"] = "jointstate"
                        last_known_state = solution.numpy().ravel()
                        solution = pa.array(last_known_state)
                        node.send_output(event["id"], solution, metadata=metadata)
                case "xyzrpy":
                    # Apply Inverse Kinematics
                    if last_known_state is not None:
                        target = event["value"].to_numpy()
                        target = target.astype(np.float32)
                        target = pk.Transform3d(
                            pos=target[:3],
                            rot=pk.transforms.euler_angles_to_matrix(
                                torch.tensor(target[3:6]), convention="XYZ",
                            ),
                        )
                        rob_target = ROB_TF.inverse().compose(target)
                        solution = robot.compute_ik(rob_target, last_known_state)
                        if solution is None:
                            print(
                                "No IK Solution for :", target, "skipping this frame.",
                            )
                            continue

                        solution = solution.numpy().ravel()
                        delta_angles = solution - last_known_state

                        valid = np.all(
                            (delta_angles >= -np.pi) & (delta_angles <= np.pi),
                        )
                        if not valid:
                            print(
                                "IK solution is not valid, as the rotation are too wide. skipping.",
                            )
                            continue
                        metadata["encoding"] = "jointstate"
                        last_known_state = solution
                        solution = pa.array(last_known_state)
                        node.send_output(event["id"], solution, metadata=metadata)
                case "jointstate":
                    target = event["value"].to_numpy()
                    last_known_state = target
                    # Apply Forward Kinematics
                    target = robot.compute_fk(target)
                    target = np.array(get_xyz_rpy_array_from_transform3d(target))
                    target = pa.array(target.ravel(), type=pa.float32())
                    metadata["encoding"] = "xyzrpy"
                    node.send_output(event["id"], target, metadata=metadata)


if __name__ == "__main__":
    main()
