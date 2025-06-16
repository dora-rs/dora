"""TODO: Add docstring."""

import os
from typing import List, Optional, Tuple, Union

import numpy as np
import pyarrow as pa
import pytorch_kinematics as pk
import torch
from dora import Node
from pytorch_kinematics.transforms.rotation_conversions import matrix_to_euler_angles
from pathlib import Path
import importlib

TRANSFORM = np.array(os.getenv("TRANSFORM", "0. 0. 0. 1. 0. 0. 0.").split(" ")).astype(
    np.float32,
) # wxyz format
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
        urdf: str,
        end_effector_link: str,
        device: Union[str, torch.device] = "cpu",
    ):
        """Initialize the kinematic chain from a URDF.

        Args:
            urdf (str): URDF string data of the URDF
            urdf_path (str): Path to the URDF file.
            end_effector_link (str): Name of the end-effector link.
            device (Union[str, torch.device]): Computation device ('cpu' or 'cuda').

        """
        self.device = torch.device(device)
        if urdf_path:
            urdf = open(urdf_path, mode="rb").read()
        # Load kinematic chain (core pytorch_kinematics object)
        self.chain = pk.build_serial_chain_from_urdf(
            urdf, end_effector_link,
        ).to(device=self.device)

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
            # Handle case with extra joints (e.g., gripper joints)
            if j.shape[0] > self.num_joints:
                j = j[:self.num_joints]  # Truncate griper or extra joints
            elif j.shape[0] < self.num_joints:
                raise ValueError(f"Expected at least {self.num_joints} joints, got {j.shape[0]}")
            
            if batch_dim_required:
                j = j.unsqueeze(0)  # Add batch dimension if needed
        elif j.ndim == 2:
            # Handle batched input with extra joints
            if j.shape[1] > self.num_joints:
                j = j[:, :self.num_joints]  # Truncate griper or extra joints
            elif j.shape[1] < self.num_joints:
                raise ValueError(f"Expected at least {self.num_joints} joints (dim 1), got {j.shape[1]}")
            
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

    def compute_jacobian(
        self, joint_angles: Union[List[float], torch.Tensor]
    ) -> torch.Tensor:
        """Compute Jacobian matrix using PyTorch Kinematics.
        
        Args:
            joint_angles (Union[List[float], torch.Tensor]): Joint angles (radians).
                Shape (num_joints,) or (B, num_joints).
                
        Returns:
            torch.Tensor: Jacobian matrix of shape (B, 6, num_joints) or (6, num_joints)
                         where rows are [linear_vel_x, linear_vel_y, linear_vel_z, 
                                       angular_vel_x, angular_vel_y, angular_vel_z]

        """
        angles_tensor = self._prepare_joint_tensor(
            joint_angles, batch_dim_required=False
        )
        
        # Ensure we have batch dimension for jacobian computation
        if angles_tensor.ndim == 1:
            angles_tensor = angles_tensor.unsqueeze(0)
            squeeze_output = True
        else:
            squeeze_output = False
            
        # Compute Jacobian (returns shape: (B, 6, num_joints))
        J = self.chain.jacobian(angles_tensor)
        
        # Remove batch dimension if input was 1D
        if squeeze_output:
            J = J.squeeze(0)
            
        return J

    def compute_jacobian_numpy(
        self, joint_angles: Union[List[float], torch.Tensor, np.ndarray]
    ) -> np.ndarray:
        """Compute Jacobian matrix and return as numpy array.
        
        Args:
            joint_angles: Joint angles (radians). Can be list, torch.Tensor, or np.ndarray.
                         Shape (num_joints,) or (B, num_joints).
                         
        Returns:
            np.ndarray: Jacobian matrix as numpy array of shape (6, num_joints) or (B, 6, num_joints)

        """
        J = self.compute_jacobian(joint_angles)
        return J.cpu().numpy()


def load_robot_description_with_cache_substitution(robot_name: str) -> str:
    """Load a robot's URDF or MJCF file and replace package:// URIs with cache paths.

    Args:
        robot_name: str (e.g., "iiwa7_description"). The robot name handler

    Returns:
    -------
    - str: File content with all package:// URIs replaced

    """
    try:
        # Dynamically import the robot description module
        desc_module = importlib.import_module(f"robot_descriptions.{robot_name}")

        # Find the URDF or MJCF path
        if hasattr(desc_module, "URDF_PATH"):
            file_path = Path(desc_module.URDF_PATH)
        elif hasattr(desc_module, "MJCF_PATH"):
            file_path = Path(desc_module.MJCF_PATH)
        else:
            raise ValueError(f"No URDF or MJCF path found for '{robot_name}'.")
        print(f"Loading robot description from: {file_path}")

        # Read and replace
        with open(file_path) as f:
            content = f.read()

        print("URDF PATH: ", file_path)
        content = content.encode("utf-8")
        return content

    except ModuleNotFoundError:
        raise ValueError(f"Robot '{robot_name}' not found.")
    except Exception as e:
        raise RuntimeError(f"Failed to process robot description: {e}")


def main():
    """TODO: Add docstring."""
    node = Node()

    model = os.getenv("URDF_PATH")
    end_effector_link = os.getenv("END_EFFECTOR_LINK")

    if not model or not Path(model).exists():
        model_name = os.getenv("MODEL_NAME")
        model = load_robot_description_with_cache_substitution(model_name)
        robot = RobotKinematics(urdf_path="", urdf=model, end_effector_link=end_effector_link)
    else:
        robot = RobotKinematics(urdf_path=model, urdf="", end_effector_link=end_effector_link)
    last_known_state = None

    for event in node:
        if event["type"] == "INPUT":
            metadata = event["metadata"]

            if event["id"] == "cmd_vel":
                if last_known_state is not None:
                    target_vel = event["value"].to_numpy() # expect 100ms
                    # Apply Forward Kinematics
                    target = robot.compute_fk(last_known_state) 
                    target = np.array(get_xyz_rpy_array_from_transform3d(target)) + target_vel
                    target = pa.array(target.ravel(), type=pa.float32())
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
                    metadata["encoding"] = "jointstate"
                    last_known_state = solution
                    solution = pa.array(last_known_state)
                    node.send_output(event["id"], solution, metadata=metadata)
            else:
                match metadata["encoding"]:
                    case "xyzquat":
                        # Apply Inverse Kinematics
                        if last_known_state is not None:
                            target = event["value"].to_numpy()
                            target = target.astype(np.float32)
                            target = pk.Transform3d(
                                pos=target[:3], rot=torch.tensor(target[3:7]),
                            )
                            rob_target = ROB_TF.inverse().compose(target)
                            solution = robot.compute_ik(rob_target, last_known_state)
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
                            delta_angles = solution - last_known_state[:len(solution)] # match with dof

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
                    case "jacobian":
                        # Compute Jacobian matrix
                        joint_angles = event["value"].to_numpy()
                        jacobian = robot.compute_jacobian_numpy(joint_angles)
                        
                        jacobian_flat = jacobian.flatten()
                        jacobian_array = pa.array(jacobian_flat, type=pa.float32())
                        
                        metadata["encoding"] = "jacobian_result"
                        metadata["jacobian_shape"] = list(jacobian.shape)
                        
                        node.send_output(event["id"], jacobian_array, metadata=metadata)
                

if __name__ == "__main__":
    main()
