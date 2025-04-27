import os
from typing import Dict, Literal, Tuple

import mediapipe as mp
import numpy as np
import pyarrow as pa
from dora import Node
from scipy.optimize import minimize

REACHY_R_SHOULDER_COORDINATES = np.array([0, -0.19, 0])
REACHY_L_SHOULDER_COORDINATES = np.array([0, 0.19, 0])
ELBOW_WEIGHT = 0.1
MAX_CHANGE = 5.0
MAX_ITER = 20
SMOOTHING_BUFFER_SIZE = 10
POSITION_ALPHA = 0.3
MAX_CHANGE = 5.0
TARGET_POS_TOLERANCE = 0.03
MOVEMENT_MIN_TOLERANCE = 0.005
HAND_SCALE_FACTOR = 0.94
ELBOW_SCALE_FACTOR = 0.93

mp_pose = mp.solutions.pose
initial_pose = [
    -0.11903145498601328,
    0.11292280260403312,
    0.48048914307403895,
    -1.4491468779308918,
    0.1895427567665842,
    0.009599310885968814,
    -0.20141099568014562,
    2.2656896114349365,
    -0.13212142437597074,
    -0.07731808586334879,
    -0.5141739976375295,
    -1.512502329778286,
    0.00034906585039886593,
    0.3193952531149623,
    0.40474185353748504,
    2.2610876560211,
]


def mattransfo(alpha, d, theta, r):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array(
        [
            [ct, -st, 0, d],
            [ca * st, ca * ct, -sa, -r * sa],
            [sa * st, sa * ct, ca, r * ca],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def compute_transformation_matrices(joint_angles, length, side):
    """
    Compute the transformation matrices for the robotic arm.
    """
    pi = np.pi
    alpha = [0, -pi / 2, -pi / 2, -pi / 2, +pi / 2, -pi / 2, -pi / 2, -pi / 2]
    r = np.array([0, 0, -length[0], 0, -length[1], 0, 0, -length[2]])
    th = [
        joint_angles[0],
        joint_angles[1] - pi / 2,
        joint_angles[2] - pi / 2,
        joint_angles[3],
        joint_angles[4],
        joint_angles[5] - pi / 2,
        joint_angles[6] - pi / 2,
        -pi / 2,
    ]

    if side == "right":
        d = [0, 0, 0, 0, 0, 0, -0.325, -0.01]
        Tbase0 = mattransfo(-pi / 2, 0, -pi / 2, -0.19)
    if side == "left":
        d = [0, 0, 0, 0, 0, 0, -0.325, 0.01]
        Tbase0 = mattransfo(-pi / 2, 0, -pi / 2, 0.19)

    T01 = Tbase0 @ mattransfo(alpha[0], d[0], th[0], r[0])
    T12 = mattransfo(alpha[1], d[1], th[1], r[1])
    T23 = mattransfo(alpha[2], d[2], th[2], r[2])
    T34 = mattransfo(alpha[3], d[3], th[3], r[3])
    T45 = mattransfo(alpha[4], d[4], th[4], r[4])
    T56 = mattransfo(alpha[5], d[5], th[5], r[5])
    T67 = mattransfo(alpha[6], d[6], th[6], r[6])
    T78 = mattransfo(alpha[7], d[7], th[7], r[7])

    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45
    T06 = T05 @ T56
    T07 = T06 @ T67
    T08 = T07 @ T78
    return T04, T08


def forward_kinematics(joint_angles, length=[0.28, 0.25, 0.075], side="right"):
    """
    Calculate the hand-effector position using forward kinematics.
    """
    T04, T08 = compute_transformation_matrices(joint_angles, length, side)
    position_e = np.array(T04[0:3, 3], dtype=np.float64).flatten()
    position = np.array(T08[0:3, 3], dtype=np.float64).flatten()
    return position_e, position


def cost_function(
    joint_angles, target_ee_coords, target_elbow_coords, length, side
) -> float:
    """
    Compute the cost function that includes end-effector and elbow position errors.
    """
    # Compute the end-effector and elbow coordinates + errors
    elbow_coords, ee_coords = forward_kinematics(joint_angles, length, side)
    ee_error = np.linalg.norm(target_ee_coords - ee_coords)
    elbow_error = np.linalg.norm(target_elbow_coords - elbow_coords)

    # Compute the total cost
    total_cost = ee_error + ELBOW_WEIGHT * elbow_error
    return total_cost


def inverse_kinematics_fixed_wrist(
    hand_position,
    elbow_position,
    initial_guess,
    length=[0.28, 0.25, 0.075],
    side="right",
):
    """
    Implement the inverse kinematics with a fixed wrist.
    Input initial_guess is in degrees. Output is in degrees.
    """
    pi = np.pi
    # Convert initial guess from degrees to radians
    initial_guess_rad = np.deg2rad(initial_guess)

    joint_limits = [
        (-1.0 * pi, 0.5 * pi),
        (-1.0 * pi, 10 / 180 * pi),
        (-0.5 * pi, 0.5 * pi),
        (-125 / 180 * pi, 0),
        (0, 0),
        (0, 0),
        (0, 0),
    ]

    result = minimize(
        cost_function,
        initial_guess_rad,  # Use radian value for optimization
        args=(hand_position, elbow_position, length, side),
        # method="SLSQP",
        method="L-BFGS-B",
        bounds=joint_limits,
        # NOTE: these are new
        tol=1e-3,  # Higher tolerance = faster but less accurate
        options={"maxiter": 20},
    )

    # Convert result from radians to degrees
    return np.rad2deg(result.x)


# Helper function to convert a landmark to 3D coordinates from the camera's perspective using the human coordinate system
def get_3D_coordinates(landmark, depth_frame, w, h, intrinsics):
    """Convert a landmark to 3D coordinates using depth information.

    Note that this function converts the camera frame to the human frame whereas the origin remains the same.

    Transforms from camera coordinates to robot coordinates:
    Human x = -Camera depth
    Human y = -Camera x
    Human z = -Camera y

    Args:
        landmark: Either a landmark object with x, y, z attributes or
                 a numpy array/list with [x, y, z] coordinates (normalized)
        depth_frame: The depth frame from the camera
        w: Image width
        h: Image height
        intrinsics: Camera intrinsic parameters
    """
    # Handle both landmark objects and numpy arrays/lists
    if hasattr(landmark, "x") and hasattr(landmark, "y"):
        cx, cy = int(landmark.x * w), int(landmark.y * h)
    else:
        # Assume it's a numpy array or list with [x, y, z]
        cx, cy = int(landmark[0] * w), int(landmark[1] * h)

    # Check if pixel coordinates are within image bounds
    if 0 <= cx < w and 0 <= cy < h:
        depth = depth_frame.get_distance(cx, cy)
        if depth > 0:  # Ensure depth is valid
            # Get camera intrinsic parameters
            fx, fy = intrinsics.fx, intrinsics.fy
            ppx, ppy = intrinsics.ppx, intrinsics.ppy

            # Convert to camera 3D coordinates
            x = (cx - ppx) * depth / fx
            y = (cy - ppy) * depth / fy

            # Transform to robot coordinate system
            # TODO: based on the camera's system, it should really be -z, -x, -y
            return np.array([-depth, x, -y])

    # Default return if coordinates are invalid
    return np.array([0, 0, 0])


def get_3D_coordinates_of_hand(
    index_landmark, pinky_landmark, depth_frame, w, h, intrinsics
):
    # Calculate average coordinates
    x_pixels = (index_landmark.x + pinky_landmark.x) / 2
    y_pixels = (index_landmark.y + pinky_landmark.y) / 2
    z_zoom_factor = (index_landmark.z + pinky_landmark.z) / 2

    # Create a numpy array with the averaged values
    avg_coords = np.array([x_pixels, y_pixels, z_zoom_factor])

    # Call get_3D_coordinates with all required parameters
    return get_3D_coordinates(avg_coords, depth_frame, w, h, intrinsics)


def get_landmark_indices(side: Literal["right", "left"], mp_pose):
    """Return MediaPipe landmark indices for the specified arm side

    Args:
        side: Either "right" or "left"
        mp_pose: MediaPipe pose module

    Returns:
        Dictionary mapping landmark names to MediaPipe indices
    """
    if side == "right":
        return {
            "shoulder": mp_pose.PoseLandmark.RIGHT_SHOULDER,
            "elbow": mp_pose.PoseLandmark.RIGHT_ELBOW,
            "index": mp_pose.PoseLandmark.RIGHT_INDEX,
            "pinky": mp_pose.PoseLandmark.RIGHT_PINKY,
        }
    else:
        return {
            "shoulder": mp_pose.PoseLandmark.LEFT_SHOULDER,
            "elbow": mp_pose.PoseLandmark.LEFT_ELBOW,
            "index": mp_pose.PoseLandmark.LEFT_INDEX,
            "pinky": mp_pose.PoseLandmark.LEFT_PINKY,
        }


def get_arm_coordinates(
    landmarks_data, landmark_indices, depth_frame, w, h, intrinsics
):
    """Get 3D coordinates for the arm

    Args:
        landmarks_data: MediaPipe pose landmarks
        landmark_indices: Dictionary mapping landmark names to MediaPipe indices
        depth_frame: RealSense depth frame
        w: Image width
        h: Image height
        intrinsics: Camera intrinsic parameters

    Returns:
        Tuple of (shoulder, elbow, hand) 3D coordinates
    """
    shoulder = get_3D_coordinates(
        landmarks_data[landmark_indices["shoulder"]],
        depth_frame,
        w,
        h,
        intrinsics,
    )
    elbow = get_3D_coordinates(
        landmarks_data[landmark_indices["elbow"]],
        depth_frame,
        w,
        h,
        intrinsics,
    )
    hand = get_3D_coordinates_of_hand(
        landmarks_data[landmark_indices["index"]],
        landmarks_data[landmark_indices["pinky"]],
        depth_frame,
        w,
        h,
        intrinsics,
    )
    return shoulder, elbow, hand


def process_new_position(
    target_ee_coords: np.ndarray,
    current_ee_coords: np.ndarray,
    position_history: list | None,
) -> Tuple[bool, np.ndarray, np.ndarray]:
    """Process a new target end effector position and determine if movement is needed

    Args:
        target_ee_coord: The target hand position in Reachy's coordinate system
        prev_hand_pos: Previous hand position
        position_history: List of previous positions for smoothing
        current_ee_pos: Current end effector position

    Returns:
        Tuple containing:
            bool: True if the position has changed enough to require an update
            np.ndarray: The smoothed target position
    """
    # ! We're not actually using the position history for smoothing here
    # if position_history is None:
    #     position_history = []

    # # Apply position smoothing
    # position_history.append(target_ee_coords)
    # if len(position_history) > SMOOTHING_BUFFER_SIZE:
    #     position_history.pop(0)

    # Compute EMA for smoother position
    smoothed_position = (
        POSITION_ALPHA * target_ee_coords + (1 - POSITION_ALPHA) * current_ee_coords
    )

    # Check if the desired position is different from current position
    already_at_position = np.allclose(
        current_ee_coords, smoothed_position, atol=TARGET_POS_TOLERANCE
    )

    # Check if the position has changed significantly from the previous position
    should_update_position = (
        not np.allclose(
            current_ee_coords, smoothed_position, atol=MOVEMENT_MIN_TOLERANCE
        )
        and not already_at_position
    )

    return should_update_position, smoothed_position


def calculate_joint_positions_custom_ik(
    target_ee_position: np.ndarray,
    target_elbow_position: np.ndarray,
    joint_array: np.ndarray,
    side: Literal["right", "left"],
) -> Tuple[bool, Dict[str, float]]:
    """Calculate new joint positions using inverse kinematics

    Args:
        target_ee_position: The target hand position
        target_elbow_position: The target elbow position
        joint_array: Current joint positions
        side: Arm side (right or left)

    Returns:
        Tuple containing:
            bool: True if calculation was successful
            Dict[str, float]: Updated joint positions dictionary
    """
    prefix = f"{side[0]}_"  # "r_" or "l_"
    joint_dict = {}

    try:
        # Compute IK
        joint_pos = inverse_kinematics_fixed_wrist(
            hand_position=target_ee_position,
            elbow_position=target_elbow_position,
            initial_guess=joint_array,
            length=[0.28, 0.25, 0.075],
            side=side,
        )

        # Get joint names for this arm
        joint_names = [
            f"{prefix}shoulder_pitch",
            f"{prefix}shoulder_roll",
            f"{prefix}arm_yaw",
            f"{prefix}elbow_pitch",
            f"{prefix}forearm_yaw",
            f"{prefix}wrist_pitch",
            f"{prefix}wrist_roll",
        ]

        # Apply rate limiting and update joint dictionary
        updated_joint_array = joint_array.copy()
        for i, (name, value) in enumerate(zip(joint_names, joint_pos)):
            # Apply rate limiting
            limited_change = np.clip(value - joint_array[i], -MAX_CHANGE, MAX_CHANGE)
            updated_joint_array[i] += limited_change
            joint_dict[name] = updated_joint_array[i]

        # Handle gripper separately - maintain closed
        joint_dict[f"{prefix}gripper"] = 0

        return True, joint_dict
    except Exception as e:
        print(f"{side.capitalize()} arm IK calculation error: {e}")
        return False, {}


def get_reachy_coordinates(point, shoulder, sf, side="right"):
    """

    Args:
        point: The point in camera-relative coordinates and human frame
        shoulder: The shoulder (on same side as the point) in camera-relative coordinates and human frame
        sf: The scaling factor
        arm: Either "right" or "left" to specify which shoulder

    Returns:
        The point, now relative to Reachy's origin and scaled
    """
    if side.lower() == "left":
        return (point - shoulder) * sf + REACHY_L_SHOULDER_COORDINATES
    else:
        return (point - shoulder) * sf + REACHY_R_SHOULDER_COORDINATES


def entry_point(
    human_shoulder_coords: np.ndarray,
    human_elbow_coords: np.ndarray,
    human_hand_coords: np.ndarray,
    current_joint_array: np.ndarray,
    side: Literal["right", "left"],
    position_history: list | None = None,
) -> Tuple[bool, Dict[str, float]]:
    """
    Process 3D pose data to generate joint positions for a robotic arm.

    Parameters:
        human_shoulder_coords: The shoulder coordinates in depth camera frame
        human_elbow_coords: The elbow coordinates in depth camera frame
        human_hand_coords: The hand coordinates in depth camera frame
        current_joint_array: Current joint angles (degrees)
        position_history: List of recent positions for smoothing

    Returns:
        Tuple containing:
            - bool: True if the arm was updated, False otherwise
            - Dict[str, float]: Dictionary mapping joint names to angle values in degrees (e.g., 'r_shoulder_pitch': 45.0)

    This function extracts 3D coordinates for the shoulder, elbow, and hand from camera data,
    converts them to the robot's coordinate system, and calculates inverse kinematics
    to determine joint angles needed to position the arm.
    """
    if (
        human_shoulder_coords is None
        or human_hand_coords is None
        or human_elbow_coords is None
    ):
        return False, {}

    # Convert to Reachy coordinate frame
    target_ee_coords = get_reachy_coordinates(
        human_hand_coords, human_shoulder_coords, 1.0, side
    )
    target_elbow_coords = get_reachy_coordinates(
        human_elbow_coords, human_shoulder_coords, 1.0, side
    )

    # Calculate current end effector position using forward kinematics
    current_elbow_coords, current_ee_coords = forward_kinematics(
        current_joint_array, length=[0.28, 0.25, 0.075], side=side
    )

    # Process the new position and determine if update is needed
    should_update, target_ee_coords_smoothed = process_new_position(
        target_ee_coords,
        current_ee_coords,
        position_history,
    )

    if should_update:
        # ! We're not smoothing the elbow position here
        # Calculate IK and update joint positions
        successful_update, joint_dict = calculate_joint_positions_custom_ik(
            target_ee_coords_smoothed,
            target_elbow_coords,
            current_joint_array,
            side,
        )
        return successful_update, joint_dict

    return False, {}


if __name__ == "__main__":
    node = Node()
    current_joint_array = np.array([0, 0, 0, 0, 0, 0, 0])
    position_history = None
    side = os.getenv("SIDE", "right").lower()

    ids = get_landmark_indices(side, mp_pose)
    current_joint_array = initial_pose[:7]
    for event in node:
        if event["type"] != "INPUT":
            continue

        # if event["id"] == "robot_pose":
        # current_joint_array = event["value"].to_pylist()
        elif event["id"] == "human_pose":
            # Process human pose data
            values = event["value"].to_numpy().reshape(-1, 3)
            shoulder = values[ids["shoulder"]]
            elbow = values[ids["elbow"]]
            index = values[ids["index"]]
            pinky = values[ids["pinky"]]
            hand = (index + pinky) / 2.0
            if hand[0] == 0 and hand[1] == 0 and hand[2] == 0:
                continue
            elif shoulder[0] == 0 and shoulder[1] == 0 and shoulder[2] == 0:
                continue
            elif elbow[0] == 0 and elbow[1] == 0 and elbow[2] == 0:
                continue

            success, joint_dict = entry_point(
                shoulder,
                elbow,
                hand,
                current_joint_array,
                side,
                position_history,
            )

            if success:
                current_joint_array = np.array(list(joint_dict.values())).ravel()[:7]
                # Send joint_dict to the robot
                current_joint_array = np.deg2rad(current_joint_array)
                node.send_output(
                    output_id="pose",
                    data=pa.array(current_joint_array),
                    metadata={},
                )
            else:
                print("Failed to update arm positions.")
