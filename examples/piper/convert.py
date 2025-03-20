"""TODO: Add docstring."""

import numpy as np
from scipy.spatial.transform import Rotation


def convert_quaternion_to_euler(quat):
    """Convert Quaternion (xyzw) to Euler angles (rpy)."""
    # Normalize
    quat = quat / np.linalg.norm(quat)
    return Rotation.from_quat(quat).as_euler("xyz")


def convert_euler_to_quaternion(euler):
    """Convert Euler angles (rpy) to Quaternion (xyzw)."""
    return Rotation.from_euler("xyz", euler).as_quat()


def convert_euler_to_rotation_matrix(euler):
    """Convert Euler angles (rpy) to rotation matrix (3x3)."""
    return Rotation.from_euler("xyz", euler).as_matrix()


def convert_rotation_matrix_to_euler(rotmat):
    """Convert rotation matrix (3x3) to Euler angles (rpy)."""
    r = Rotation.from_matrix(rotmat)
    return r.as_euler("xyz", degrees=False)


def normalize_vector(v):
    """TODO: Add docstring."""
    v_mag = np.linalg.norm(v, axis=-1, keepdims=True)
    v_mag = np.maximum(v_mag, 1e-8)
    return v / v_mag


def cross_product(u, v):
    """TODO: Add docstring."""
    i = u[:, 1] * v[:, 2] - u[:, 2] * v[:, 1]
    j = u[:, 2] * v[:, 0] - u[:, 0] * v[:, 2]
    k = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]

    return np.stack((i, j, k), axis=1)


def compute_rotation_matrix_from_ortho6d(ortho6d):
    """TODO: Add docstring."""
    x_raw = ortho6d[:, 0:3]
    y_raw = ortho6d[:, 3:6]

    x = normalize_vector(x_raw)
    z = cross_product(x, y_raw)
    z = normalize_vector(z)
    y = cross_product(z, x)

    x = x.reshape(-1, 3, 1)
    y = y.reshape(-1, 3, 1)
    z = z.reshape(-1, 3, 1)
    return np.concatenate((x, y, z), axis=2)


def compute_ortho6d_from_rotation_matrix(matrix):
    # The ortho6d represents the first two column vectors a1 and a2 of the
    # rotation matrix: [ | , |,  | ]
    #                  [ a1, a2, a3]
    #                  [ | , |,  | ]
    """TODO: Add docstring."""
    return matrix[:, :, :2].transpose(0, 2, 1).reshape(matrix.shape[0], -1)
