import numpy as np
from scipy.spatial.transform import Rotation as R

from multi_drone_core.utils.geometry import (
    rotate_ENU_NED,
    rotated_ENU_NED_quaternion,
    rotated_ENU_NED_euler,
    transform_coordinates,
    transform_orientation,
)


def test_rotate_ENU_NED_swaps_axes():
    v = np.array([1.0, 2.0, 3.0])
    out = rotate_ENU_NED(v)
    assert np.allclose(out, np.array([2.0, 1.0, -3.0]))


def test_rotate_ENU_NED_is_involution():
    v = np.array([4.0, -5.0, 6.0])
    out = rotate_ENU_NED(rotate_ENU_NED(v))
    assert np.allclose(out, v)


def test_rotated_ENU_NED_quaternion_is_involution():
    q = R.from_euler("xyz", [0.2, -0.3, 0.4]).as_quat()
    q2 = rotated_ENU_NED_quaternion(rotated_ENU_NED_quaternion(q))
    r1 = R.from_quat(q).as_matrix()
    r2 = R.from_quat(q2).as_matrix()
    assert np.allclose(r1, r2, atol=1e-6)


def test_rotated_ENU_NED_euler_matches_quaternion_conversion():
    euler = np.array([0.1, -0.2, 0.3])
    q = R.from_euler("xyz", euler).as_quat()
    q_conv = rotated_ENU_NED_quaternion(q)
    euler_from_q = R.from_quat(q_conv).as_euler("xyz")
    euler_conv = rotated_ENU_NED_euler(euler, order="xyz")
    assert np.allclose(euler_conv, euler_from_q, atol=1e-6)


def test_transform_coordinates_local_to_global_no_rotation():
    src = np.array([1.0, 2.0, 3.0])
    ref_pos = np.array([10.0, 20.0, 30.0])
    ref_ori = np.array([0.0, 0.0, 0.0])
    out = transform_coordinates(
        source_position=src,
        reference_position=ref_pos,
        reference_orientation=ref_ori,
        transform_type="local_to_global",
        rotation_type="euler",
        sequence="xyz",
    )
    assert np.allclose(out, src + ref_pos)


def test_transform_coordinates_with_yaw_rotation():
    src = np.array([1.0, 0.0, 0.0])
    ref_pos = np.array([0.0, 0.0, 0.0])
    ref_ori = np.array([0.0, 0.0, np.pi / 2.0])
    out = transform_coordinates(
        source_position=src,
        reference_position=ref_pos,
        reference_orientation=ref_ori,
        transform_type="local_to_global",
        rotation_type="euler",
        sequence="xyz",
    )
    assert np.allclose(out, np.array([0.0, 1.0, 0.0]), atol=1e-6)


def test_transform_orientation_quaternion_local_to_global():
    ref_q = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).as_quat()
    src_q = R.from_euler("xyz", [0.0, 0.0, 0.0]).as_quat()
    out_q = transform_orientation(
        source_orientation=src_q,
        reference_orientation=ref_q,
        transform_type="local_to_global",
        mode="quaternion",
    )
    r_out = R.from_quat(out_q).as_matrix()
    r_ref = R.from_quat(ref_q).as_matrix()
    assert np.allclose(r_out, r_ref, atol=1e-6)


def test_transform_orientation_quaternion_global_to_local():
    ref_q = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).as_quat()
    src_q = R.from_euler("xyz", [0.0, 0.0, 0.0]).as_quat()
    out_q = transform_orientation(
        source_orientation=src_q,
        reference_orientation=ref_q,
        transform_type="global_to_local",
        mode="quaternion",
    )
    r_out = R.from_quat(out_q).as_matrix()
    r_ref_inv = R.from_quat(ref_q).inv().as_matrix()
    assert np.allclose(r_out, r_ref_inv, atol=1e-6)
