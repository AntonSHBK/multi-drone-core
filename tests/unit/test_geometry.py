import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from multi_drone_core.utils.geometry import (
    calculate_distance,
    calculate_yaw_towards_target,
    rotate_ENU_NED,
    rotated_ENU_NED_quaternion,
    transform_coordinates,
    transform_orientation,
)


def test_rotate_enu_ned_happy_path():
    vec = np.array([1.0, 2.0, 3.0])
    got = rotate_ENU_NED(vec)
    np.testing.assert_allclose(got, np.array([2.0, 1.0, -3.0]))


def test_rotate_enu_ned_invalid_shape_raises():
    with pytest.raises(ValueError):
        rotate_ENU_NED(np.array([1.0, 2.0]))


def test_rotated_enu_ned_quaternion_happy_path():
    q = np.array([0.1, 0.2, 0.3, 0.9])
    got = rotated_ENU_NED_quaternion(q)
    np.testing.assert_allclose(got, np.array([0.2, 0.1, -0.3, 0.9]))


def test_rotated_enu_ned_quaternion_invalid_shape_raises():
    with pytest.raises(ValueError):
        rotated_ENU_NED_quaternion(np.array([0.0, 0.0, 1.0]))


def test_transform_coordinates_local_to_global_euler_identity():
    source = np.array([1.0, -2.0, 3.0])
    reference_position = np.array([10.0, 20.0, 30.0])
    reference_orientation = np.array([0.0, 0.0, 0.0])

    got = transform_coordinates(
        source_position=source,
        reference_position=reference_position,
        reference_orientation=reference_orientation,
        transform_type="local_to_global",
        rotation_type="euler",
    )

    np.testing.assert_allclose(got, source + reference_position)


def test_transform_coordinates_global_to_local_euler_identity():
    source_global = np.array([11.0, 18.0, 33.0])
    reference_position = np.array([10.0, 20.0, 30.0])
    reference_orientation = np.array([0.0, 0.0, 0.0])

    got = transform_coordinates(
        source_position=source_global,
        reference_position=reference_position,
        reference_orientation=reference_orientation,
        transform_type="global_to_local",
        rotation_type="euler",
    )

    np.testing.assert_allclose(got, np.array([1.0, -2.0, 3.0]))


def test_transform_coordinates_round_trip_quaternion():
    source_local = np.array([2.0, -1.0, 0.5])
    reference_position = np.array([4.0, 5.0, -2.0])
    reference_quat = R.from_euler("zyx", [0.3, -0.2, 0.1]).as_quat()

    global_pos = transform_coordinates(
        source_position=source_local,
        reference_position=reference_position,
        reference_orientation=reference_quat,
        transform_type="local_to_global",
        rotation_type="quaternion",
    )
    back_to_local = transform_coordinates(
        source_position=global_pos,
        reference_position=reference_position,
        reference_orientation=reference_quat,
        transform_type="global_to_local",
        rotation_type="quaternion",
    )

    np.testing.assert_allclose(back_to_local, source_local, atol=1e-8)


def test_transform_coordinates_invalid_rotation_type_raises():
    with pytest.raises(ValueError):
        transform_coordinates(
            source_position=np.array([1.0, 2.0, 3.0]),
            reference_position=np.array([0.0, 0.0, 0.0]),
            reference_orientation=np.array([0.0, 0.0, 0.0]),
            transform_type="local_to_global",
            rotation_type="bad",  # type: ignore[arg-type]
        )


def test_transform_coordinates_invalid_transform_type_raises():
    with pytest.raises(ValueError):
        transform_coordinates(
            source_position=np.array([1.0, 2.0, 3.0]),
            reference_position=np.array([0.0, 0.0, 0.0]),
            reference_orientation=np.array([0.0, 0.0, 0.0]),
            transform_type="bad",  # type: ignore[arg-type]
            rotation_type="euler",
        )


def test_transform_orientation_quaternion_local_to_global_identity_reference():
    source = R.from_euler("zyx", [0.4, -0.1, 0.2]).as_quat()
    identity_ref = np.array([0.0, 0.0, 0.0, 1.0])

    got = transform_orientation(
        source_orientation=source,
        reference_orientation=identity_ref,
        transform_type="local_to_global",
        mode="quaternion",
    )

    np.testing.assert_allclose(got, source, atol=1e-8)


def test_transform_orientation_euler_round_trip():
    source_euler = np.array([0.2, -0.3, 0.1])
    ref_euler = np.array([0.4, 0.2, -0.2])

    global_ori = transform_orientation(
        source_orientation=source_euler,
        reference_orientation=ref_euler,
        transform_type="local_to_global",
        mode="euler",
        euler_order="zyx",
    )
    back_local = transform_orientation(
        source_orientation=global_ori,
        reference_orientation=ref_euler,
        transform_type="global_to_local",
        mode="euler",
        euler_order="zyx",
    )

    np.testing.assert_allclose(back_local, source_euler, atol=1e-8)


def test_transform_orientation_invalid_mode_raises():
    with pytest.raises(ValueError):
        transform_orientation(
            source_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            reference_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            transform_type="local_to_global",
            mode="bad",  # type: ignore[arg-type]
        )


def test_transform_orientation_invalid_transform_type_raises():
    with pytest.raises(ValueError):
        transform_orientation(
            source_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            reference_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            transform_type="bad",  # type: ignore[arg-type]
            mode="quaternion",
        )


def test_calculate_yaw_towards_target_axis_aligned():
    current = np.array([0.0, 0.0, 0.0])
    target_x = np.array([1.0, 0.0, 0.0])
    target_y = np.array([0.0, 1.0, 0.0])

    assert calculate_yaw_towards_target(current, target_x) == pytest.approx(0.0)
    assert calculate_yaw_towards_target(current, target_y) == pytest.approx(np.pi / 2)


def test_calculate_distance_happy_path_and_list_support():
    assert calculate_distance([0.0, 0.0, 0.0], [3.0, 4.0, 0.0]) == pytest.approx(5.0)


def test_calculate_distance_shape_mismatch_raises():
    with pytest.raises(ValueError):
        calculate_distance(np.array([1.0, 2.0]), np.array([1.0, 2.0, 3.0]))
