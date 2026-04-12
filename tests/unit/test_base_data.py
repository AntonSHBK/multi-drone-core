import numpy as np
import pytest

from multi_drone_core.controllers.base_data import (
    AccelerationData,
    EulerData,
    OrientationData,
    PositionData,
    QuaternionData,
    VelocityData,
)


def test_euler_to_array_returns_ndarray():
    data = EulerData(roll=0.1, pitch=0.2, yaw=0.3)
    vec = data.to_array()
    assert isinstance(vec, np.ndarray)
    np.testing.assert_allclose(vec, np.array([0.1, 0.2, 0.3]))


def test_position_to_array_returns_ndarray():
    data = PositionData(x=1.0, y=2.0, z=3.0)
    vec = data.to_array()
    assert isinstance(vec, np.ndarray)
    np.testing.assert_allclose(vec, np.array([1.0, 2.0, 3.0]))


def test_velocity_to_array_returns_ndarray():
    data = VelocityData(vx=1.0, vy=2.0, vz=3.0)
    vec = data.to_array()
    assert isinstance(vec, np.ndarray)
    np.testing.assert_allclose(vec, np.array([1.0, 2.0, 3.0]))


def test_acceleration_to_array_returns_ndarray():
    data = AccelerationData(ax=1.0, ay=2.0, az=3.0)
    vec = data.to_array()
    assert isinstance(vec, np.ndarray)
    np.testing.assert_allclose(vec, np.array([1.0, 2.0, 3.0]))


def test_quaternion_to_ndarray_returns_ndarray():
    data = QuaternionData(x=0.0, y=0.0, z=0.0, w=2.0)  # будет нормализован
    quat = data.to_ndarray()
    assert isinstance(quat, np.ndarray)
    np.testing.assert_allclose(quat, np.array([0.0, 0.0, 0.0, 1.0]))


def test_orientation_to_array_and_quaternion_return_ndarray():
    data = OrientationData(roll=0.1, pitch=-0.2, yaw=0.3)
    euler = data.euler.to_array()
    quat = data.to_ndarray()

    assert isinstance(euler, np.ndarray)
    assert isinstance(quat, np.ndarray)
    assert euler.shape == (3,)
    assert quat.shape == (4,)


def test_position_update_from_array_invalid_shape_raises():
    data = PositionData()
    with pytest.raises(ValueError):
        data.update_from_array(np.array([1.0, 2.0]))


def test_velocity_update_from_array_invalid_shape_raises():
    data = VelocityData()
    with pytest.raises(ValueError):
        data.update_from_array(np.array([1.0, 2.0]))


def test_quaternion_update_from_array_invalid_shape_raises():
    data = QuaternionData()
    with pytest.raises(ValueError):
        data.update_from_array(np.array([1.0, 2.0, 3.0]))


def test_euler_update_from_quaternion_invalid_shape_raises():
    data = EulerData()
    with pytest.raises(ValueError):
        data.update_from_quaternion(np.array([1.0, 2.0, 3.0]))


def test_orientation_sync_after_euler_update():
    data = OrientationData()
    data.update_from_euler(roll=0.4, pitch=0.1, yaw=-0.2)

    q = data.to_ndarray()
    assert isinstance(q, np.ndarray)
    assert q.shape == (4,)
    np.testing.assert_allclose(np.linalg.norm(q), 1.0, atol=1e-8)


def test_orientation_init_prefers_quaternion_when_provided():
    q = QuaternionData(x=0.1, y=0.2, z=0.3, w=0.9)
    data = OrientationData(x=q.x, y=q.y, z=q.z, w=q.w)
    np.testing.assert_allclose(data.quaternion.to_array(), q.to_array())
    np.testing.assert_allclose(
        data.euler.to_array(),
        QuaternionData(x=q.x, y=q.y, z=q.z, w=q.w).to_euler(),
        atol=1e-6,
    )


def test_euler_quaternion_roundtrip_xyz():
    euler = np.array([0.3, -0.4, 0.5])
    q = QuaternionData()
    q.update_from_euler(euler)
    euler_back = q.to_euler()
    np.testing.assert_allclose(euler_back, euler, atol=1e-6)


def test_to_dict_returns_values():
    e = EulerData(roll=1.0, pitch=2.0, yaw=3.0).to_dict()
    assert e == {"roll": 1.0, "pitch": 2.0, "yaw": 3.0}

    p = PositionData(x=1.0, y=2.0, z=3.0).to_dict()
    assert p == {"x": 1.0, "y": 2.0, "z": 3.0}

    v = VelocityData(vx=1.0, vy=2.0, vz=3.0).to_dict()
    assert v == {"vx": 1.0, "vy": 2.0, "vz": 3.0}

    a = AccelerationData(ax=1.0, ay=2.0, az=3.0).to_dict()
    assert a == {"ax": 1.0, "ay": 2.0, "az": 3.0}

    q = QuaternionData(x=0.0, y=0.0, z=0.0, w=1.0).to_dict()
    assert q == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
