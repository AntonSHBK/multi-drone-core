import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from multi_drone_core.controllers.base_data import OrientationData, PositionData
from multi_drone_core.controllers.position_transformer import DroneLocalityState
from multi_drone_core.utils.geometry import rotated_ENU_NED_quaternion


def test_update_position_from_local_ned_syncs_all_frames():
    state = DroneLocalityState()
    ned = np.array([1.0, 2.0, -3.0])

    state.update_position(ned, system="local_NED")

    np.testing.assert_allclose(state.get_position("local_NED"), np.array([1.0, 2.0, -3.0]))
    np.testing.assert_allclose(state.get_position("local_ENU"), np.array([2.0, 1.0, 3.0]))
    np.testing.assert_allclose(state.get_position("global_ENU"), np.array([2.0, 1.0, 3.0]))
    np.testing.assert_allclose(state.get_position("global_NED"), np.array([1.0, 2.0, -3.0]))


def test_update_velocity_from_global_enu_syncs_all_frames():
    state = DroneLocalityState()
    enu = np.array([4.0, -1.0, 2.5])

    state.update_velocity(enu, system="global_ENU")

    np.testing.assert_allclose(state.get_velocity("global_ENU"), np.array([4.0, -1.0, 2.5]))
    np.testing.assert_allclose(state.get_velocity("global_NED"), np.array([-1.0, 4.0, -2.5]))
    np.testing.assert_allclose(state.get_velocity("local_ENU"), np.array([4.0, -1.0, 2.5]))
    np.testing.assert_allclose(state.get_velocity("local_NED"), np.array([-1.0, 4.0, -2.5]))


def test_update_orientation_sets_enu_and_ned_yaw():
    state = DroneLocalityState()
    yaw = np.pi / 2.0
    q = np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)])

    state.update_orientation_quaternion(q, system="local_ENU")

    expected_ned = R.from_quat(rotated_ENU_NED_quaternion(q)).as_euler("xyz")

    assert state.get_orientation_euler("local_ENU")[2] == pytest.approx(yaw)
    np.testing.assert_allclose(
        state.get_orientation_euler("local_NED"),
        expected_ned,
        atol=1e-6,
    )
    assert state.get_orientation_euler("global_ENU")[2] == pytest.approx(yaw)
    np.testing.assert_allclose(
        state.get_orientation_euler("global_NED"),
        expected_ned,
        atol=1e-6,
    )


def test_get_position_unknown_system_raises():
    state = DroneLocalityState()

    with pytest.raises(ValueError):
        state.get_position("bad_system")


def test_to_dict_returns_expected_structure():
    state = DroneLocalityState()
    state.update_position(np.array([1.0, 1.0, -1.0]), system="local_NED")
    state.update_velocity(np.array([0.5, 0.25, -0.75]), system="local_NED")

    data = state.to_dict()

    assert set(data.keys()) == {"local_enu", "local_ned", "global_enu", "global_ned"}
    for frame in data.values():
        assert set(frame.keys()) == {"position", "velocity", "acceleration", "euler", "quaternion"}
        assert isinstance(frame["position"], np.ndarray)
        assert isinstance(frame["velocity"], np.ndarray)
        assert isinstance(frame["acceleration"], np.ndarray)
        assert isinstance(frame["euler"], np.ndarray)
        assert isinstance(frame["quaternion"], np.ndarray)


def test_update_acceleration_from_local_ned_syncs_all_frames():
    state = DroneLocalityState()
    ned = np.array([0.1, -0.2, 0.3])

    state.update_acceleration(ned, system="local_NED")

    np.testing.assert_allclose(state.get_acceleration("local_NED"), np.array([0.1, -0.2, 0.3]))
    np.testing.assert_allclose(state.get_acceleration("local_ENU"), np.array([-0.2, 0.1, -0.3]))
    np.testing.assert_allclose(state.get_acceleration("global_ENU"), np.array([-0.2, 0.1, -0.3]))
    np.testing.assert_allclose(state.get_acceleration("global_NED"), np.array([0.1, -0.2, 0.3]))


def test_get_orientation_quaternion_returns_quaternion():
    state = DroneLocalityState()
    q = np.array([0.0, 0.0, 0.0, 1.0])
    state.update_orientation_quaternion(q, system="local_NED")

    out = state.get_orientation_quaternion("local_NED")
    assert isinstance(out, np.ndarray)
    assert out.shape == (4,)
    np.testing.assert_allclose(out, q)


def test_update_orientation_euler_matches_quaternion():
    state = DroneLocalityState()
    euler = np.array([0.1, -0.2, 0.3])
    state.update_orientation_euler(euler, system="local_ENU")

    q = state.get_orientation_quaternion("local_ENU")
    euler_back = state.get_orientation_euler("local_ENU")
    assert q.shape == (4,)
    np.testing.assert_allclose(euler_back, euler, atol=1e-6)


def test_position_with_world_offset_and_orientation():
    world_pos = PositionData(x=10.0, y=0.0, z=0.0)
    world_ori = OrientationData(roll=0.0, pitch=0.0, yaw=np.pi / 2.0)
    state = DroneLocalityState(world_position=world_pos, world_orientation=world_ori)

    local_enu = np.array([1.0, 0.0, 0.0])
    state.update_position(local_enu, system="local_ENU")

    expected_global = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).apply(local_enu) + world_pos.to_array()
    np.testing.assert_allclose(state.get_position("global_ENU"), expected_global, atol=1e-6)

    expected_local = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).inv().apply(
        state.get_position("global_ENU") - world_pos.to_array()
    )
    np.testing.assert_allclose(state.get_position("local_ENU"), expected_local, atol=1e-6)


def test_orientation_global_to_local_with_world_orientation():
    world_ori = OrientationData(roll=0.0, pitch=0.0, yaw=np.pi / 2.0)
    state = DroneLocalityState(world_orientation=world_ori)

    global_euler = np.array([0.0, 0.0, 0.0])
    state.update_orientation_euler(global_euler, system="global_ENU")

    expected_local = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).inv().as_euler("xyz")
    np.testing.assert_allclose(state.get_orientation_euler("local_ENU"), expected_local, atol=1e-6)


def test_orientation_global_ned_with_world_orientation():
    world_ori = OrientationData(roll=0.0, pitch=0.0, yaw=np.pi / 2.0)
    state = DroneLocalityState(world_orientation=world_ori)

    global_ned_q = rotated_ENU_NED_quaternion(
        R.from_euler("xyz", [0.0, 0.0, 0.0]).as_quat()
    )
    state.update_orientation_quaternion(global_ned_q, system="global_NED")

    expected_local_enu = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).inv().as_euler("xyz")
    np.testing.assert_allclose(state.get_orientation_euler("local_ENU"), expected_local_enu, atol=1e-6)


def test_update_orientation_quaternion_global_enu():
    world_ori = OrientationData(roll=0.0, pitch=0.0, yaw=np.pi / 4.0)
    state = DroneLocalityState(world_orientation=world_ori)

    global_q = R.from_euler("xyz", [0.0, 0.0, np.pi / 2.0]).as_quat()
    state.update_orientation_quaternion(global_q, system="global_ENU")

    expected_local = R.from_euler("xyz", [0.0, 0.0, np.pi / 4.0]).as_euler("xyz")
    np.testing.assert_allclose(state.get_orientation_euler("local_ENU"), expected_local, atol=1e-6)


def test_position_and_orientation_combined():
    world_pos = PositionData(x=5.0, y=-2.0, z=1.0)
    world_ori = OrientationData(roll=0.0, pitch=0.0, yaw=np.pi / 3.0)
    state = DroneLocalityState(world_position=world_pos, world_orientation=world_ori)

    local_enu_pos = np.array([2.0, 1.0, -1.0])
    local_enu_euler = np.array([0.0, 0.0, np.pi / 6.0])
    state.update_position(local_enu_pos, system="local_ENU")
    state.update_orientation_euler(local_enu_euler, system="local_ENU")

    expected_global_pos = R.from_euler("xyz", [0.0, 0.0, np.pi / 3.0]).apply(local_enu_pos) + world_pos.to_array()
    np.testing.assert_allclose(state.get_position("global_ENU"), expected_global_pos, atol=1e-6)

    expected_global_euler = R.from_euler("xyz", [0.0, 0.0, np.pi / 3.0]).as_euler("xyz") + local_enu_euler
    np.testing.assert_allclose(state.get_orientation_euler("global_ENU"), expected_global_euler, atol=1e-6)
