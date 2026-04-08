import numpy as np
import pytest

from multi_drone_core.controllers.position_transformer import DroneLocalityState


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

    state.update_orientation(q)

    assert state.get_orientation("local_ENU") == pytest.approx(yaw)
    assert state.get_orientation("local_NED") == pytest.approx(-yaw)
    assert state.get_orientation("global_ENU") == pytest.approx(yaw)
    assert state.get_orientation("global_NED") == pytest.approx(-yaw)


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
        assert set(frame.keys()) == {"position", "velocity", "yaw"}
        assert isinstance(frame["position"], np.ndarray)
        assert isinstance(frame["velocity"], np.ndarray)
        assert isinstance(frame["yaw"], float)

