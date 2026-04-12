from __future__ import annotations

import threading
from typing import TYPE_CHECKING

import numpy as np
from pymavlink import mavutil

if TYPE_CHECKING:
    from multi_drone_core.backend.mavlink.handler import MavlinkBackend
    from multi_drone_core.controllers.position_transformer import CoordinateSystem

class OffboardCommander:
    def __init__(self, backend: "MavlinkBackend") -> None:
        self._backend = backend
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._running = False
        
        self._position: np.ndarray | None = None
        self._velocity: np.ndarray | None = None
        self._acceleration: np.ndarray | None = None
        self._yaw: float | None = None
        self._yaw_speed: float | None = None
        
        rate_hz = max(0.1, float(self._backend._config.offboard_setpoint_rate_hz))
        self.offboard_period = 1.0 / rate_hz

        self.target_orientation_system: "CoordinateSystem" = "local_NED"

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        if self._running:
            return
        if not self._backend.is_connected:
            raise RuntimeError("MAVLink backend is not connected. Call connect() first.")

        with self._state_lock:
            self._sync_target_with_current_state()
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._offboard_loop,
                name="mavlink-offboard-thread",
                daemon=True,
            )
            self._running = True
            self._thread.start()

        # Switch mode after stream startup so setpoints are already flowing.
        # self._backend._set_mode("OFFBOARD")
        # self._backend.log_info("Offboard commander started")

    def stop(self) -> None:
        if not self._running:
            return

        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

        self._thread = None
        self._running = False
        self._backend.log_info("Offboard commander stopped")
    
    def update(
        self, 
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        acceleration: np.ndarray | None = None,
        yaw: float | None = None,
        yaw_speed: float | None = None,
        system: "CoordinateSystem" = "global_ENU",
    ):
        """
        Обновляет целевой setpoint.

        Если параметр не указан, целевой параметр сбрасывается.
        :param position: np.ndarray или список из 3 элементов [x, y, z], позиция.
        :param velocity: np.ndarray или список из 3 элементов [vx, vy, vz], скорость.
        :param acceleration: np.ndarray или список из 3 элементов [ax, ay, az], ускорение.
        :param yaw: float, ориентация в радианах.
        :param yaw_speed: float, скорость изменения yaw в радианах/сек.
        """
        
        target_state = self._backend.controller.target_state
        
        if position is not None:
            position = np.asarray(position, dtype=float)
            target_state.update_position(position, system=system)
            self._position = target_state.get_position(
                system=self.target_orientation_system
            ).copy()
        else:
            self._position = None
            target_state.reset_position()
            
        if velocity is not None:
            velocity = np.asarray(velocity, dtype=float)
            target_state.update_velocity(velocity, system=system)
            self._velocity = target_state.get_velocity(
                system=self.target_orientation_system
            ).copy()
        else:
            self._velocity = None
            target_state.reset_velocity()
            
        if acceleration is not None:
            acceleration = np.asarray(acceleration, dtype=float)
            target_state.update_acceleration(acceleration, system=system)
            self._acceleration = target_state.get_acceleration(
                system=self.target_orientation_system
            ).copy()
        else:
            self._acceleration = None
            target_state.reset_acceleration()
        
        if yaw is not None:
            yaw_value = float(yaw)
            target_state.update_orientation_euler(
                np.array([0.0, 0.0, yaw_value], dtype=float),
                system=system,
            )
            self._yaw = float(
                target_state.get_orientation_euler(system=self.target_orientation_system)[2]
            )
        else:
            self._yaw = None
            target_state.reset_orientation()
        
        if yaw_speed is not None:
            yaw_speed_value = float(yaw_speed)
            self._yaw_speed = -yaw_speed_value if system.endswith("ENU") else yaw_speed_value
        else:
            self._yaw_speed = None
        
    def offboard_send_msg(self) -> None:
        if not self._backend.is_connected:
            return

        position = np.zeros(3, dtype=float) if self._position is None else self._position
        velocity = np.zeros(3, dtype=float) if self._velocity is None else self._velocity
        acceleration = (
            np.zeros(3, dtype=float)
            if self._acceleration is None
            else self._acceleration
        )
        yaw = 0.0 if self._yaw is None else float(self._yaw)
        yaw_rate = 0.0 if self._yaw_speed is None else float(self._yaw_speed)

        type_mask = 0

        if self._position is None:
            type_mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            )

        if self._velocity is None:
            type_mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            )

        if self._acceleration is None:
            type_mask |= (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            )

        if self._yaw is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        if self._yaw_speed is None:
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE

        self._backend._mavlink.set_position_target_local_ned_send(
            time_boot_ms=0,
            target_system=int(self._backend._target_system),
            target_component=int(self._backend._target_component),
            coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask=int(type_mask),
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            vx=float(velocity[0]),
            vy=float(velocity[1]),
            vz=float(velocity[2]),
            afx=float(acceleration[0]),
            afy=float(acceleration[1]),
            afz=float(acceleration[2]),
            yaw=yaw,
            yaw_rate=yaw_rate,
        )

    def _offboard_loop(self) -> None:
        

        while not self._stop_event.is_set():
            try:
                self.offboard_send_msg()
            except Exception as exc:
                self._backend.log_warning(f"Offboard send error: {exc}")

            self._stop_event.wait(self.offboard_period)

    def _sync_target_with_current_state(self) -> None:
        current_state = self._backend.controller.current_state
        target_state = self._backend.controller.target_state

        current_pos_ned = current_state.get_position(system=self.target_orientation_system)
        current_orient_quat_ned = current_state.get_orientation_quaternion(system=self.target_orientation_system)

        target_state.update_position(current_pos_ned, system=self.target_orientation_system)
        target_state.reset_velocity()
        target_state.reset_acceleration()
        target_state.update_orientation_quaternion(
            current_orient_quat_ned,
            system=self.target_orientation_system,
        )

        self._position = current_pos_ned.copy()
        self._velocity = None
        self._acceleration = None
        self._yaw = float(target_state.get_orientation_euler(system=self.target_orientation_system)[2])
        self._yaw_speed = None
