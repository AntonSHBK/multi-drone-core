from __future__ import annotations

from pathlib import Path
from dataclasses import dataclass
from abc import ABC
from typing import TYPE_CHECKING, Any, Dict, Optional

import numpy as np

from multi_drone_core.backend.base_backend import BaseBackend
from multi_drone_core.controllers.states import MachineStateMonitor
from multi_drone_core.controllers.base_data import OrientationData, PositionData
from multi_drone_core.controllers.position_transformer import DroneLocalityState
from multi_drone_core.commands.common_commander import CommonCommander
from multi_drone_core.utils.logger import get_core_loggers

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


@dataclass
class ControllerConfig:
    execute_command_timer: float = 0.1
    
    state_monitor_time_period: float = 0.1


class BaseController(ABC):
    """
    Base core controller.
    """

    def __init__(
        self,
        machine_id: int,
        machine_type: str = "default",
        world_position_enu: np.ndarray = [0.0, 0.0, 0.0],
        world_orientation_enu_rpy: np.ndarray = [0.0, 0.0, 0.0],
        log_dir: Path = Path("logs"),
        log_level: str = "INFO",
        config: ControllerConfig = ControllerConfig(),
    ) -> None:
        
        # TODO: проверять id на уникальность
        self.machine_id = int(machine_id)
        self.machine_type = str(machine_type)
        self.machine_name = f"machine_{self.machine_id}"

        self.config = config

        self.loggers = get_core_loggers(
            node_id=self.machine_name,
            log_dir=log_dir,
            log_level=log_level,
        )

        default_position = np.asarray(world_position_enu, dtype=float)
        default_orientation = np.asarray(world_orientation_enu_rpy, dtype=float)

        self.world_position_enu = PositionData(
            x=default_position[0],
            y=default_position[1],
            z=default_position[2],
        )
        self.world_orientation_enu = OrientationData(
            roll=default_orientation[0],
            pitch=default_orientation[1],
            yaw=default_orientation[2],
        )

        self.current_state = DroneLocalityState(
            world_position=self.world_position_enu,
            world_orientation=self.world_orientation_enu,
        )
        self.target_state = DroneLocalityState(
            world_position=self.world_position_enu,
            world_orientation=self.world_orientation_enu,
        )

        # self.machine_system_data = MachineSystemData()

        self.commander = CommonCommander(
            self,
            execute_command_timer=self.config.execute_command_timer
        )
        self.machine_state_monitor = MachineStateMonitor(
            self,
            state_monitor_time_period=self.config.state_monitor_time_period 
        )

        self._running = False
        self._backend: BaseBackend | None = None

    def start(self) -> None:
        self.backend.connect()
        self.commander.start()
        self.machine_state_monitor.start()
        self._running = True

    def stop(self) -> None:
        self.commander.stop()
        self.machine_state_monitor.stop()
        self.backend.disconnect()
        self._running = False

    def log_info(self, message: str) -> None:
        self.loggers.controller.info(message)

    def log_warning(self, message: str) -> None:
        self.loggers.controller.warning(message)

    def log_error(self, message: str) -> None:
        self.loggers.controller.error(message)

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def backend(self) -> BaseBackend:
        if self._backend is None:
            raise RuntimeError("Backend is not initialized")
        return self._backend

    def set_backend(self, backend: BaseBackend) -> None:
        self._backend = backend

    def get_current_position(self, system: "CoordinateSystem" = "local_NED") -> np.ndarray:
        return self.current_state.get_position(system=system)

    def get_current_velocity(self, system: "CoordinateSystem" = "local_NED") -> np.ndarray:
        return self.current_state.get_velocity(system=system)

    def get_current_acceleration(self, system: "CoordinateSystem" = "local_NED") -> np.ndarray:
        return self.current_state.get_acceleration(system=system)

    def get_current_orientation_euler(
        self, system: "CoordinateSystem" = "local_NED"
    ) -> np.ndarray:
        return self.current_state.get_orientation_euler(system=system)

    def get_current_orientation_quaternion(
        self, system: "CoordinateSystem" = "local_NED"
    ) -> np.ndarray:
        return self.current_state.get_orientation_quaternion(system=system)

    def send_offboard_setpoint(
        self,
        *,
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        acceleration: np.ndarray | None = None,
        yaw: float | None = None,
        yaw_speed: float | None = None,
        system: str = "global_ENU",
    ) -> None:
        self.backend.send_offboard_setpoint(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            yaw=yaw,
            yaw_speed=yaw_speed,
            system=system,
        )       

    def set_parameter(
        self,
        parm_name: str,
        parm_value: float,
        parm_type: Optional[int] = None,
    ) -> None:
        self.backend.set_parameter(
            parm_name=parm_name,
            parm_value=parm_value,
            parm_type=parm_type,
        )

    def get_parameter(self, parm_name: str) -> float:
        return self.backend.get_parameter(parm_name=parm_name)

    def fetch_all_parameters(self) -> None:
        self.backend.fetch_all_parameters()

    def get_all_parameters(self) -> Dict[str, float]:
        return self.backend.get_all_parameters()

    def set_mode(self, mode) -> None:
        self.backend.set_mode(mode=mode)

    def get_mode(self) -> str | None:
        return self.backend.get_mode()

    def offboard_start(self) -> None:
        self.backend.offboard_start()

    def offboard_stop(self) -> None:
        self.backend.offboard_stop()
        pass

    def check_offboard_mode(self) -> bool:
        return self.backend.check_offboard_mode()
    
    def check_armed(self) -> bool:
        return self.backend.check_armed()

    def arm(self) -> None:
        self.backend.arm()

    def disarm(self) -> None:
        self.backend.disarm()

    def set_offboard_mode(self) -> None:
        self.backend.set_offboard_mode()

    def set_manual_mode(self) -> None:
        self.backend.set_manual_mode()

    def set_loiter_mode(self) -> None:
        self.backend.set_loiter_mode()

    def set_stabilized_mode(self) -> None:
        self.backend.set_stabilized_mode()

    def set_followme_mode(self) -> None:
        self.backend.set_followme_mode()

    def set_altctl_mode(self) -> None:
        self.backend.set_altctl_mode()

    def set_posctl_mode(self) -> None:
        self.backend.set_posctl_mode()

    def set_mission_mode(self) -> None:
        self.backend.set_mission_mode()

    def set_acro_mode(self) -> None:
        self.backend.set_acro_mode()

    def set_rtl_mode(self) -> None:
        self.backend.set_rtl_mode()

    def auto_land(self) -> None:
        self.backend.auto_land()

    def auto_takeoff(self, altitude: float = 2.0) -> None:
        self.backend.auto_takeoff(altitude=altitude)

    def reboot(self) -> None:
        self.backend.reboot()

    def calibrate_gyro(self) -> None:
        self.backend.calibrate_gyro()

    def calibrate_magnetometer(self) -> None:
        self.backend.calibrate_magnetometer()

    def calibrate_accelerometer(self, simple: bool = False) -> None:
        self.backend.calibrate_accelerometer(simple=simple)

    def calibrate_vehicle_level(self) -> None:
        self.backend.calibrate_vehicle_level()

    def calibrate_barometer(self) -> None:
        self.backend.calibrate_barometer()

    def send_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        target_system: Optional[int] = None,
        target_component: Optional[int] = None,
        confirmation: int = 0,
    ) -> Any:
        return self.backend.send_command(
            command=command,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            param5=param5,
            param6=param6,
            param7=param7,
            target_system=target_system,
            target_component=target_component,
            confirmation=confirmation,
        )
