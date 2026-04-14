from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Optional

import numpy as np

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController
    from multi_drone_core.backend.base_offboard_commander import BaseOffboardCommander
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class BaseBackend(ABC):
    """
    Base interface for backend adapters (ROS2/MAVLink).
    """

    def __init__(
        self,
        controller: "BaseController",
        backend_type: str = "default",
    ) -> None:
        self._controller = controller
        self.logger = controller.loggers.backend
        self._backend_type = backend_type

        self.offboard_commander: "BaseOffboardCommander" = None

    @property
    def controller(self) -> "BaseController":
        return self._controller

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """
        Current connection state to the flight controller.
        """

    @abstractmethod
    def connect(self) -> None:
        """
        Initialize and start backend.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """
        Gracefully stop backend and release resources.
        """

    @abstractmethod
    def wait_ready(
        self,
        timeout: float = 15.0,
        poll_period_s: float = 0.05,
    ) -> bool:
        """
        Wait until backend has received all required telemetry and initialization data.
        Returns True when ready, otherwise False on timeout.
        """

    @abstractmethod  
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
    ) -> None:
        """
        Send command
        """
        

    @abstractmethod
    def get_mode(self) -> str | None:
        """
        Read current flight mode.
        """

    @abstractmethod
    def set_mode(self, mode: str) -> None:
        """
        Set flight mode by name.
        """

    @abstractmethod
    def get_parameter(self, parm_name: str) -> float:
        """
        Read one autopilot parameter.
        """
        
    @abstractmethod
    def fetch_all_parameters(self):
        """
        Read all autopilot parameters
        """
        
    
    @abstractmethod
    def get_all_parameters(self) -> Dict[str, float]:
        """
        Get all autopilot parameters.
        """

    @abstractmethod
    def set_parameter(
        self, 
        parm_name: str, 
        parm_value: float,
        parm_type: int = None
    ) -> None:
        """
        Set one autopilot parameter.
        """    
    
    @abstractmethod
    def offboard_start(self):        
        """
        Offboard thread start
        """
        
    @abstractmethod
    def offboard_stop(self):
         """
        Offboard thread stop
        """
    
    @abstractmethod
    def set_manual_mode(self) -> None:
        """
        Set MANUAL mode.
        """

    @abstractmethod
    def set_offboard_mode(self) -> None:
        """
        Set OFFBOARD mode.
        """

    @abstractmethod
    def arm(self) -> None:
        """
        Arm vehicle motors.
        """

    @abstractmethod
    def disarm(self) -> None:
        """
        Disarm vehicle motors.
        """

    @abstractmethod
    def set_loiter_mode(self) -> None:
        """
        Switch vehicle to hold/loiter behavior.
        """
    
    @abstractmethod
    def set_stabilized_mode(self) -> None:
        """
        Set STABILIZED mode.
        """

    @abstractmethod
    def set_followme_mode(self) -> None:
        """
        Set FOLLOWME mode.
        """     
        
    @abstractmethod
    def set_altctl_mode(self) -> None:
        """
        Set ALTCTL (Altitude Control) mode.
        """

    @abstractmethod
    def set_posctl_mode(self) -> None:
        """
        Set POSCTL (Position Control) mode.
        """
        
    @abstractmethod
    def set_mission_mode(self) -> None:
        """
        Execute mission.
        """
    
    @abstractmethod
    def set_acro_mode(self) -> None:
        """
        Set ACRO mode.
        """
        
    @abstractmethod
    def set_rtl_mode(self) -> None:
        """
        Return to launch position.
        """

    @abstractmethod
    def auto_land(self) -> None:
        """
        Start landing procedure.
        """

    @abstractmethod
    def auto_takeoff(self) -> None:
        """
        Start takeoff procedure.
        """

    @abstractmethod
    def send_offboard_setpoint(
        self,
        *,
        position: np.array = None,
        velocity: np.array = None,
        acceleration: np.array = None,
        yaw: float | None = None,
        yaw_speed: float | None = None,
        system: "CoordinateSystem" = "global_ENU",
    ) -> None:
        """
        Send offboard setpoint.
        """

    @abstractmethod
    def check_offboard_mode(self) -> bool:
        """
        Check if vehicle is currently in OFFBOARD mode.
        """
        
    @abstractmethod
    def reboot(self) -> None:
        """
        Reboot autopilot using MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN.
        """


    @abstractmethod
    def calibrate_gyro(self) -> None:
        """
        Start gyroscope calibration.
        """


    @abstractmethod
    def calibrate_magnetometer(self) -> None:
        """
        Start magnetometer calibration.
        """


    @abstractmethod
    def calibrate_accelerometer(self, simple: bool = False) -> None:
        """
        Start accelerometer calibration.

        :param simple: Use simplified calibration procedure.
        """


    @abstractmethod
    def calibrate_vehicle_level(self) -> None:
        """
        Perform vehicle level (accelerometer trim) calibration.
        """


    @abstractmethod
    def calibrate_barometer(self) -> None:
        """
        Start barometer calibration.
        """
