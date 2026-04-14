from __future__ import annotations

from abc import abstractmethod
from typing import Optional

import numpy as np

from multi_drone_core.commands.base_command import BaseCommand
from multi_drone_core.utils.geometry import calculate_distance

from typing import TYPE_CHECKING, List
if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class MoveTarget:
    """
    Единая модель цели для offboard setpoint.
    Совпадает по структуре с параметрами update() в offboard.
    """

    def __init__(
        self,
        position: Optional[np.ndarray] = None,
        velocity: Optional[np.ndarray] = None,
        acceleration: Optional[np.ndarray] = None,
        yaw: Optional[float] = None,
        yaw_speed: Optional[float] = None,
    ) -> None:
        self.position = self._as_vector(position)
        self.velocity = self._as_vector(velocity)
        self.acceleration = self._as_vector(acceleration)
        self.yaw = None if yaw is None else float(yaw)
        self.yaw_speed = None if yaw_speed is None else float(yaw_speed)

    @staticmethod
    def _as_vector(value: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if value is None:
            return None
        arr = np.asarray(value, dtype=float)
        if arr.shape != (3,):
            raise ValueError("Vector must be a 3-element array")
        return arr



class BaseMoveGCommand(BaseCommand):
    """
    Base class for move commands.
    """

    def __init__(
        self,
        name: str,
        counter: int = 0,
        coordinate_system: "CoordinateSystem" = "global_ENU",
        current_step: int = 0,
        yaw_tolerance: float = 0.5,
        position_tolerance: float = 0.5,
        velocity_tolerance: float = 0.2,
        acceleration_tolerance: float = 0.5,
    ):
        super().__init__(name, counter, current_step)
        self.targets_positions: List[MoveTarget] = []
        self.coordinate_system = coordinate_system
        self.yaw_tolerance = float(yaw_tolerance)
        self.position_tolerance = float(position_tolerance)
        self.velocity_tolerance = float(velocity_tolerance)
        self.acceleration_tolerance = float(acceleration_tolerance)

    def can_execute(self, controller: "BaseController") -> bool:
        """
        Check if the drone is in offboard mode.
        """
        if not controller.check_offboard_mode():
            controller.log_error(
                f"{self.name}: command can only be executed in Offboard mode."
            )
            return False
        return True

    @abstractmethod
    def update_targets_positions(self):
        """
        Build target list in child classes.
        """
        raise NotImplementedError(
            "update_targets_positions() must be implemented in child class."
        )

    def execute(self, controller: "BaseController"):
        """
        Execute command for current step.
        """
        if self.current_step < len(self.targets_positions):
            target: MoveTarget = self.targets_positions[self.current_step]
            controller.send_offboard_setpoint(
                position=target.position,
                velocity=target.velocity,
                acceleration=target.acceleration,
                yaw=target.yaw,
                yaw_speed=target.yaw_speed,
                system=self.coordinate_system,
            )
            
    def _is_position_reached(
        self, controller: "BaseController", target: MoveTarget
    ) -> bool:
        if target.position is None:
            return True
        current_position = controller.get_current_position(system=self.coordinate_system)
        position_error = calculate_distance(current_position, target.position)
        return position_error <= self.position_tolerance

    def _is_yaw_reached(
        self, controller: "BaseController", target: MoveTarget
    ) -> bool:
        if target.yaw is None:
            return True
        current_yaw = controller.get_current_orientation_euler(
            system=self.coordinate_system
        )[2]
        yaw_error = abs((current_yaw - target.yaw + np.pi) % (2 * np.pi) - np.pi)
        return yaw_error <= self.yaw_tolerance

    def _is_velocity_reached(
        self, controller: "BaseController", target: MoveTarget
    ) -> bool:
        if target.velocity is None:
            return True
        current_velocity = controller.get_current_velocity(
            system=self.coordinate_system
        )
        velocity_error = calculate_distance(current_velocity, target.velocity)
        return velocity_error <= self.velocity_tolerance

    def _is_acceleration_reached(
        self, controller: "BaseController", target: MoveTarget
    ) -> bool:
        if target.acceleration is None:
            return True
        current_acceleration = controller.get_current_acceleration(
            system=self.coordinate_system
        )
        acceleration_error = calculate_distance(
            current_acceleration, target.acceleration
        )
        return acceleration_error <= self.acceleration_tolerance

    def is_complete(self, controller: "BaseController") -> bool:
        """
        Check if current step is complete and move to the next one.
        """
        if not self.targets_positions:
            self.mark_as_interrupted()
            return self._check_finish()

        if self.current_step < len(self.targets_positions):
            target = self.targets_positions[self.current_step]

            position_ok = self._is_position_reached(controller, target)
            velocity_ok = self._is_velocity_reached(controller, target)
            acceleration_ok = self._is_acceleration_reached(controller, target)
            yaw_ok = self._is_yaw_reached(controller, target)

            if position_ok and yaw_ok and velocity_ok and acceleration_ok:
                self.current_step += 1
                if self.current_step >= len(self.targets_positions):
                    self.complete_command()
        return self._check_finish()
