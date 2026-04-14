from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M00_Stop(BaseCommand):
    """
    Аварийная остановка выполнения команд.
    """

    def __init__(self, counter: int = 0, velocity_tolerance: float = 0.05):
        super().__init__(name="M00", counter=counter, is_special_command=True)
        self.velocity_tolerance = velocity_tolerance
        self.description = "Stop all command execution"

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        commander = controller.commander

        if commander.active_command is not None:
            commander.active_command.mark_as_interrupted()

        velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        controller.send_offboard_setpoint(velocity=velocity)
        controller.log_warning("M00_Stop: Выполнена команда STOP. Активная команда прервана, очередь очищена.")

    def is_complete(self, controller: "BaseController") -> bool:
        current_velocity = controller.get_current_velocity(system="local_ENU")

        if all(abs(v) <= self.velocity_tolerance for v in current_velocity):
            self.complete_command()

        return self._check_finish()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data["velocity_tolerance"] = self.velocity_tolerance
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "M00_Stop":
        return cls(
            counter=data.get("counter", 0),
            velocity_tolerance=data.get("velocity_tolerance", 0.05),
        )

    def __repr__(self) -> str:
        return (
            f"M00_Stop(counter={self.counter}, complete={self.complete}, "
            f"interrupt={self.interrupt})"
        )
