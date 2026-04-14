from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M42_KillMotors(BaseCommand):
    """
    Команда KILL MOTORS.
    """

    # MAVLink MAV_CMD_COMPONENT_ARM_DISARM
    ARM_DISARM_COMMAND_ID = 400
    # MAVLink force disarm magic number (PX4/ArduPilot)
    FORCE_DISARM_PARAM2 = 21196.0

    def __init__(self, counter: int = 0):
        super().__init__(name="M42", counter=counter, is_special_command=True)
        self.description = "Kill motors"

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        pass

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()

    def to_dict(self) -> dict:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict) -> "M42_KillMotors":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M42_KillMotors(counter={self.counter}, complete={self.complete})"
