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

    def __init__(self, counter: int = 0):
        super().__init__(name="M00", counter=counter, is_special_command=True)
        self.description = "Stop all command execution"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        commander = controller.commander
        commander.stop_command_execution()
        controller.log_warning("M00_Stop: Выполнена команда STOP. Активная команда прервана, очередь очищена.")

    def is_complete(self, controller: "BaseController") -> bool:
        if controller.commander.active_command is None:
            return self.complete_command()
        return self._check_finish()
    
    @classmethod
    def from_dict(cls, data: dict) -> "M00_Stop":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return (
            f"M00_Stop(counter={self.counter}, complete={self.complete}, "
            f"interrupt={self.interrupt})"
        )
