from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M02_Continue(BaseCommand):
    """
    Команда продолжения выполнения после паузы/остановки.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M02", counter=counter, is_special_command=True)
        self.description = "Continue command execution"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.commander.resume_command_execution()

    def is_complete(self, controller: "BaseController") -> bool:
        if not controller.commander._execution_paused:
            self.complete_command()
        return self._check_finish()

    def to_dict(self) -> dict:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict) -> "M02_Continue":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M02_Continue(counter={self.counter}, complete={self.complete})"
