from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M22_Position(BaseCommand):
    """
    Переключение в POSITION (POSCTL) режим.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M22", counter=counter)
        self.description = "Switch to POSITION mode"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.set_posctl_mode()
        self.complete_command()

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()

    @classmethod
    def from_dict(cls, data: dict) -> "M22_Position":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M22_Position(counter={self.counter}, complete={self.complete})"
