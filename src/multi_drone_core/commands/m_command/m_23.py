from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M23_Offboard(BaseCommand):
    """
    Переключение в OFFBOARD режим.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M23", counter=counter)
        self.description = "Switch to OFFBOARD mode"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.offboard_start()
        controller.set_offboard_mode()

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()
    
    @classmethod
    def from_dict(cls, data: dict) -> "M23_Offboard":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M23_Offboard(counter={self.counter}, complete={self.complete})"
