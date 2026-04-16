from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M10_Arm(BaseCommand):
    """
    Команда ARM.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M10", counter=counter)
        self.description = "Arm drone"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.arm()

    def is_complete(self, controller: "BaseController") -> bool:
        if controller.check_armed():
            self.complete_command()
        return self._check_finish()

    def to_dict(self) -> dict:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict) -> "M10_Arm":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M10_Arm(counter={self.counter}, complete={self.complete})"
