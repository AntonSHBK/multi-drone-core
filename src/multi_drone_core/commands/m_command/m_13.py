from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M13_Land(BaseCommand):
    """
    Команда LAND.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M13", counter=counter)
        self.description = "Land"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.auto_land()

    def is_complete(self, controller: "BaseController") -> bool:
        # TODO: Добавить логику остановки
        return self._check_finish()

    @classmethod
    def from_dict(cls, data: dict) -> "M13_Land":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M13_Land(counter={self.counter}, complete={self.complete})"
