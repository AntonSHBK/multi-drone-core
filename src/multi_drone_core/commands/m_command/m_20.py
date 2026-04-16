from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M20_Manual(BaseCommand):
    """
    Переключение в MANUAL режим.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M20", counter=counter)
        self.description = "Switch to MANUAL mode"
        self.ready()

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.set_manual_mode()

    def is_complete(self, controller: "BaseController") -> bool:
        # TODO: Добавить логику остановки
        return self._check_finish()

    @classmethod
    def from_dict(cls, data: dict) -> "M20_Manual":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M20_Manual(counter={self.counter}, complete={self.complete})"
