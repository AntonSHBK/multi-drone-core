from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M40_Failsafe(BaseCommand):
    """
    Команда FAILSAFE.
    Текущая реализация: прерывает очередь и переводит в безопасный режим (предпочтительно RTL).
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M40", counter=counter, is_special_command=True)
        self.description = "Enter failsafe mode"

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        pass

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()

    def to_dict(self) -> dict:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict) -> "M40_Failsafe":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M40_Failsafe(counter={self.counter}, complete={self.complete})"
