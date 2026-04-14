from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M01_Pause(BaseCommand):
    """
    Заглушка команды паузы.

    Текущая реализация только отправляет setpoint на нулевую скорость.
    Полноценная логика "pause/resume" будет добавлена позже.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M01", counter=counter, is_special_command=True)
        self.description = "Pause mission (stub)"

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        # velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        # controller.send_offboard_setpoint(velocity=velocity)
        # controller.log_warning(
        #     "M01_Pause: Заглушка. Отправлена нулевая скорость; логика pause/resume пока не реализована."
        # )
        # self.complete_command()
        pass

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()
    
    def to_dict(self) -> dict:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict) -> "M01_Pause":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M01_Pause(counter={self.counter}, complete={self.complete})"
