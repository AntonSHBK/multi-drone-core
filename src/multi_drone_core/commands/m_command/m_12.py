from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M12_Takeoff(BaseCommand):
    """
    Команда TAKEOFF.
    """

    def __init__(self, counter: int = 0, takeoff_altitude: float = 3.0):
        super().__init__(name="M12", counter=counter, is_special_command=True)
        self.takeoff_altitude = float(takeoff_altitude)
        self.description = "Takeoff"

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        controller.auto_takeoff(altitude=self.takeoff_altitude)
        controller.log_info(
            f"M12_Takeoff: отправлена команда взлёта на высоту {self.takeoff_altitude:.2f} м."
        )
        self.complete_command()

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data["takeoff_altitude"] = self.takeoff_altitude
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "M12_Takeoff":
        return cls(
            counter=data.get("counter", 0),
            takeoff_altitude=data.get("takeoff_altitude", 3.0),
        )

    def __repr__(self) -> str:
        return (
            f"M12_Takeoff(counter={self.counter}, complete={self.complete}, "
            f"takeoff_altitude={self.takeoff_altitude})"
        )
