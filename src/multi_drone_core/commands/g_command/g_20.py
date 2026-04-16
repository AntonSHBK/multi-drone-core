from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G20_PolylineMove(BaseMoveGCommand):
    """
    G20: Движение по ломаной траектории.
    """

    def __init__(
        self,
        points: list | np.ndarray,
        counter: int = 0,
        velocity: float = 1.0,
        coordinate_system: "CoordinateSystem" = "global_ENU",
        current_step: int = 0,
    ) -> None:
        """
        :param points: Точки траектории (list или ndarray), ожидаем форма (N, 3).
        :param counter: Счётчик команды.
        :param velocity: Скорость перемещения (в м/с). Пока не используется.
        :param coordinate_system: Система координат.
        :param current_step: Текущий шаг выполнения.
        """
        super().__init__(
            name="G20",
            counter=counter,
            coordinate_system=coordinate_system,
            current_step=current_step,
        )

        self.points = [np.asarray(point, dtype=float) for point in points]
        self.velocity = float(velocity)
        self.description = "Polyline movement"

        self.update_targets_positions()

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        if len(self.points) < 2:
            raise ValueError("points must contain at least 2 points for polyline movement.")

        for point in self.points:
            if point.shape != (3,):
                raise ValueError("each point must be ndarray with shape (3,).")
            self.targets_positions.append(
                MoveTarget(position=np.asarray(point, dtype=float))
            )

        self.ready()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(
            {
                "points": [point.tolist() for point in self.points],
                "velocity": self.velocity,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G20_PolylineMove":
        return cls(
            points=[np.asarray(point, dtype=float) for point in data["points"]],
            counter=data.get("counter", 0),
            velocity=data.get("velocity", 1.0),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0),
        )

    def __repr__(self) -> str:
        return (
            "G20_PolylineMove("
            f"counter={self.counter}, points={[point.tolist() for point in self.points]}, "
            f"velocity={self.velocity}, "
            f"coordinate_system='{self.coordinate_system}', "
            f"current_step={self.current_step}, complete={self.complete})"
        )
