from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G01_LineMove(BaseMoveGCommand):
    """
    G01: Линейное движение (точка -> точка).
    """

    def __init__(
        self,
        start_point: list | np.ndarray,
        end_point: list | np.ndarray,
        counter: int = 0,
        velocity: Optional[float] = None,
        start_yaw: Optional[float] = None,
        end_yaw: Optional[float] = None,
        coordinate_system: "CoordinateSystem" = "global_ENU",
    ) -> None:
        """
        :param start_point: Начальная точка [x, y, z] (list или ndarray).
        :param end_point: Конечная точка [x, y, z] (list или ndarray).
        :param counter: Счётчик команды.
        :param velocity: Скорость перемещения (в м/с). Пока не используется.
        :param start_yaw: Целевой yaw в начальной точке (рад).
        :param end_yaw: Целевой yaw в конечной точке (рад).
        :param coordinate_system: Система координат.
        """
        super().__init__(
            name="G01",
            counter=counter,
            coordinate_system=coordinate_system,
        )

        self.start_point = np.asarray(start_point, dtype=float)
        self.end_point = np.asarray(end_point, dtype=float)
        self.velocity = None if velocity is None else abs(float(velocity))
        self.start_yaw = None if start_yaw is None else float(start_yaw)
        self.end_yaw = None if end_yaw is None else float(end_yaw)

        self.description = "Linear movement from start point to end point"

        self.update_targets_positions()

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        if self.start_point.shape != (3,) or self.end_point.shape != (3,):
            raise ValueError("start_point and end_point must be 3-element vectors [x, y, z].")

        # Пока отправляем только position и yaw.
        self.targets_positions.append(
            MoveTarget(
                position=self.start_point,
                yaw=self.start_yaw,
            )
        )
        self.targets_positions.append(
            MoveTarget(
                position=self.end_point,
                yaw=self.end_yaw,
            )
        )

        self.ready()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(
            {
                "start_point": self.start_point.tolist(),
                "end_point": self.end_point.tolist(),
                "velocity": self.velocity,
                "start_yaw": self.start_yaw,
                "end_yaw": self.end_yaw,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G01_LineMove":
        return cls(
            start_point=np.asarray(data["start_point"], dtype=float),
            end_point=np.asarray(data["end_point"], dtype=float),
            counter=data.get("counter", 0),
            velocity=data.get("velocity"),
            start_yaw=data.get("start_yaw"),
            end_yaw=data.get("end_yaw"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
        )

    def __repr__(self) -> str:
        return (
            "G01_LineMove("
            f"counter={self.counter}, "
            f"start_point={self.start_point.tolist()}, "
            f"end_point={self.end_point.tolist()}, "
            f"velocity={self.velocity}, start_yaw={self.start_yaw}, end_yaw={self.end_yaw}, "
            f"coordinate_system='{self.coordinate_system}', complete={self.complete})"
        )
