from __future__ import annotations

from typing import TYPE_CHECKING, Literal, Optional

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G00_MoveToPoint(BaseMoveGCommand):
    """
    G00: Движение в точку.
    """

    def __init__(
        self,
        counter: int = 0,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        velocity: Optional[float] = None,
        yaw: Optional[float] = None,
        yaw_speed: Optional[float] = None,
        coordinate_system: "CoordinateSystem" = "global_ENU",
    ) -> None:
        """
        :param counter: Счётчик команды.
        :param x: Координата X целевой точки.
        :param y: Координата Y целевой точки.
        :param z: Координата Z целевой точки.
        :param yaw: Угол ориентации в радианах.
        :param velocity: Скорость перемещения (м/с).
         yaw_speed: Скорость вращения.
        :param coordinate_system: Система координат для перемещения.
        """
        super().__init__(
            name="G00",
            counter=counter,
            coordinate_system=coordinate_system,
        )
        self.x = None if x is None else float(x)
        self.y = None if y is None else float(y)
        self.z = None if z is None else float(z)
        self.velocity = None if velocity is None else abs(float(velocity))
        self.yaw = None if yaw is None else float(yaw)
        self.yaw_speed = None if yaw_speed is None else float(yaw_speed)
        self.description = "Move to point"

        self.update_targets_positions()

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        point_values = (self.x, self.y, self.z)
        provided_count = sum(value is not None for value in point_values)

        if provided_count not in (0, 3):
            raise ValueError(
                "For G00 either provide all coordinates (x, y, z) or none of them."
            )

        target_position = None
        if provided_count == 3:
            target_position = np.array([self.x, self.y, self.z], dtype=float)

        target_velocity = None
        # if self.velocity is not None and target_position is None:
        #     target_velocity = np.zeros(3, dtype=float)

        self.targets_positions.append(
            MoveTarget(
                position=target_position,
                velocity=target_velocity,
                yaw=self.yaw,
                # yaw_speed=self.yaw_speed,
            )
        )

        self.ready()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(
            {
                "x": self.x,
                "y": self.y,
                "z": self.z,
                "yaw": self.yaw,
                "yaw_speed": self.yaw_speed,
                "velocity": self.velocity,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G00_MoveToPoint":
        return cls(
            counter=data.get("counter", 0),
            x=data.get("x", None),
            y=data.get("y", None),
            z=data.get("z", None),
            yaw=data.get("yaw", None),
            yaw_speed=data.get("yaw_speed", None),
            velocity=data.get("velocity", None),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
        )

    def __repr__(self) -> str:
        return (
            "G00_MoveToPoint("
            f"counter={self.counter}, x={self.x}, y={self.y}, z={self.z}, "
            f"yaw={self.yaw}, yaw_speed={self.yaw_speed}, velocity={self.velocity}, "
            f"coordinate_system='{self.coordinate_system}', complete={self.complete})"
        )
