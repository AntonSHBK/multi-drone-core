from __future__ import annotations

from typing import TYPE_CHECKING, Literal, Optional

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G22_SpiralMove(BaseMoveGCommand):
    """
    G22: Движение по спирали.
    """

    def __init__(
        self,
        counter: int = 0,
        center_point: Optional[list | np.ndarray] = None,
        radius_start: float = 1.0,
        radius_end: float = 2.0,
        height_change: float = 5.0,
        turns: int = 3,
        direction: Literal["CW", "CCW"] = "CW",
        points_per_turn: int = 100,
        velocity: Optional[float] = None,
        yaw_mode: Literal["fixed", "facing_center"] = "facing_center",
        coordinate_system: "CoordinateSystem" = "global_ENU",
        current_step: int = 0,
    ) -> None:
        """
        :param counter: Счётчик команды.
        :param center_point: Центр спирали [x, y, z] (list или ndarray).
        :param radius_start: Начальный радиус.
        :param radius_end: Конечный радиус.
        :param height_change: Изменение высоты за всю спираль.
        :param turns: Количество витков.
        :param direction: Направление ("CW"/"CCW").
        :param points_per_turn: Количество точек на один виток.
        :param velocity: Скорость (м/с). Пока не используется.
        :param yaw_mode: Режим yaw: "fixed" или "facing_center".
        :param coordinate_system: Система координат.
        :param current_step: Текущий шаг выполнения.
        """
        super().__init__(
            name="G22",
            counter=counter,
            coordinate_system=coordinate_system,
            current_step=current_step,
        )

        self.center_point = (
            None if center_point is None else np.asarray(center_point, dtype=float)
        )
        self.radius_start = float(radius_start)
        self.radius_end = float(radius_end)
        self.height_change = float(height_change)
        self.turns = int(turns)
        self.direction: Literal["CW", "CCW"] = direction.upper()
        self.points_per_turn = int(points_per_turn)
        self.velocity = None if velocity is None else float(velocity)
        self.yaw_mode: Literal["fixed", "facing_center"] = yaw_mode

        self.description = "Spiral movement"

        self.update_targets_positions()

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        if self.center_point is None:
            raise ValueError("center_point is required for G22.")
        if self.center_point.shape != (3,):
            raise ValueError("center_point must be ndarray with shape (3,).")

        if self.radius_start < 0.0 or self.radius_end < 0.0:
            raise ValueError("radius_start and radius_end must be >= 0")
        if self.turns <= 0:
            raise ValueError("turns must be > 0")
        if self.points_per_turn < 3:
            raise ValueError("points_per_turn must be >= 3")
        if self.direction not in ("CW", "CCW"):
            raise ValueError("direction must be 'CW' or 'CCW'")
        if self.yaw_mode not in ("fixed", "facing_center"):
            raise ValueError("yaw_mode must be 'fixed' or 'facing_center'")

        total_points = self.turns * self.points_per_turn + 1
        sign = -1.0 if self.direction == "CW" else 1.0

        cx, cy, cz = self.center_point.astype(float)

        for i in range(total_points):
            t = i / (total_points - 1)
            angle = sign * (2.0 * np.pi * self.turns * t)
            radius = self.radius_start + (self.radius_end - self.radius_start) * t

            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            z = cz + self.height_change * t

            yaw = None
            if self.yaw_mode == "facing_center":
                yaw = float(np.arctan2(cy - y, cx - x))

            self.targets_positions.append(
                MoveTarget(
                    position=np.array([x, y, z], dtype=float),
                    yaw=yaw,
                )
            )

        self.ready()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(
            {
                "center_point": None if self.center_point is None else self.center_point.tolist(),
                "radius_start": self.radius_start,
                "radius_end": self.radius_end,
                "height_change": self.height_change,
                "turns": self.turns,
                "direction": self.direction,
                "points_per_turn": self.points_per_turn,
                "velocity": self.velocity,
                "yaw_mode": self.yaw_mode,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G22_SpiralMove":
        center_raw = data.get("center_point")
        center = None if center_raw is None else np.asarray(center_raw, dtype=float)

        return cls(
            counter=data.get("counter", 0),
            center_point=center,
            radius_start=data.get("radius_start", 1.0),
            radius_end=data.get("radius_end", 2.0),
            height_change=data.get("height_change", 5.0),
            turns=data.get("turns", 3),
            direction=data.get("direction", "CW"),
            points_per_turn=data.get("points_per_turn", 100),
            velocity=data.get("velocity"),
            yaw_mode=data.get("yaw_mode", "facing_center"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0),
        )

    def __repr__(self) -> str:
        return (
            "G22_SpiralMove("
            f"counter={self.counter}, center_point={None if self.center_point is None else self.center_point.tolist()}, "
            f"radius_start={self.radius_start}, radius_end={self.radius_end}, "
            f"height_change={self.height_change}, turns={self.turns}, direction='{self.direction}', "
            f"points_per_turn={self.points_per_turn}, velocity={self.velocity}, yaw_mode='{self.yaw_mode}', "
            f"coordinate_system='{self.coordinate_system}', current_step={self.current_step}, "
            f"complete={self.complete})"
        )
