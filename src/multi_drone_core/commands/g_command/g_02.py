from __future__ import annotations

from typing import TYPE_CHECKING, Literal, Optional

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G02_CircleMove(BaseMoveGCommand):
    """
    G02: Круг/дуга (по часовой / против часовой) между двумя точками.
    """

    def __init__(
        self,
        start_point: list | np.ndarray,
        end_point: list | np.ndarray,
        counter: int = 0,
        radius: float = 1.0,
        direction: Literal["CW", "CCW"] = "CW",
        points_count: int = 10,
        velocity: float = 1.0,
        yaw: Optional[float] = None,
        coordinate_system: "CoordinateSystem" = "global_ENU",
        current_step: int = 0,
    ) -> None:
        """
        :param counter: Счётчик команды.
        :param start_point: Начальная точка [x, y, z] (list или ndarray).
        :param end_point: Конечная точка [x, y, z] (list или ndarray).
        :param radius: Радиус дуги.
        :param direction: Направление ("CW" - по часовой, "CCW" - против часовой).
        :param points_count: Количество точек на дуге.
        :param velocity: Скорость перемещения (в м/с). Пока не используется.
        :param yaw: Угол ориентации (в радианах).
        :param coordinate_system: Система координат.
        """
        super().__init__(
            name="G02",
            counter=counter,
            coordinate_system=coordinate_system,
            current_step=current_step,
        )

        self.start_point = np.asarray(start_point, dtype=float)
        self.end_point = np.asarray(end_point, dtype=float)
        self.radius = float(radius)
        self.direction: Literal["CW", "CCW"] = direction.upper()
        self.points_count = int(points_count)
        self.velocity = float(velocity)
        self.yaw = None if yaw is None else float(yaw)

        self.description = "Circle/arc movement from start point to end point"

        self.update_targets_positions()

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        if self.start_point.shape != (3,) or self.end_point.shape != (3,):
            raise ValueError("start_point and end_point must be 3-element vectors [x, y, z].")

        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")

        if self.direction not in ("CW", "CCW"):
            raise ValueError("direction must be 'CW' or 'CCW'")

        points_count = max(2, self.points_count)

        start_xy = self.start_point[:2]
        end_xy = self.end_point[:2]
        chord = end_xy - start_xy
        d = float(np.linalg.norm(chord))

        if d > 2.0 * self.radius + 1e-9:
            raise ValueError("No circle possible: distance(start,end) > 2 * radius")

        if d < 1e-9:
            center = start_xy + np.array([self.radius, 0.0], dtype=float)
            start_angle = float(np.arctan2(start_xy[1] - center[1], start_xy[0] - center[0]))
            sweep = 2.0 * np.pi
            sign = -1.0 if self.direction == "CW" else 1.0

            for i in range(points_count):
                t = i / (points_count - 1)
                angle = start_angle + sign * sweep * t
                x = center[0] + self.radius * np.cos(angle)
                y = center[1] + self.radius * np.sin(angle)
                z = float(self.start_point[2])

                self.targets_positions.append(
                    MoveTarget(
                        position=np.array([x, y, z], dtype=float),
                        yaw=self.yaw,
                    )
                )

            self.ready()
            return

        mid = 0.5 * (start_xy + end_xy)
        h = float(np.sqrt(max(self.radius * self.radius - (d * 0.5) * (d * 0.5), 0.0)))

        u = chord / d
        perp_left = np.array([-u[1], u[0]], dtype=float)

        centers = [mid + h * perp_left, mid - h * perp_left]

        def sweep_for_center(center_xy: np.ndarray) -> tuple[float, float]:
            a0 = float(np.arctan2(start_xy[1] - center_xy[1], start_xy[0] - center_xy[0]))
            a1 = float(np.arctan2(end_xy[1] - center_xy[1], end_xy[0] - center_xy[0]))
            if self.direction == "CCW":
                delta = (a1 - a0) % (2.0 * np.pi)
            else:
                delta = (a0 - a1) % (2.0 * np.pi)
            return a0, delta

        candidates: list[tuple[np.ndarray, float, float]] = []
        for center in centers:
            a0, delta = sweep_for_center(center)
            candidates.append((center, a0, delta))

        center, start_angle, sweep = min(candidates, key=lambda item: item[2])
        sign = -1.0 if self.direction == "CW" else 1.0

        start_z = float(self.start_point[2])
        end_z = float(self.end_point[2])

        for i in range(points_count):
            t = i / (points_count - 1)
            angle = start_angle + sign * sweep * t
            x = center[0] + self.radius * np.cos(angle)
            y = center[1] + self.radius * np.sin(angle)
            z = start_z + (end_z - start_z) * t

            self.targets_positions.append(
                MoveTarget(
                    position=np.array([x, y, z], dtype=float),
                    yaw=self.yaw,
                )
            )

        self.targets_positions[-1] = MoveTarget(
            position=self.end_point.copy(),
            yaw=self.yaw,
        )

        self.ready()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(
            {
                "start_point": self.start_point.tolist(),
                "end_point": self.end_point.tolist(),
                "radius": self.radius,
                "direction": self.direction,
                "points_count": self.points_count,
                "velocity": self.velocity,
                "yaw": self.yaw,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G02_CircleMove":
        return cls(
            start_point=np.asarray(data["start_point"], dtype=float),
            end_point=np.asarray(data["end_point"], dtype=float),
            counter=data.get("counter", 0),
            radius=data.get("radius", 1.0),
            direction=data.get("direction", "CW"),
            points_count=data.get("points_count", 10),
            velocity=data.get("velocity", 1.0),
            yaw=data.get("yaw"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0),
        )

    def __repr__(self) -> str:
        return (
            "G02_CircleMove("
            f"counter={self.counter}, start_point={self.start_point.tolist()}, "
            f"end_point={self.end_point.tolist()}, radius={self.radius}, "
            f"direction='{self.direction}', points_count={self.points_count}, "
            f"velocity={self.velocity}, yaw={self.yaw}, "
            f"coordinate_system='{self.coordinate_system}', current_step={self.current_step}, "
            f"complete={self.complete})"
        )
