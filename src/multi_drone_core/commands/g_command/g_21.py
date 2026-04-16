from __future__ import annotations

from typing import TYPE_CHECKING, Optional

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G21_SplineMove(BaseMoveGCommand):
    """
    G21: Движение по сплайну.
    """

    def __init__(
        self,
        points: list | np.ndarray,
        counter: int = 0,
        points_count: int = 50,
        velocity: float = 1.0,
        yaw: Optional[float] = None,
        coordinate_system: "CoordinateSystem" = "global_ENU",
        current_step: int = 0,
    ) -> None:
        """
        :param points: Контрольные точки (list или ndarray), ожидаем форма (N, 3).
        :param counter: Счётчик команды.
        :param points_count: Количество точек траектории после интерполяции.
        :param velocity: Скорость перемещения (в м/с). Пока не используется.
        :param yaw: Целевой yaw (рад). Если задан, одинаковый для всей траектории.
        :param coordinate_system: Система координат.
        :param current_step: Текущий шаг выполнения.
        """
        super().__init__(
            name="G21",
            counter=counter,
            coordinate_system=coordinate_system,
            current_step=current_step,
        )

        self.points = [np.asarray(point, dtype=float) for point in points]
        self.points_count = int(points_count)
        self.velocity = float(velocity)
        self.yaw = None if yaw is None else float(yaw)

        self.description = "Spline movement"

        self.update_targets_positions()

    @staticmethod
    def _catmull_rom(
        p0: np.ndarray,
        p1: np.ndarray,
        p2: np.ndarray,
        p3: np.ndarray,
        t: float,
    ) -> np.ndarray:
        t2 = t * t
        t3 = t2 * t
        return 0.5 * (
            (2.0 * p1)
            + (-p0 + p2) * t
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
            + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
        )

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        if len(self.points) < 2:
            raise ValueError("points must contain at least 2 points for spline movement.")

        for point in self.points:
            if point.shape != (3,):
                raise ValueError("each point must be ndarray with shape (3,).")

        total_points = max(2, self.points_count)

        if len(self.points) == 2:
            p_start = self.points[0]
            p_end = self.points[1]
            for i in range(total_points):
                t = i / (total_points - 1)
                p = (1.0 - t) * p_start + t * p_end
                self.targets_positions.append(
                    MoveTarget(position=np.asarray(p, dtype=float), yaw=self.yaw)
                )
            self.ready()
            return

        segments = len(self.points) - 1
        samples_per_segment = max(1, int(np.ceil((total_points - 1) / segments)))

        generated: list[np.ndarray] = []

        for i in range(segments):
            p1 = self.points[i]
            p2 = self.points[i + 1]
            p0 = self.points[i - 1] if i > 0 else self.points[i]
            p3 = self.points[i + 2] if i + 2 < len(self.points) else self.points[i + 1]

            for j in range(samples_per_segment):
                t = j / samples_per_segment
                generated.append(self._catmull_rom(p0, p1, p2, p3, t))

        generated.append(self.points[-1])

        if len(generated) > total_points:
            idx = np.linspace(0, len(generated) - 1, total_points, dtype=int)
            generated = [generated[k] for k in idx]
        elif len(generated) < total_points:
            while len(generated) < total_points:
                generated.append(self.points[-1].copy())

        for point in generated:
            self.targets_positions.append(
                MoveTarget(position=np.asarray(point, dtype=float), yaw=self.yaw)
            )

        self.targets_positions[-1] = MoveTarget(position=self.points[-1].copy(), yaw=self.yaw)

        self.ready()

    def to_dict(self) -> dict:
        data = super().to_dict()
        data.update(
            {
                "points": [point.tolist() for point in self.points],
                "points_count": self.points_count,
                "velocity": self.velocity,
                "yaw": self.yaw,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G21_SplineMove":
        return cls(
            points=[np.asarray(point, dtype=float) for point in data["points"]],
            counter=data.get("counter", 0),
            points_count=data.get("points_count", 50),
            velocity=data.get("velocity", 1.0),
            yaw=data.get("yaw"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0),
        )

    def __repr__(self) -> str:
        return (
            "G21_SplineMove("
            f"counter={self.counter}, points={[point.tolist() for point in self.points]}, "
            f"points_count={self.points_count}, velocity={self.velocity}, yaw={self.yaw}, "
            f"coordinate_system='{self.coordinate_system}', current_step={self.current_step}, "
            f"complete={self.complete})"
        )
