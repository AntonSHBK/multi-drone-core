from __future__ import annotations

from typing import TYPE_CHECKING, Literal, Optional

import numpy as np

from multi_drone_core.commands.base_move_command import BaseMoveGCommand, MoveTarget

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class G23_OrbitMove(BaseMoveGCommand):
    """
    G23: Движение по орбите вокруг точки.
    """

    def __init__(
        self,
        counter: int = 0,
        center_point: Optional[list | np.ndarray] = None,
        radius: float = 1.0,
        angular_velocity: float = 0.1,
        orbit_direction: Literal["CW", "CCW"] = "CW",
        yaw_mode: Literal["fixed", "facing_center"] = "facing_center",
        duration: Optional[float] = None,
        velocity: Optional[float] = None,
        coordinate_system: "CoordinateSystem" = "global_ENU",
        current_step: int = 0,
    ) -> None:
        """
        :param counter: Счётчик команды.
        :param center_point: Центр орбиты [x, y, z] (list или ndarray).
        :param radius: Радиус орбиты.
        :param angular_velocity: Угловая скорость (рад/с).
        :param orbit_direction: Направление орбиты ("CW"/"CCW").
        :param yaw_mode: Режим yaw: "fixed" или "facing_center".
        :param duration: Длительность полёта по орбите (сек). Если None — один полный виток.
        :param velocity: Линейная скорость (м/с). Пока не используется.
        :param coordinate_system: Система координат.
        :param current_step: Текущий шаг выполнения.
        """
        super().__init__(
            name="G23",
            counter=counter,
            coordinate_system=coordinate_system,
            current_step=current_step,
        )

        self.center_point = (
            None if center_point is None else np.asarray(center_point, dtype=float)
        )
        self.radius = float(radius)
        self.angular_velocity = float(angular_velocity)
        self.orbit_direction: Literal["CW", "CCW"] = orbit_direction.upper()
        self.yaw_mode: Literal["fixed", "facing_center"] = yaw_mode
        self.duration = None if duration is None else float(duration)
        self.velocity = None if velocity is None else float(velocity)

        self.description = "Orbit movement around center point"

        self.update_targets_positions()

    def update_targets_positions(self) -> None:
        self.targets_positions.clear()

        if self.center_point is None:
            raise ValueError("center_point is required for G23.")
        if self.center_point.shape != (3,):
            raise ValueError("center_point must be ndarray with shape (3,).")

        if self.radius <= 0.0:
            raise ValueError("radius must be > 0")
        if self.angular_velocity == 0.0:
            raise ValueError("angular_velocity must be non-zero")
        if self.orbit_direction not in ("CW", "CCW"):
            raise ValueError("orbit_direction must be 'CW' or 'CCW'")
        if self.yaw_mode not in ("fixed", "facing_center"):
            raise ValueError("yaw_mode must be 'fixed' or 'facing_center'")

        omega = abs(self.angular_velocity)

        if self.duration is None:
            total_duration = 2.0 * np.pi / omega
        else:
            if self.duration <= 0.0:
                raise ValueError("duration must be > 0")
            total_duration = self.duration

        sample_rate_hz = 20.0
        total_points = max(2, int(total_duration * sample_rate_hz) + 1)

        sign = -1.0 if self.orbit_direction == "CW" else 1.0
        cx, cy, cz = self.center_point.astype(float)

        for i in range(total_points):
            t = i / (total_points - 1)
            theta = sign * omega * total_duration * t

            x = cx + self.radius * np.cos(theta)
            y = cy + self.radius * np.sin(theta)
            z = cz

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
                "radius": self.radius,
                "angular_velocity": self.angular_velocity,
                "orbit_direction": self.orbit_direction,
                "yaw_mode": self.yaw_mode,
                "duration": self.duration,
                "velocity": self.velocity,
                "coordinate_system": self.coordinate_system,
            }
        )
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "G23_OrbitMove":
        center_raw = data.get("center_point")
        center = None if center_raw is None else np.asarray(center_raw, dtype=float)

        return cls(
            counter=data.get("counter", 0),
            center_point=center,
            radius=data.get("radius", 1.0),
            angular_velocity=data.get("angular_velocity", 0.1),
            orbit_direction=data.get("orbit_direction", "CW"),
            yaw_mode=data.get("yaw_mode", "facing_center"),
            duration=data.get("duration"),
            velocity=data.get("velocity"),
            coordinate_system=data.get("coordinate_system", "global_ENU"),
            current_step=data.get("current_step", 0),
        )

    def __repr__(self) -> str:
        return (
            "G23_OrbitMove("
            f"counter={self.counter}, center_point={None if self.center_point is None else self.center_point.tolist()}, "
            f"radius={self.radius}, angular_velocity={self.angular_velocity}, "
            f"orbit_direction='{self.orbit_direction}', yaw_mode='{self.yaw_mode}', "
            f"duration={self.duration}, velocity={self.velocity}, "
            f"coordinate_system='{self.coordinate_system}', current_step={self.current_step}, "
            f"complete={self.complete})"
        )
