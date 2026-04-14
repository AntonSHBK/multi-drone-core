from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from multi_drone_core.controllers.position_transformer import CoordinateSystem


class BaseOffboardCommander(ABC):
    """
    Базовый класс для offboard-командеров.
    Используется для типизации и создания реализаций для разных систем.
    """

    @property
    @abstractmethod
    def is_running(self) -> bool:
        """
        Возвращает True, если командер активен и отправляет setpoint'ы.
        """
        raise NotImplementedError

    @abstractmethod
    def start(self) -> None:
        """
        Запускает цикл отправки команд.
        """
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """
        Останавливает цикл отправки команд.
        """
        raise NotImplementedError

    @abstractmethod
    def update(
        self,
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        acceleration: np.ndarray | None = None,
        yaw: float | None = None,
        yaw_speed: float | None = None,
        system: "CoordinateSystem" = "global_ENU",
    ) -> None:
        """
        Обновляет целевой setpoint. Любые None значения считаются сброшенными.
        """
        raise NotImplementedError
