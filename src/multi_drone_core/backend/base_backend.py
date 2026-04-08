from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, Mapping, Optional, Protocol, TypedDict

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController

class BaseBackend(ABC):
    """
    Базовый интерфейс backend-адаптера для конкретного транспорта (ROS2/MAVLink).
    """

    def __init__(
        self, 
        controller: 'BaseController',
        backend_type: str = 'default',
    ) -> None:
        self._controller = controller
        self.logger = controller.loggers.backend
        self._backend_type = backend_type

    @property
    def controller(self) -> 'BaseController':
        return self._controller
    
    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """
        Текущее состояние соединения с автопилотом.
        """

    @abstractmethod
    def connect(self) -> None:
        """
        Инициализация и запуск backend (подписки, порты, сессии, heartbeat и т.п.).
        """

    @abstractmethod
    def disconnect(self) -> None:
        """
        Корректное отключение backend и освобождение ресурсов.
        """

    @abstractmethod
    def send_command(self) -> Any:
        """
        Отправка унифицированной команды на дрон.
        """

    # @abstractmethod
    # def set_parameter(self) -> Any:
    #     """
    #     Установка одного параметра автопилота.
    #     """

    # @abstractmethod
    # def set_parameters(self) -> Any:
    #     """
    #     Установка нескольких параметров автопилота.
    #     """

    # @abstractmethod
    # def get_parameter(self) -> Any:
    #     """
    #     Чтение одного параметра автопилота.
    #     """
    
    # @abstractmethod
    # def get_parameters(self) -> Any:
    #     """
    #     Чтение нескольких параметров автопилота.
    #     """