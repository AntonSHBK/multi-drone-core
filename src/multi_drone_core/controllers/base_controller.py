from __future__ import annotations

from pathlib import Path
from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Deque, Dict, Mapping, Optional

import numpy as np

from multi_drone_core.backend.base_backend import BaseBackend
from multi_drone_core.controllers.machine_system_data import MachineSystemData
from multi_drone_core.controllers.base_data import OrientationData, PositionData
from multi_drone_core.controllers.position_transformer import DroneLocalityState
from multi_drone_core.utils.logger import get_core_loggers



class BaseController(ABC):
    """
    Базовый контроллер ядра.
    """

    def __init__(
        self,
        machine_id: int,
        machine_type: str = 'default',
        world_position_enu: np.ndarray = [0.0, 0.0, 0.0],
        world_orientation_rpy: np.ndarray = [0.0, 0.0, 0.0],
        max_command_queue: int = 128,
        log_dir: Path = Path("logs"),
        log_level: str = "INFO",
    ) -> None:
        """
        Инициализация базового контроллера.
        """
        self.machine_id = int(machine_id)
        self.machine_type = str(machine_type)
        self.machine_name = f"machine_{self.machine_id}"
        
        self.loggers = get_core_loggers(
            node_id=self.machine_name,
            log_dir=log_dir,
            log_level=log_level,
        )

        default_position = np.asarray( world_position_enu, dtype=float)
        default_orientation = np.asarray(world_orientation_rpy, dtype=float)

        self.world_position_enu = PositionData(
            x=default_position[0],
            y=default_position[1],
            z=default_position[2],
        )
        self.world_orientation_enu = OrientationData(
            roll=default_orientation[0],
            pitch=default_orientation[1],
            yaw=default_orientation[2],
        )

        self.current_state = DroneLocalityState(
            world_position=self.world_position_enu,
            world_orientation=self.world_orientation_enu,
        )
        self.target_state = DroneLocalityState(
            world_position=self.world_position_enu,
            world_orientation=self.world_orientation_enu,
        )
        
        self.machine_system_data = MachineSystemData()

        self._running = False
        self._backend: BaseBackend = None
        self._command_queue: Deque[Dict[str, Any]] = deque(maxlen=max_command_queue)

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def backend(self) -> BaseBackend:
        return self._backend

    def set_backend(self, backend: BaseBackend) -> None:
        self._backend = backend

    def start(self) -> None:
        self._backend.connect()
        self._running = True

    def stop(self) -> None:
        self._backend.disconnect()
        self._running = False