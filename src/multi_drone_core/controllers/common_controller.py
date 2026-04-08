from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

import numpy as np


from multi_drone_core.backend.base_backend import BaseBackend
from multi_drone_core.controllers.base_controller import BaseController


class CommonController(BaseController):

    def __init__(
        self,
        machine_id: int,
        machine_type: str = "default",
        world_position_enu: np.ndarray = np.array([0.0, 0.0, 0.0]),
        world_orientation_rpy: np.ndarray = np.array([0.0, 0.0, 0.0]),
        backend: Optional[BaseBackend] = None,
        max_command_queue: int = 128,
    ) -> None:
        super().__init__(
            machine_id=machine_id,
            machine_type=machine_type,
            world_position_enu=world_position_enu,
            world_orientation_rpy=world_orientation_rpy,
            max_command_queue=max_command_queue,
        )
        self._logger = self.loggers.controller
        if backend is not None:
            self.set_backend(backend)      
        

        self._logger.info("CommonController initialized")


if __name__ == "__main__":
    # Пример создания контроллера и backend
    controller = CommonController(machine_id=1)
    