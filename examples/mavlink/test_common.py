from __future__ import annotations

import sys
from pathlib import Path
import time


def _bootstrap_local_path() -> None:
    core_src = Path(__file__).resolve().parents[2] / "src"
    if core_src.exists():
        core_src_str = str(core_src)
        if core_src_str not in sys.path:
            sys.path.insert(0, core_src_str)


_bootstrap_local_path()

from multi_drone_core.backend.mavlink.handler import MavlinkBackend, MavlinkBackendConfig, MavModes
from multi_drone_core.controllers.common_controller import CommonController

def main() -> None:  

    controller = CommonController(
        machine_id=1,
    )

    config = MavlinkBackendConfig()
    config.connect_device = "udp:127.0.0.1:14540"
    config.connect_baudrate = 115200

    backend = MavlinkBackend(
        controller=controller,
        config=config,
    )
    controller.set_backend(backend)

    print("Controller and backend created.")
    print(f"machine: id={controller.machine_id}, type={controller.machine_type}")
    print(f"mavlink device: {config.connect_device}")
    print("sys.path configured for local repos.")
    
    controller.start()
    print("Controller started.")
    
    controller.fetch_all_parameters()
    
    time.sleep(5)
    
    controller.set_manual_mode()
    print(f"Set mode to {controller.get_mode()}")
    
    time.sleep(5)
    controller.set_stabilized_mode()
    print(f"Set mode to {controller.get_mode()}")
    
    time.sleep(5)
    controller.set_manual_mode()
    print(f"Set mode to {controller.get_mode()}")  
    
    print("Parameters:")
    for name, value in controller.get_all_parameters().items():
        print(f"  {name}: {value}")

        
    controller.stop()
    print("Controller stopped.")

if __name__ == "__main__":
    main()
