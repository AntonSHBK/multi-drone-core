from __future__ import annotations

import sys
import time
from pathlib import Path


def _bootstrap_local_path() -> None:
    core_src = Path(__file__).resolve().parents[2] / "src"
    if core_src.exists():
        core_src_str = str(core_src)
        if core_src_str not in sys.path:
            sys.path.insert(0, core_src_str)


_bootstrap_local_path()

from multi_drone_core.backend.mavlink.handler import MavlinkBackend, MavlinkBackendConfig
from multi_drone_core.commands.m_command import M10_Arm, M11_Disarm, M23_Offboard, M24_HoldLoiter
from multi_drone_core.controllers.common_controller import CommonController


def main() -> None:
    controller = CommonController(
        machine_id=1,
    )

    config = MavlinkBackendConfig()
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

    try:
        print("M23: switch to OFFBOARD...")
        controller.commander.process_new_command(M23_Offboard(counter=1))
        print(f"Current mode: {controller.get_mode()}")
        time.sleep(5.0)
        
        backend.get_status()
        backend.get_preflight_parameters()

        print("M10: ARM...")
        controller.commander.process_new_command(M10_Arm(counter=1))
        print("Waiting 5 seconds...")
        time.sleep(5.0)

        print("M11: DISARM...")
        controller.commander.process_new_command(M11_Disarm(counter=1))

        print("M24: switch to HOLD/LOITER...")
        controller.commander.process_new_command(M24_HoldLoiter(counter=1))
        print(f"Current mode: {controller.get_mode()}")
    finally:
        controller.stop()
        print("Controller stopped.")


if __name__ == "__main__":
    main()
