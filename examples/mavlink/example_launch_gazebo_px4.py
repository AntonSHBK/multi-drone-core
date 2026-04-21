from __future__ import annotations

import os
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

from multi_drone_core.scripts.gazebo_server import start_gazebo_simulation
from multi_drone_core.scripts.px4 import PX4Process


def main() -> None:
    world_name = "default"
    px4_dir = os.getenv("PX4_DIR", "/workspace/src/PX4-Autopilot")

    print("Starting Gazebo...")
    gazebo_process = start_gazebo_simulation(
        world=world_name,
        headless=False,
    )

    time.sleep(3.0)

    print("Starting PX4...")
    px4 = PX4Process(
        drone_id=1,
        gz_world=world_name,
        px4_dir=px4_dir,
        standalone=True,
    )
    px4_process = px4.run(terminal="bash")

    print("Gazebo and PX4 are running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping processes...")
    finally:
        if px4_process.poll() is None:
            px4_process.terminate()
            px4_process.wait(timeout=10)

        if gazebo_process.poll() is None:
            gazebo_process.terminate()
            gazebo_process.wait(timeout=10)

        print("Done.")


if __name__ == "__main__":
    main()
