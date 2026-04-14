import threading
from typing import Optional, Dict, Type
from abc import ABC, abstractmethod

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class VehicleState(ABC):
    def __init__(self, controller: "BaseController"):
        self.controller = controller
        self.logger = self.controller.loggers.controller

    @abstractmethod
    def enter(self):
        pass

    @abstractmethod
    def handle(self):
        pass

    @abstractmethod
    def exit(self):
        pass


class HoldState(VehicleState):
    name = "HOLD"
    
    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class ManualState(VehicleState):
    name = "MANUAL"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class OffboardState(VehicleState):
    name = "OFFBOARD"

    def enter(self):
        self.controller.offboard_start()

    def handle(self):
        pass

    def exit(self):
        self.controller.offboard_stop()


class StabilizedState(VehicleState):
    name = "STABILIZED"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class RattitudeState(VehicleState):
    name = "RATTITUDE"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class FollowMeState(VehicleState):
    name = "AUTO_FOLLOW_TARGET"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class AltCtlState(VehicleState):
    name = "ALTCTL"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class PosCtlState(VehicleState):
    name = "POSCTL"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class MissionState(VehicleState):
    name = "AUTO_MISSION"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class AcroState(VehicleState):
    name = "ACRO"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class RTLState(VehicleState):
    name = "AUTO_RTL"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class TakeoffState(VehicleState):
    name = "AUTO_TAKEOFF"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class LandState(VehicleState):
    name = "AUTO_LAND"

    def enter(self):
        pass

    def handle(self):
        pass

    def exit(self):
        pass


class MachineStateMonitor:
    def __init__(
        self,
        controller: "BaseController",
        state_monitor_time_period: float = 0.1,
    ) -> None:
        self.controller = controller
        self.logger = self.controller.loggers.controller
        self.state_monitor_time_period = state_monitor_time_period

        self.state_monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self.current_state: Optional[VehicleState] = None
        self.current_state_name: str = "UNKNOWN"
        self.current_mode_name: str = "UNKNOWN"

        self._state_classes: Dict[str, Type[VehicleState]] = {
            "MANUAL": ManualState,
            "STABILIZED": StabilizedState,
            "STAB": StabilizedState,
            "ACRO": AcroState,
            "RATTITUDE": RattitudeState,
            "ALTCTL": AltCtlState,
            "POSCTL": PosCtlState,
            "LOITER": HoldState,
            "AUTO_LOITER": HoldState,
            "MISSION": MissionState,
            "AUTO_MISSION": MissionState,
            "RTL": RTLState,
            "AUTO_RTL": RTLState,
            "LAND": LandState,
            "AUTO_LAND": LandState,
            "FOLLOWME": FollowMeState,
            "AUTO_FOLLOW_TARGET": FollowMeState,
            "OFFBOARD": OffboardState,
            "TAKEOFF": TakeoffState,
            "AUTO_TAKEOFF": TakeoffState,
            "UNKNOWN": HoldState,
        }
        self._states: Dict[Type[VehicleState], VehicleState] = {}
        self._fallback_state_type = HoldState

    def start(self) -> None:
        self._stop_event.clear()

        self.state_monitor_thread = threading.Thread(
            target=self._state_monitor_loop,
            name="machine-state-monitor-thread",
            daemon=True,
        )
        self.state_monitor_thread.start()

    def stop(self) -> None:
        self._stop_event.set()

        thread = self.state_monitor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

        self.state_monitor_thread = None

    def _state_monitor_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.state_monitor()
            except Exception as exc:
                self.logger.exception(f"ошибка мониторинга состояния: {exc}")

            self._stop_event.wait(self.state_monitor_time_period)

    def _resolve_state(self, mode_name: str) -> VehicleState:
        state_type = self._state_classes.get(mode_name, self._fallback_state_type)
        state = self._states.get(state_type)
        if state is None:
            state = state_type(self.controller)
            self._states[state_type] = state
        return state

    def state_monitor(self) -> None:
        detected_mode_name = self.controller.get_mode()
        
        if detected_mode_name is None:
            detected_mode_name = "UNKNOWN"

        with self._lock:
            previous_state = self.current_state
            if previous_state is None:  
                next_state = self._resolve_state(detected_mode_name)              
                self.current_state = next_state
                self.current_state_name = next_state.name
                self.current_mode_name = detected_mode_name
                next_state.enter()
                self.logger.info(
                    f"Инициализация состояния: {self.current_state_name} (режим={self.current_mode_name})"
                )
                return

            if self.current_mode_name == detected_mode_name:
                previous_state.handle()
                return
            
            else:            
                next_state = self._resolve_state(detected_mode_name)
                previous_mode_name = self.current_mode_name
                previous_state.exit()
                self.current_state = next_state
                self.current_state_name = next_state.name
                self.current_mode_name = detected_mode_name
            self.current_state.enter()
            self.logger.info(
                "Состояние изменено: "
                f"{previous_state.name}/{previous_mode_name} -> "
                f"{self.current_state.name}/{detected_mode_name}"
            )
