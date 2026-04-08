from __future__ import annotations

import time
import threading
import math
from os import PathLike
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Dict, Iterable, Mapping, Optional, Callable

import numpy as np
from pymavlink import mavutil

from multi_drone_core.backend.base_backend import BaseBackend
from multi_drone_core.backend.mavlink.connection import mavlink_connection

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class VehicleStatusLocal:
    def __init__(self) -> None:
        self.connected: bool = False
        self.armed: bool = False
        self.custom_mode: int = 0
        self.system_status: int = 0
        self.failsafe: bool = False
        self.battery_voltage_v: float = 0.0
        self.battery_current_a: float = 0.0
        self.battery_remaining_pct: int = -1
        self.last_heartbeat_time: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "armed": self.armed,
            "custom_mode": self.custom_mode,
            "system_status": self.system_status,
            "failsafe": self.failsafe,
            "battery_voltage_v": self.battery_voltage_v,
            "battery_current_a": self.battery_current_a,
            "battery_remaining_pct": self.battery_remaining_pct,
            "last_heartbeat_time": self.last_heartbeat_time,
        }
    
    def reset(self):
        self.__init__()
        
    def __repr__(self):
        return (
            f"VehicleStatusLocal(connected={self.connected}, armed={self.armed}, "
            f"custom_mode={self.custom_mode}, failsafe={self.failsafe}, "
            f"battery={self.battery_voltage_v:.2f}V/{self.battery_remaining_pct}%)"
        )
    
@dataclass
class MavlinkBackendConfig:
    
    connect_device: str | PathLike[str] = "/dev/ttyS3"
    connect_baudrate: int = 57600

    # MAVLink идентификация
    connect_source_system: int = 255
    connect_source_component: int = 0

    # Логи / формат
    connect_planner_format: str | None = None
    connect_write: bool = False
    connect_append: bool = False

    # Парсинг
    connect_robust_parsing: bool = True
    connect_notimestamps: bool = False

    # Направление / транспорт
    connect_input: bool = True

    # MAVLink dialect
    connect_dialect: str | None = None

    # Поведение соединения
    connect_autoreconnect: bool = False
    connect_zero_time_base: bool = False
    connect_retries: int = 3
    connect_use_native: bool = mavutil.default_native
    connect_force_connected: bool = False

    # Callback и таймауты
    connect_progress_callback: Callable[..., Any] | None = None
    connect_udp_timeout: float = 0.0

    # Дополнительные параметры соединения
    connect_opts: dict[str, Any] = field(default_factory=dict)
    
    # --- Параметры recv_match ---

    # Фильтр по типам MAVLink сообщений (например: "HEARTBEAT" или ["ATTITUDE", "LOCAL_POSITION_NED"])
    recv_match_type: str | Iterable[str] | None = None

    # Условие фильтрации (строка, вычисляемая относительно self.messages)
    recv_match_condition: str | None = None

    # Блокирующий режим ожидания сообщения
    recv_match_blocking: bool = True

    # Таймаут ожидания сообщения (в секундах)
    recv_match_timeout: float | None = 0.5
    
    # Частота применения буфера к core-состоянию (сек)
    local_update_period_s: float = 0.1


class MavlinkBackend(BaseBackend):
    """
    Backend-адаптер для прямого подключения к автопилоту по MAVLink.
    """

    def __init__(
        self,
        controller: "BaseController",
        backend_type: str = "mavlink",
        config: MavlinkBackendConfig = MavlinkBackendConfig(),
    ) -> None:
        super().__init__(controller=controller, backend_type=backend_type)

        self._config = config    

        self._master = None
        self._connected = False
        
        self._vehicle_status_local = VehicleStatusLocal()
        
        self._buffer_lock = threading.Lock()
        # TODO сделать не словарь а класс с конкретными полями для каждого типа сообщений, которые нам нужны
        self._message_buffer: Dict[str, Any] = {}

        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._local_position_thread: threading.Thread | None = None
        # self._heartbeat_thread: threading.Thread | None = None
        # self._vehicle_status_thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and self.ping())

    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._master = mavlink_connection(
                device=self._config.connect_device,
                baud=self._config.connect_baudrate,
                source_system=self._config.connect_source_system,
                source_component=self._config.connect_source_component,
                planner_format=self._config.connect_planner_format,
                write=self._config.connect_write,
                append=self._config.connect_append,
                robust_parsing=self._config.connect_robust_parsing,
                notimestamps=self._config.connect_notimestamps,
                input=self._config.connect_input,
                dialect=self._config.connect_dialect,
                autoreconnect=self._config.connect_autoreconnect,
                zero_time_base=self._config.connect_zero_time_base,
                retries=self._config.connect_retries,
                use_native=self._config.connect_use_native,
                force_connected=self._config.connect_force_connected,
                progress_callback=self._config.connect_progress_callback,
                udp_timeout=self._config.connect_udp_timeout,
                **self._config.connect_opts,
            )

            heartbeat_timeout = max(1.0, float(self._config.connect_udp_timeout or 3.0))
            heartbeat = self._master.wait_heartbeat(timeout=heartbeat_timeout)
            if heartbeat is None:
                raise TimeoutError("No heartbeat received from MAVLink target.")
            
            self.log_info(
                f"MAVLink connected: device={self._config.connect_device}, "
                f"target={self._master.target_system}:{self._master.target_component}"
            )
            
            self._start_background_workers()

            self._connected = True            

        except Exception:
            if self._master is not None:
                try:
                    self._master.close()
                except Exception:
                    pass
                self._master = None
            self._connected = False
            raise

    def disconnect(self) -> None:
        self._stop_background_workers()
        
        if not self._connected and self._master is None:
            return

        if self._master is not None:
            try:
                self._master.close()
            except Exception as exc:
                self.log_warning(f"MAVLink close warning: {exc}")
            finally:
                self._master = None
        self._vehicle_status_local.reset()
        
        with self._buffer_lock:
            self._message_buffer.clear()
            
        self._connected = False
        self.log_info("MAVLink disconnected")

    def send_command(self) -> Any:
        pass

    def _send_long_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        target_system: Optional[int] = None,
        target_component: Optional[int] = None,
        confirmation: int = 0,
    ) -> Any:
        """
        Отправляет MAVLink-команду через COMMAND_LONG.

        Parameters
        ----------
        command : int
            Идентификатор MAV_CMD_*.
        param1..param7 : float
            Параметры команды (назначение зависит от command).
        target_system : int | None
            Целевой system id. Если None, используется target из heartbeat.
        target_component : int | None
            Целевой component id. Если None, используется target из heartbeat.
        confirmation : int
            Поле confirmation для повторной отправки/идемпотентности.
        """
        if self._master is None or not self._connected:
            raise RuntimeError("MAVLink backend is not connected. Call connect() first.")

        ts = int(target_system if target_system is not None else self._master.target_system)
        tc = int(target_component if target_component is not None else self._master.target_component)

        self._master.mav.command_long_send(
            ts,
            tc,
            int(command),
            int(confirmation),
            float(param1),
            float(param2),
            float(param3),
            float(param4),
            float(param5),
            float(param6),
            float(param7),
        )
        self.log_info(
            f"Sent COMMAND_LONG: command={command}, target={ts}:{tc}, "
            f"params=[{param1}, {param2}, {param3}, {param4}, {param5}, {param6}, {param7}], "
            f"confirmation={confirmation}"
        )
        return 

    def _send_int_command(
        self,
        command: int,
        *,
        frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        x: int = 0,
        y: int = 0,
        z: float = 0.0,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        current: int = 0,
        autocontinue: int = 0,
        target_system: Optional[int] = None,
        target_component: Optional[int] = None,
    ) -> Any:
        """
        Отправляет MAVLink-команду через COMMAND_INT.

        Parameters
        ----------
        command : int
            Идентификатор MAV_CMD_*.
        frame : int
            Система координат MAV_FRAME_*.
        x : int
            Обычно lat * 1e7 (для GLOBAL_INT кадров) или x в локальной системе.
        y : int
            Обычно lon * 1e7 (для GLOBAL_INT кадров) или y в локальной системе.
        z : float
            Высота/координата z.
        param1..param4 : float
            Доп. параметры команды.
        current : int
            Флаг current (совместимость с mission-полями).
        autocontinue : int
            Флаг autocontinue (совместимость с mission-полями).
        target_system : int | None
            Целевой system id. Если None, используется target из heartbeat.
        target_component : int | None
            Целевой component id. Если None, используется target из heartbeat.
        """
        if self._master is None or not self._connected:
            raise RuntimeError("MAVLink backend is not connected. Call connect() first.")

        ts = int(target_system if target_system is not None else self._master.target_system)
        tc = int(target_component if target_component is not None else self._master.target_component)

        self._master.mav.command_int_send(
            ts,
            tc,
            int(frame),
            int(command),
            int(current),
            int(autocontinue),
            float(param1),
            float(param2),
            float(param3),
            float(param4),
            int(x),
            int(y),
            float(z),
        )
        
        self.log_info(
            f"Sent COMMAND_INT: command={command}, frame={frame}, target={ts}:{tc}, "
            f"params=[{param1}, {param2}, {param3}, {param4}], "
            f"current={current}, autocontinue={autocontinue}, "
            f"x={x}, y={y}, z={z}"
        )

        return    
    
    def _start_background_workers(self) -> None:
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._read_messages_loop,
            name="mavlink-reader-thread",
            daemon=True,
        )
        self._local_position_thread = threading.Thread(
            target=self._local_position_update_loop,
            name="mavlink-local-position-thread",
            daemon=True,
        )
        
        self._reader_thread.start()
        self._local_position_thread.start()

    def _stop_background_workers(self) -> None:
        self._stop_event.set()

        for thread in (self._reader_thread, self._local_position_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

        self._reader_thread = None
        self._local_position_thread = None

    def _read_messages_loop(self) -> None:
        interesting_types = {
            "HEARTBEAT",
            "LOCAL_POSITION_NED",
            "ATTITUDE",
            "ATTITUDE_QUATERNION",
            "SYS_STATUS",
            "GLOBAL_POSITION_INT",
        }

        while not self._stop_event.is_set():
            try:
                msg = self._master.recv_match(
                    type=self._config.recv_match_type,
                    condition=self._config.recv_match_condition,
                    blocking=self._config.recv_match_blocking,
                    timeout=self._config.recv_match_timeout,
                )

                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type in interesting_types:
                    with self._buffer_lock:
                        self._message_buffer[msg_type] = msg

            except Exception as exc:
                self.log_warning(f"MAVLink reader error: {exc}")

    def _local_position_update_loop(self) -> None:
        period = max(0.01, float(self._config.local_update_period_s))

        while not self._stop_event.is_set():
            try:
                with self._buffer_lock:
                    snapshot = dict(self._message_buffer)

                # 1) POSITION + VELOCITY
                lpn = snapshot.get("LOCAL_POSITION_NED")
                if lpn is not None:
                    self.controller.current_state.update_position(
                        np.array([lpn.x, lpn.y, lpn.z], dtype=float),
                        system="local_NED",
                    )
                    self.controller.current_state.update_velocity(
                        np.array([lpn.vx, lpn.vy, lpn.vz], dtype=float),
                        system="local_NED",
                    )

                # 2) ORIENTATION (prefer quaternion if available)
                att_q = snapshot.get("ATTITUDE_QUATERNION")
                if att_q is not None:
                    # MAVLink ATTITUDE_QUATERNION: q1=w, q2=x, q3=y, q4=z
                    q = np.array([att_q.q2, att_q.q3, att_q.q4, att_q.q1], dtype=float)
                    self.controller.current_state.update_orientation(q, system="local_NED")
                else:
                    att = snapshot.get("ATTITUDE")
                    # self.controller.current_state.update_orientation(q, system="local_NED")
                    pass

                # 3) STATUS
                hb = snapshot.get("HEARTBEAT")
                if hb is not None:
                #     self._vehicle_status_local.connected = True
                #     self._vehicle_status_local.last_heartbeat_time = time.monotonic()
                #     self._vehicle_status_local.custom_mode = int(getattr(hb, "custom_mode", 0))
                #     self._vehicle_status_local.system_status = int(getattr(hb, "system_status", 0))
                #     self._vehicle_status_local.armed = bool(
                #         int(getattr(hb, "base_mode", 0))
                #         & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                #     )
                    pass

                sys_status = snapshot.get("SYS_STATUS")
                if sys_status is not None:
                    # self._vehicle_status_local.battery_voltage_v = float(
                    #     getattr(sys_status, "voltage_battery", 0)
                    # ) / 1000.0
                    # self._vehicle_status_local.battery_current_a = float(
                    #     getattr(sys_status, "current_battery", 0)
                    # ) / 100.0
                    # self._vehicle_status_local.battery_remaining_pct = int(
                    #     getattr(sys_status, "battery_remaining", -1)
                    # )
                    # self._vehicle_status_local.failsafe = bool(
                    #     int(getattr(sys_status, "errors_count1", 0))
                    #     or int(getattr(sys_status, "errors_count2", 0))
                    # )
                    pass

            except Exception as exc:
                self.log_warning(f"MAVLink local update error: {exc}")

            self._stop_event.wait(period)
    
    def log_error(self, message: str) -> None:
        self.logger.error(message)

    def log_warning(self, message: str) -> None:
        self.logger.warning(message)

    def log_info(self, message: str) -> None:
        self.logger.info(message)
