from __future__ import annotations

from platform import machine
import time
import threading
import math
from os import PathLike
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Dict, Iterable, Mapping, Optional, Callable

import numpy as np
from pymavlink import mavutil

from multi_drone_core.backend.base_backend import BaseBackend
from multi_drone_core.backend.mavlink.connection import mavlink_connection

if TYPE_CHECKING:
    
    from pymavlink.dialects.v10.ardupilotmega import MAVLink as MAVLink1
    from pymavlink.dialects.v10.ardupilotmega import MAVLink_message as MAVLink_message1 
    from pymavlink.dialects.v10.ardupilotmega import(
        MAVLink_param_value_message
    )
    # from pymavlink.dialects.v20.common import MAVLink as MAVLink2
    # from pymavlink.dialects.v20.common import MAVLink_message as MAVLink_message2
    
    from multi_drone_core.controllers.base_controller import BaseController
    
    
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
    
    # --- Параметры локального обновления состояния ---
    
    # Частота обновления локального состояния машины (сек)
    local_position_update_period_s: float = 0.2
    
    # --- MAVLink heartbeat параметры ---
    
    heartbeat_period_s: float = 0.1
    
    # Тип MAVLink для heartbeat
    heartbeat_type: int = mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER
    
    # Тип автопилота для heartbeat
    heartbeat_autopilot: int = mavutil.mavlink.MAV_AUTOPILOT_INVALID


class MavMode(str, Enum):
    manual = "MANUAL"
    stabilized = "STABILIZED"
    acro = "ACRO"
    rattitude = "RATTITUDE"
    altctl = "ALTCTL"
    posctl = "POSCTL"
    loiter = "LOITER"
    mission = "MISSION"
    rtl = "RTL"
    land = "LAND"
    rtgs = "RTGS"
    followme = "FOLLOWME"
    offboard = "OFFBOARD"
    takeoff = "TAKEOFF"    


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

        self._mavlink_connect = None
        self._mavlink: "MAVLink1" = None
        self._connected = False
        
        # self._vehicle_status_local = VehicleStatusLocal()
        
        self._buffer_lock = threading.Lock()
        # TODO сделать не словарь а класс с конкретными полями для каждого типа сообщений, которые нам нужны
        self._message_buffer: Dict[str, Any] = {}

        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._local_position_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and self.ping())

    def connect(self) -> None:
        if self._connected:
            return
        try:
            self._mavlink_connect = mavlink_connection(
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
            heartbeat = self._mavlink_connect.wait_heartbeat(timeout=heartbeat_timeout)
            if heartbeat is None:
                raise TimeoutError("No heartbeat received from MAVLink target.")
            
            self._mavlink = self._mavlink_connect.mav
            
            self._target_system = self._mavlink_connect.target_system
            self._target_component = self._mavlink_connect.target_component
            
            self.log_info(
                f"MAVLink connected: device={self._config.connect_device}, "
                f"target={self._mavlink_connect.target_system}:{self._mavlink_connect.target_component}"
            )
            
            self._start_background_workers()

            self._connected = True            

        except Exception:
            if self._mavlink_connect is not None:
                try:
                    self._mavlink_connect.close()
                except Exception:
                    pass
                self._mavlink_connect = None
            self._connected = False
            raise

    def disconnect(self) -> None:
        self._stop_background_workers()
        
        if not self._connected and self._mavlink_connect is None:
            return
        
        self._mavlink = None

        if self._mavlink_connect is not None:
            try:
                self._mavlink_connect.close()
            except Exception as exc:
                self.log_warning(f"MAVLink close warning: {exc}")
            finally:
                self._mavlink_connect = None
        
        with self._buffer_lock:
            self._message_buffer.clear()
            
        self._connected = False
        self.log_info("MAVLink disconnected")

    def send_command(
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
        # https://mavlink.io/en/mavgen_python/howto_requestmessages.html
        return self._send_long_command(
            command=command,
            param1=param1,
            param2=param2,
            param3=param3,
            param4=param4,
            param5=param5,
            param6=param6,
            param7=param7,
            target_system=target_system,
            target_component=target_component,
            confirmation=confirmation,
        )

    def _start_background_workers(self) -> None:
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._read_messages_loop,
            name="mavlink-reader-thread",
            daemon=True,
        )
        self._local_position_thread = threading.Thread(
            target=self._machine_local_position_update_loop,
            name="machine-local-position-thread",
            daemon=True,
        )
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="mavlink-heartbeat-thread",
            daemon=True,
        )
        
        self._reader_thread.start()
        self._local_position_thread.start()

    def _stop_background_workers(self) -> None:
        self._stop_event.set()

        for thread in (
            self._reader_thread, 
            self._local_position_thread,
            self._heartbeat_thread,
        ):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)

        self._reader_thread = None
        self._local_position_thread = None

    def _read_messages_loop(self) -> None:
        # https://mavlink.io/en/mavgen_python/
        while not self._stop_event.is_set():
            try:
                msg: "MAVLink_message1" = self._mavlink_connect.recv_match(
                    type=self._config.recv_match_type,
                    condition=self._config.recv_match_condition,
                    blocking=self._config.recv_match_blocking,
                    timeout=self._config.recv_match_timeout,
                )

                if msg is None:
                    continue

                msg_type = msg.get_type()
                
                if msg_type != "BAD_DATA":
                    self.controller.machine_system_data.update_from_msg(
                        msg_type=msg_type,
                        msg=msg,
                    )

            except Exception as exc:
                self.log_warning(f"MAVLink reader error: {exc}")

    def _machine_local_position_update_loop(self) -> None:
        period = max(0.01, float(self._config.local_position_update_period_s))

        while not self._stop_event.is_set():
            try:
                with self._buffer_lock:
                    x = self.controller.machine_system_data.local_position_ned.x
                    y = self.controller.machine_system_data.local_position_ned.y
                    z = self.controller.machine_system_data.local_position_ned.z
                    
                    vx = self.controller.machine_system_data.local_position_ned.vx
                    vy = self.controller.machine_system_data.local_position_ned.vy
                    vz = self.controller.machine_system_data.local_position_ned.vz
                    
                    q1 = self.controller.machine_system_data.attitude_quaternion.q1
                    q2 = self.controller.machine_system_data.attitude_quaternion.q2
                    q3 = self.controller.machine_system_data.attitude_quaternion.q3
                    q4 = self.controller.machine_system_data.attitude_quaternion.q4
                    
                if not any(math.isnan(v) for v in (x, y, z, vx, vy, vz)):
                    self.controller.current_state.update_position(
                        np.array([x, y, z], dtype=float),
                        system="local_NED",
                    )
                    self.controller.current_state.update_velocity(
                        np.array([vx, vy, vz], dtype=float),
                        system="local_NED",
                    )
                
                if not any(math.isnan(v) for v in (q1, q2, q3, q4)):
                    # MAVLink ATTITUDE_QUATERNION: q1=w, q2=x, q3=y, q4=z
                    q = np.array([q2, q3, q4, q1], dtype=float)
                    self.controller.current_state.update_orientation(q, system="local_NED")

            except Exception as exc:
                self.log_warning(f"MAVLink local update error: {exc}")

            self._stop_event.wait(period)
            
    def _heartbeat_loop(self) -> None:
        period = max(0.01, float(self._config.heartbeat_period_s))

        while not self._stop_event.is_set():
            try:
                self._heartbeat_send()
            except Exception as exc:
                self.log_warning(f"MAVLink heartbeat error: {exc}")

            self._stop_event.wait(period)
    
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
        if self._mavlink_connect is None or not self._connected:
            raise RuntimeError("MAVLink backend is not connected. Call connect() first.")

        ts = int(target_system if target_system is not None else self._mavlink_connect.target_system)
        tc = int(target_component if target_component is not None else self._mavlink_connect.target_component)

        self._mavlink.command_long_send(
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
        response = self._mavlink_connect.recv_match(type='COMMAND_ACK', blocking=True)
        self.log_info(
            f"Sent COMMAND_LONG: command={command}, target={ts}:{tc}, "
            f"params=[{param1}, {param2}, {param3}, {param4}, {param5}, {param6}, {param7}], "
            f"confirmation={confirmation}"
        )
        return response

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
        if self._mavlink_connect is None or not self._connected:
            raise RuntimeError("MAVLink backend is not connected. Call connect() first.")

        ts = int(target_system if target_system is not None else self._mavlink_connect.target_system)
        tc = int(target_component if target_component is not None else self._mavlink_connect.target_component)

        self._mavlink.command_int_send(
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
        
        response = self._mavlink_connect.recv_match(type='COMMAND_ACK', blocking=True)
        
        self.log_info(
            f"Sent COMMAND_INT: command={command}, frame={frame}, target={ts}:{tc}, "
            f"params=[{param1}, {param2}, {param3}, {param4}], "
            f"current={current}, autocontinue={autocontinue}, "
            f"x={x}, y={y}, z={z}"
        )

        return response 
    
    def _heartbeat_send(self) -> None:
        # https://mavlink.io/en/services/heartbeat.html
        self._mavlink.heartbeat_send(
            self._config.heartbeat_type,
            self._config.heartbeat_autopilot,
             0, 0, 0
        )
        
    def _set_mode_send(self, mode: MavMode) -> None:
        mode_id = self._mavlink_connect.mode_mapping().get(mode)
        if mode_id is None:
            raise ValueError(f"Unknown mode '{mode}' for current MAVLink target.")
        
        self._mavlink.set_mode_send(
            self._mavlink_connect.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
    
    def _set_mode(self, mode: MavMode) -> None:
        self._mavlink_connect.set_mode(mode)
        
    def set_parameter(
        self,
        parm_name: str,
        parm_value: float,
        parm_type: int = None
    ) -> None:
        self._mavlink_connect.param_set_send(
            parm_name=parm_name,
            parm_value=parm_value,
            parm_type=parm_type
        )
        
    def get_parameter(self, parm_name: int | str) -> Optional[float]:
        self._mavlink_connect.param_fetch_one(parm_name)
        msg: "MAVLink_param_value_message" = self._mavlink_connect.recv_match(type='PARAM_VALUE', blocking=True, timeout=2.0)
        if msg is not None:
            return msg.param_value
        return None
    
    def get_all_parameters(self, timeout: float = 5.0) -> Dict[str, float]:
        params: Dict[str, float] = {}

        self._mavlink_connect.param_fetch_all()

        start_time = time.time()
        expected_count = None

        while True:
            if time.time() - start_time > timeout:
                break

            msg: "MAVLink_param_value_message" = self._mavlink_connect.recv_match(
                type="PARAM_VALUE",
                blocking=True,
                timeout=1.0
            )

            if msg is None:
                continue

            name = msg.param_id.rstrip(b"\x00").decode("ascii")
            params[name] = msg.param_value

            if expected_count is None:
                expected_count = msg.param_count

            if expected_count is not None and len(params) >= expected_count:
                break

        return params

    def log_error(self, message: str) -> None:
        self.logger.error(message)

    def log_warning(self, message: str) -> None:
        self.logger.warning(message)

    def log_info(self, message: str) -> None:
        self.logger.info(message)
