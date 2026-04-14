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
from multi_drone_core.backend.mavlink.offboard import OffboardCommander

if TYPE_CHECKING:    
    from pymavlink.dialects.v10.ardupilotmega import MAVLink as MAVLink1
    from pymavlink.dialects.v10.ardupilotmega import MAVLink_message as MAVLink_message1 
    from pymavlink.dialects.v10.ardupilotmega import (
        MAVLink_message,
        MAVLink_heartbeat_message,
        MAVLink_sys_status_message,
        MAVLink_extended_sys_state_message,
        MAVLink_local_position_ned_message,
        MAVLink_position_target_local_ned_message,
        MAVLink_attitude_message,
        MAVLink_attitude_quaternion_message,
        MAVLink_attitude_target_message,
        MAVLink_estimator_status_message,
        MAVLink_scaled_pressure_message,
        MAVLink_mission_current_message,
        MAVLink_ping_message,
        MAVLink_highres_imu_message,
        MAVLink_vfr_hud_message,
        MAVLink_gps_raw_int_message,
        MAVLink_param_value_message
    )
    from pymavlink.dialects.v20.common import MAVLink as MAVLink2
    # from pymavlink.dialects.v20.common import MAVLink_message as MAVLink_message2
    
    from multi_drone_core.controllers.base_controller import BaseController
    from multi_drone_core.controllers.position_transformer import CoordinateSystem
    

    
    
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
    
    # # --- Параметры recv_match ---

    # # Фильтр по типам MAVLink сообщений (например: "HEARTBEAT" или ["ATTITUDE", "LOCAL_POSITION_NED"])
    # recv_match_type: str | Iterable[str] | None = None

    # # Условие фильтрации (строка, вычисляемая относительно self.messages)
    # recv_match_condition: str | None = None

    # # Блокирующий режим ожидания сообщения
    # recv_match_blocking: bool = False

    # # Таймаут ожидания сообщения (в секундах)
    # recv_match_timeout: float | None = None
    
    # --- Параметры локального обновления состояния ---
    
    # Частота обновления локального состояния машины (сек)
    local_position_update_period_s: float = 0.2
    
    # --- MAVLink heartbeat параметры ---
    
    heartbeat_period_s: float = 0.1
    
    # Тип MAVLink для heartbeat
    heartbeat_type: int = mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER
    
    # Тип автопилота для heartbeat
    heartbeat_autopilot: int = mavutil.mavlink.MAV_AUTOPILOT_INVALID

    # Offboard setpoint send frequency (Hz), 5.0
    offboard_setpoint_rate_hz: float = 10.0


class MavModes(str, Enum):
    """
    Перечисление режимов полёта PX4, используемых при переключении через MAVLink.

    manual
        Полностью ручной режим. Управление осуществляется напрямую стиками,
        без стабилизации по позиции и высоте. Минимальная автоматизация.

    stabilized
        Режим стабилизации. Автопилот стабилизирует углы (roll/pitch),
        но управление остаётся ручным. Высота и позиция не удерживаются автоматически.

    acro
        Акробатический режим. Управление по угловым скоростям.
        Предназначен для опытных пилотов и агрессивных манёвров.

    rattitude
        Комбинированный режим: вблизи центра стиков работает как stabilized,
        при больших отклонениях — как acro.

    altctl
        Altitude Control. Удержание высоты по барометру/датчикам.
        Горизонтальное перемещение управляется вручную.

    posctl
        Position Control. Удержание позиции и высоты.
        При отпускании стиков дрон зависает в текущей точке.
        Полуавтоматический режим с ручным управлением.

    loiter
        Автоматический режим удержания позиции (Hold).
        Дрон автономно удерживает текущую позицию без участия оператора.
        Используется как безопасный режим ожидания.

    mission
        Выполнение загруженной миссии (AUTO_MISSION).
        Автопилот самостоятельно проходит последовательность waypoint.

    rtl
        Return To Launch. Автоматическое возвращение к точке взлёта
        с последующей посадкой или зависанием.

    land
        Автоматическая посадка в текущей точке.

    rtgs
        Return To Ground Station. Возврат к наземной станции (если поддерживается конфигурацией).

    followme
        Режим следования за движущейся целью (например, за оператором с GPS).

    offboard
        Внешнее управление. Дрон управляется внешним компьютером через
        поток setpoint (MAVLink или ROS). Требует постоянного обновления команд.

    takeoff
        Автоматический взлёт на заданную высоту (AUTO_TAKEOFF).
        После завершения обычно переходит в loiter или mission.
    """

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
    unknown = "UNKNOWN"
    

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
        
        self._buffer_lock = threading.Lock()
        # TODO сделать не словарь а класс с конкретными полями для каждого типа сообщений, которые нам нужны
        self._message_buffer: Dict[str, Any] = {}

        self._stop_event = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._local_position_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        
        self.offboard_commander = OffboardCommander(self)

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and self._mavlink_connect is not None)
    
    def log_error(self, message: str) -> None:
        self.logger.error(message)

    def log_warning(self, message: str) -> None:
        self.logger.warning(message)

    def log_info(self, message: str) -> None:
        self.logger.info(message)

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

            if not self.wait_ready():
                raise TimeoutError("MAVLink backend is not ready: timed out waiting for telemetry/parameters.")

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
    ) -> None:
        target_system = target_system or self._target_system
        target_component = target_component or self._target_component

        self._mavlink.command_long_send(
            target_system,
            target_component,
            command,
            confirmation,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
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
        self._heartbeat_thread.start()
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
        self._heartbeat_thread = None
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

            except Exception as exc:
                self.log_warning(f"MAVLink reader error: {exc}")
                self._stop_event.wait(0.05)

    def wait_ready(
        self,
        timeout: float = 15.0,
        poll_period_s: float = 0.5,
    ) -> bool:
        timeout = max(0.1, float(timeout))
        poll_period_s = max(0.01, float(poll_period_s))

        self.fetch_all_parameters()

        required_messages = (
            "HEARTBEAT",
            "LOCAL_POSITION_NED",
            "ATTITUDE_QUATERNION",
            "HIGHRES_IMU",
        )

        deadline = time.time() + timeout

        while time.time() < deadline and not self._stop_event.is_set():
            messages: dict = self._mavlink_connect.messages
            params: Dict[str, float] = self._mavlink_connect.params

            missing_messages = [
                message_name
                for message_name in required_messages
                if messages.get(message_name) is None
            ]

            param_value_msg = messages.get("PARAM_VALUE")
            expected_param_count = 0
            if param_value_msg is not None:
                try:
                    expected_param_count = max(0, int(getattr(param_value_msg, "param_count", 0)))
                except Exception:
                    expected_param_count = 0

            params_ready = expected_param_count > 0 and len(params) >= expected_param_count

            if not missing_messages and params_ready:
                self.log_info(
                    f"MAVLink ready: params={len(params)}/{expected_param_count}, "
                    f"messages={','.join(required_messages)}"
                )
                return True

            self._stop_event.wait(poll_period_s)

        messages = self._mavlink_connect.messages
        params = self._mavlink_connect.params
        param_value_msg = messages.get("PARAM_VALUE")
        expected_param_count = 0
        if param_value_msg is not None:
            try:
                expected_param_count = max(0, int(getattr(param_value_msg, "param_count", 0)))
            except Exception:
                expected_param_count = 0

        missing_messages = [
            message_name
            for message_name in required_messages
            if messages.get(message_name) is None
        ]

        self.log_warning(
            "MAVLink wait_ready timeout: "
            f"missing_messages={missing_messages}, "
            f"params={len(params)}/{expected_param_count}"
        )
        return False
    
    def _machine_local_position_update_loop(self) -> None:
        period = max(0.01, float(self._config.local_position_update_period_s))

        while not self._stop_event.is_set():
            try:
                messages: dict = self._mavlink_connect.messages

                local_pos_msg: "MAVLink_local_position_ned_message" = messages.get("LOCAL_POSITION_NED")
                attitude_msg: "MAVLink_attitude_quaternion_message" = messages.get("ATTITUDE_QUATERNION")
                imu_msg: "MAVLink_highres_imu_message" = messages.get("HIGHRES_IMU")

                if local_pos_msg is not None:
                    x = float(local_pos_msg.x)
                    y = float(local_pos_msg.y)
                    z = float(local_pos_msg.z)

                    vx = float(local_pos_msg.vx)
                    vy = float(local_pos_msg.vy)
                    vz = float(local_pos_msg.vz)

                    if not any(math.isnan(v) for v in (x, y, z)):
                        self.controller.current_state.update_position(
                            np.array([x, y, z], dtype=float),
                            system="local_NED",
                        )

                    if not any(math.isnan(v) for v in (vx, vy, vz)):
                        self.controller.current_state.update_velocity(
                            np.array([vx, vy, vz], dtype=float),
                            system="local_NED",
                        )

                if imu_msg is not None:
                    ax = float(imu_msg.xacc)
                    ay = float(imu_msg.yacc)
                    az = float(imu_msg.zacc)

                    if not any(math.isnan(v) for v in (ax, ay, az)):
                        self.controller.current_state.update_acceleration(
                            np.array([ax, ay, az], dtype=float),
                            system="local_NED",
                        )

                if attitude_msg is not None:
                    q1 = float(attitude_msg.q1)
                    q2 = float(attitude_msg.q2)
                    q3 = float(attitude_msg.q3)
                    q4 = float(attitude_msg.q4)

                    if not any(math.isnan(v) for v in (q1, q2, q3, q4)):
                        # MAVLink ATTITUDE_QUATERNION: q1=w, q2=x, q3=y, q4=z
                        q = np.array([q2, q3, q4, q1], dtype=float)
                        self.controller.current_state.update_orientation_quaternion(
                            q,
                            system="local_NED",
                        )

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
    
    def _heartbeat_send(self) -> None:
        # https://mavlink.io/en/services/heartbeat.html
        self._mavlink.heartbeat_send(
            self._config.heartbeat_type,
            self._config.heartbeat_autopilot,
             0, 0, 0
        )
        
    def send_offboard_setpoint(
        self,
        *,
        position: np.ndarray | None = None,
        velocity: np.ndarray | None = None,
        acceleration: np.ndarray | None = None,
        yaw: float | None = None,
        yaw_speed: float | None = None,
        system: "CoordinateSystem" = "global_ENU",
    ) -> None:
        self.offboard_commander.update(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            yaw=yaw,
            yaw_speed=yaw_speed,
            system=system,
        )
    
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
        
    def get_parameter(self, parm_name: str) -> float:
        params = self._mavlink_connect.params

        if parm_name not in params:
            raise ValueError(f"Parameter '{parm_name}' not found")

        return params[parm_name]
    
    def fetch_all_parameters(self):
        self._mavlink_connect.param_fetch_all()
    
    def get_all_parameters(self) -> Dict[str, float]:
        params: Dict[str, float] = self._mavlink_connect.params
        return params.copy()
    
    def set_mode(self, mode: MavModes) -> None:
        self._mavlink_connect.set_mode(mode)
        
    def get_mode(self) -> str:
        heartbeat = self._mavlink_connect.messages.get("HEARTBEAT")
        if heartbeat is None:
            return "UNKNOWN"
        return mavutil.mode_string_v10(heartbeat)
       
    def offboard_start(self):        
        self.offboard_commander.start()
    
    def offboard_stop(self):
        self.offboard_commander.stop()
        
    def check_offboard_mode(self) -> bool:
        mode = self.get_mode()
        return mode == MavModes.offboard.value
    
    def arm(self) -> None:
        self._mavlink_connect.arducopter_arm()
        self._mavlink_connect.motors_armed_wait()

    def disarm(self) -> None:        
        self._mavlink_connect.arducopter_disarm()
        self._mavlink_connect.motors_disarmed_wait()

    def set_offboard_mode(self) -> None:
        self.set_mode(MavModes.offboard)
        complete = self._wait_mode(MavModes.offboard.value)

    def set_manual_mode(self) -> None:
        self.set_mode(MavModes.manual)
        complete = self._wait_mode(MavModes.manual.value)

    def set_loiter_mode(self) -> None:
        self.set_mode(MavModes.loiter)
        complete = self._wait_mode(MavModes.loiter.value)
        
    def set_stabilized_mode(self) -> None:
        self.set_mode(MavModes.stabilized)
        complete = self._wait_mode(MavModes.stabilized.value)

    def set_followme_mode(self) -> None:
        self.set_mode(MavModes.followme)
        complete = self._wait_mode(MavModes.followme.value)
        
    def set_altctl_mode(self) -> None:
        self.set_mode(MavModes.altctl)
        complete = self._wait_mode(MavModes.altctl.value)

    def set_posctl_mode(self) -> None:
        self.set_mode(MavModes.posctl)
        complete = self._wait_mode(MavModes.posctl.value)
        
    def set_mission_mode(self) -> None:
        self.set_mode(MavModes.mission)
        self._wait_mode(MavModes.mission.value)

    def set_acro_mode(self) -> None:
        self.set_mode(MavModes.acro)
        self._wait_mode(MavModes.acro.value)

    def set_rtl_mode(self) -> None:
        self.set_mode(MavModes.rtl)
        complete = self._wait_mode(MavModes.rtl.value)
        
    def _wait_mode(self, target_mode: str, timeout: float = 5.0) -> None:
        start = time.time()

        while time.time() - start < timeout:
            mode = self.get_mode()
            if mode == target_mode:
                return True
            time.sleep(0.05)
            
        return False

    def auto_land(self) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_NAV_LAND,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    def auto_takeoff(self, altitude: float = 2.0) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=altitude,
        )

    def reboot(self) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=1.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    def calibrate_gyro(self) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=1.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    def calibrate_magnetometer(self) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=0.0,
            param2=1.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )

    def calibrate_accelerometer(self, simple: bool = False) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=4.0 if simple else 1.0,
            param6=0.0,
            param7=0.0,
        )

    def calibrate_vehicle_level(self) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=2.0,
            param6=0.0,
            param7=0.0,
        )

    def calibrate_barometer(self) -> None:
        self.send_command(
            command=mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            target_system=self._target_system,
            target_component=self._target_component,
            confirmation=0,
            param1=0.0,
            param2=0.0,
            param3=1.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0,
        )
    
    def get_status(self) -> Dict[str, Any]:
        heartbeat = self._mavlink_connect.messages.get("HEARTBEAT")
        sys_status = self._mavlink_connect.messages.get("SYS_STATUS")

        status: Dict[str, Any] = {}

        if heartbeat is not None:
            status["mode"] = mavutil.mode_string_v10(heartbeat)
            status["armed"] = bool(
                heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            status["system_status"] = heartbeat.system_status
        else:
            status["mode"] = "UNKNOWN"
            status["armed"] = False
            status["system_status"] = None

        if sys_status is not None:
            status["battery_voltage"] = sys_status.voltage_battery
            status["battery_remaining"] = sys_status.battery_remaining
            status["load"] = sys_status.load
            status["sensors_health"] = sys_status.onboard_control_sensors_health
        else:
            status["battery_voltage"] = None
            status["battery_remaining"] = None
            status["load"] = None
            status["sensors_health"] = None

        return status
    
    def get_preflight_parameters(self) -> Dict[str, Dict[str, Optional[float]]]:
        params = self._mavlink_connect.params

        def safe_get(name: str) -> Optional[float]:
            return params.get(name)

        return {
            "core": {
                "COM_ARMABLE": safe_get("COM_ARMABLE"),
                "COM_ARM_PRECHK": safe_get("COM_ARM_PRECHK"),
                "COM_ARM_BAT_MIN": safe_get("COM_ARM_BAT_MIN"),
                "COM_ARM_IMU_ACC": safe_get("COM_ARM_IMU_ACC"),
                "COM_ARM_IMU_GYR": safe_get("COM_ARM_IMU_GYR"),
                "COM_ARM_MAG_ANG": safe_get("COM_ARM_MAG_ANG"),
                "COM_ARM_MAG_STR": safe_get("COM_ARM_MAG_STR"),
            },
            "indoor": {
                "COM_ARM_WO_GPS": safe_get("COM_ARM_WO_GPS"),
                "EKF2_GPS_CHECK": safe_get("EKF2_GPS_CHECK"),
                "EKF2_AID_MASK": safe_get("EKF2_AID_MASK"),
                "EKF2_HGT_MODE": safe_get("EKF2_HGT_MODE"),
                "CBRK_IO_SAFETY": safe_get("CBRK_IO_SAFETY"),
                "CBRK_USB_CHK": safe_get("CBRK_USB_CHK"),
            },
            "offboard": {
                "COM_OF_LOSS_T": safe_get("COM_OF_LOSS_T"),
                "COM_OBL_ACT": safe_get("COM_OBL_ACT"),
                "COM_OBL_RC_ACT": safe_get("COM_OBL_RC_ACT"),
                "COM_RC_IN_MODE": safe_get("COM_RC_IN_MODE"),
                "NAV_RCL_ACT": safe_get("NAV_RCL_ACT"),
                "NAV_DLL_ACT": safe_get("NAV_DLL_ACT"),
            },
        }
    
    def get_statustext(self) -> Optional[str]:
        msg = self._mavlink_connect.messages.get("STATUSTEXT")
        if msg is None:
            return None

        return msg.text
        
    
