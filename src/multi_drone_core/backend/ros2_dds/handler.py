from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, Dict, Mapping, Optional

import numpy as np

from multi_drone_core.backend.base_backend import BaseBackend

try:
    import rclpy
    from rclpy.clock import Clock
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    from px4_msgs.msg import (
        VehicleAttitude, 
        VehicleCommand, 
        VehicleLocalPosition, 
        VehicleStatus
    )

    _ROS2_AVAILABLE = True
except Exception:
    rclpy = None
    Node = Any
    Clock = None
    QoSProfile = Any
    QoSReliabilityPolicy = Any
    QoSDurabilityPolicy = Any
    QoSHistoryPolicy = Any
    VehicleAttitude = Any
    VehicleCommand = Any
    VehicleLocalPosition = Any
    VehicleStatus = Any
    _ROS2_AVAILABLE = False

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController
    
    
class VehicleStatusLocal:
    def __init__(
            self,
            nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX,
            arm_state: int = VehicleStatus.ARMING_STATE_ARMED,
            offboard_mode: bool = True,
            takeoff: bool = False,
            landing: bool = False,
            arming: bool = True,
            flight_check: bool = False,
            failsafe: bool = False,
            updating_state_rate: float = 0.5,
            takeoff_height: float = -3.5,
        ):
        self.nav_state: int = nav_state
        self.arm_state: int = arm_state

        self.offboard_mode: bool = offboard_mode
        self.takeoff: bool = takeoff
        self.landing: bool = landing
        self.arming: bool = arming

        self.flight_check: bool = flight_check
        self.failsafe: bool = failsafe
        self.takeoff_height: float = takeoff_height
        
        self.updating_state_rate: float = updating_state_rate

    def update_params(self, msg: "VehicleStatus"):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass
        
    def to_msg(self) -> "DroneParamsMsg":
        # msg = DroneParamsMsg()
        # msg.arm_message = self.arming
        # msg.arm_state = self.arm_state
        # msg.nav_state = self.nav_state
        # msg.offboard_mode = self.offboard_mode
        # msg.landing = self.landing
        # msg.failsafe = self.failsafe
        # msg.flight_check = self.flight_check
        # return msg
        pass
    
    def reset(self):
        self.__init__()
        
    def __repr__(self):
        pass
    
class Ros2BackendConfig:
    def __init__(
            self,
            nav_state: int = VehicleStatus.NAVIGATION_STATE_MAX,
            arm_state: int = VehicleStatus.ARMING_STATE_ARMED,
            offboard_mode: bool = True,
            takeoff: bool = False,
            landing: bool = False,
            arming: bool = True,
            flight_check: bool = False,
            failsafe: bool = False,
            updating_state_rate: float = 0.5,
            takeoff_height: float = -3.5,
        ):
        self.nav_state: int = nav_state
        self.arm_state: int = arm_state

        self.offboard_mode: bool = offboard_mode
        self.takeoff: bool = takeoff
        self.landing: bool = landing
        self.arming: bool = arming

        self.flight_check: bool = flight_check
        self.failsafe: bool = failsafe
        self.takeoff_height: float = takeoff_height
        
        self.updating_state_rate: float = updating_state_rate
        
    
class Ros2Backend(BaseBackend):
    """
    Backend-адаптер для PX4 через ROS2 DDS bridge.
    """

    def __init__(
        self,
        controller: "BaseController",
        backend_type: str = 'ros2_dds',
        config: Ros2BackendConfig = Ros2BackendConfig(),
    ) -> None:
        super().__init__(controller=controller, backend_type=backend_type)

        
        self._machine_id = controller.machine_id if controller is not None else 1
        self._machine_type = controller.machine_type if controller is not None else "default"
        
        self._config: Ros2BackendConfig = config
        self._params = VehicleStatusLocal()

        self._prefix_px = f"px4_{self._machine_id}"
        self._prefix_namespace = f"id_{self._machine_id}_type_{self._machine_type}"

        self._node: Optional[Node] = None
        self._publisher_vehicle_command = None
        self._subs_created = False
        self._pubs_created = False
        self._started_rclpy_here = False
        self._connected = False
        
        self._nodes_update_timers: list[Any] = []
        
        self._buffer_quaternion: np.ndarray = np.array([0.0, 0.0, 0.0, 1.0])
        self._buffer_position: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._buffer_velocity: np.ndarray = np.array([0.0, 0.0, 0.0])
        self._buffer_vessel_status: Optional["VehicleStatus"] = None

        self._last_status_time = 0.0

    @property
    def is_connected(self) -> bool:
        return bool(self._connected and self.ping())

    @property
    def topic_prefix(self) -> str:
        return self._prefix_namespace
    
    @property
    def topic_px(self) -> str:
        return self._prefix_px

    def connect(self) -> None:
        if self._connected:
            return
        self._require_ros2()

        if not rclpy.ok():
            rclpy.init()
            self._started_rclpy_here = True

        node_name = f"{self.topic_prefix}_core_node"
        self._node = rclpy.create_node(node_name)

        qos_unreliable = self._get_qos_profile(reliable=False, depth=5)
        qos_reliable = self._get_qos_profile(reliable=True, depth=10)

        self._node.create_subscription(
            VehicleAttitude,
            f"{self.topic_px}/fmu/out/vehicle_attitude",
            self._callback_vehicle_attitude,
            qos_unreliable,
        )
        self._node.create_subscription(
            VehicleLocalPosition,
            f"{self.topic_px}/fmu/out/vehicle_local_position",
            self._callback_vehicle_local_position,
            qos_unreliable,
        )
        self._node.create_subscription(
            VehicleStatus,
            f"{self.topic_px}/fmu/out/vehicle_status",
            self._callback_vehicle_status,
            qos_unreliable,
        )
        self._subs_created = True
        
        self._publisher_vehicle_command = self._node.create_publisher(
            VehicleCommand,
            f"{self.topic_px}/fmu/in/vehicle_command",
            qos_reliable,
        )
        self._pubs_created = True
        
        self._start_update_tasks()       
        
        self._connected = True

    def disconnect(self) -> None:
        if not self._connected:
            return
        
        self._stop_update_tasks()

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        if self._started_rclpy_here and rclpy is not None and rclpy.ok():
            rclpy.shutdown()
            self._started_rclpy_here = False

        self._connected = False
        self._subs_created = False
        self._pubs_created = False

    def wait_ready(
        self,
        timeout: float = 15.0,
        poll_period_s: float = 0.05,
    ) -> bool:
        timeout = max(0.1, float(timeout))
        poll_period_s = max(0.01, float(poll_period_s))
        deadline = time.time() + timeout

        while time.time() < deadline:
            if self._buffer_vessel_status is not None:
                return True
            time.sleep(poll_period_s)

        return False

    def publish_vehicle_command(self, 
        command_id: int, 
        param1: float = 0.0, 
        param2: float = 0.0, 
        param3: float = 0.0, 
        param4: float = 0.0, 
        param5: float = 0.0, 
        param6: float = 0.0, 
        param7: float = 5.0, 
        target_system: Optional[int] = None, 
        target_component: int = 1, 
        source_system: Optional[int] = None, 
        source_component: int = 1, 
        from_external: bool = True
    ):
        """
        Публикует команду VehicleCommand для управления дроном.

        Параметры:
        ----------
        command : int
            Идентификатор команды (например, армирование, взлёт, посадка и т.д.).
        param1 : float, optional
            Первичный параметр команды. Зависит от типа команды (по умолчанию 0.0).
            Например:
            - Для армирования: 1.0 (ARM), 0.0 (DISARM).
            - Для взлёта: время в секундах.
        param2 : float, optional
            Вторичный параметр команды. Используется для дополнительных данных (по умолчанию 0.0).
            Например:
            - Для SET_MODE: режим полёта (1: AUTO, 6: OFFBOARD и т.д.).
        param3 : float, optional
            Третичный параметр команды (по умолчанию 0.0).
        param4 : float, optional
            Четвёртый параметр команды (по умолчанию 0.0).
        param5 : float, optional
            Пятый параметр команды (по умолчанию 0.0). Может использоваться для координаты X.
        param6 : float, optional
            Шестой параметр команды (по умолчанию 0.0). Может использоваться для координаты Y.
        param7 : float, optional
            Седьмой параметр команды (по умолчанию 5.0). Часто используется для высоты (координаты Z).
        target_system : int, optional
            Система, которая должна выполнить команду. Если не указано, используется `self.drone_id + 1`.
        target_component : int, optional
            Компонент, который должен выполнить команду (по умолчанию 1).
        source_system : int, optional
            Система, отправляющая команду. Если не указано, используется `self.drone_id`.
        source_component : int, optional
            Компонент, отправляющий команду (по умолчанию 1).
        from_external : bool, optional
            Флаг, показывающий, что команда отправлена извне (по умолчанию True).
        """
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command_id
        
        msg.target_system = target_system if target_system is not None else self.drone_id + 1
        msg.target_component = target_component
        msg.source_system = source_system if source_system is not None else self.drone_id
        msg.source_component = source_component
        msg.from_external = from_external
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_vehicle_command.publish(msg)
        
    def send_command(self):
        pass

    def set_parameter(self, name: str, value: Any):
        pass

    def set_parameters(self, params: Mapping[str, Any]):
        pass

    def get_parameter(self, name: str):
        pass

    def get_parameters(self, names: Mapping[str, Any]):
        pass

    def read_telemetry(self):
        pass

    def get_system_state(self):
        pass

    def check_health(self):
        pass

    def ping(self):
        pass
    
    def auto_takeoff(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 
            param1=1.0, 
            param7=self.params.takeoff_hight
        )

    def enable_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0, 
            param2=6.0
        )
                
    def enable_loiter_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            param1=1.0,
            param2=5.0
        )

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=1.0
        )
    
    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
            param1=0.0
        )

    def auto_land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        
    # def stop_on_offboard(self):
    #     self.offboard_commander.update()

    def _callback_vehicle_attitude(self, msg: "VehicleAttitude") -> None:
        """
        Обработчик для получения ориентации дрона в системе ENU и преобразования в NED.
        """
        self._buffer_quaternion = msg.q

    def _callback_vehicle_local_position(self, msg: "VehicleLocalPosition") -> None:
        """
        Обработчик для получения локальной позиции дрона.
        """
        self._buffer_position = np.array([msg.x, msg.y, msg.z])
        self._buffer_velocity = np.array([msg.vx, msg.vy, msg.vz])

    def _callback_vehicle_status(self, msg: "VehicleStatus") -> None:
        """
        Обработчик для получения статуса дрона и обновления времени последнего heartbeat.
        """        
        self._buffer_vessel_status = msg
        
    def _start_update_tasks(self) -> None:
        if self._node is None or self._nodes_update_timers:
            return

        self._nodes_update_timers.append(
            self._node.create_timer(
                self._params.updating_state_rate, 
                self._task_update_orientation
            )
        )
        self._nodes_update_timers.append(
            self._node.create_timer(
                self._params.updating_state_rate, 
                self._task_update_position_velocity
            )
        )
        self._nodes_update_timers.append(
            self._node.create_timer(
                self._params.updating_state_rate, 
                self._task_update_status
            )
        )

    def _stop_update_tasks(self) -> None:
        if self._node is None:
            self._nodes_update_timers = []
            return

        for timer in self._nodes_update_timers:
            try:
                self._node.destroy_timer(timer)
            except Exception:
                pass
        self._nodes_update_timers = []

    def _task_update_orientation(self) -> None:
        try:
            self.controller.current_state.update_orientation_quaternion(
                self._buffer_quaternion,
                system="local_NED",
            )
        except Exception as exc:
            self.log_warning(f"orientation task error: {exc}")

    def _task_update_position_velocity(self) -> None:
        try:
            self.controller.current_state.update_position(
                self._buffer_position,
                system="local_NED",
            )
            self.controller.current_state.update_velocity(
                self._buffer_velocity,
                system="local_NED",
            )
        except Exception as exc:
            self.log_warning(f"position/velocity task error: {exc}")

    def _task_update_status(self) -> None:
        self._params.update_params(self._buffer_vessel_status)

    def _get_qos_profile(self, reliable: bool = True, depth: int = 10) -> Any:
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE if reliable else QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
        )

    def _require_ros2(self) -> None:
        if not _ROS2_AVAILABLE:
            raise RuntimeError(
                "ROS2 backend dependencies are unavailable. Install rclpy and px4_msgs in the runtime environment."
            )

    def _ensure_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("ROS2 backend is not connected. Call connect() first.")
        
    def log_error(self, message):
        self.logger.error(message)
        self._node.get_logger().error(message)
        
    def log_warning(self, message):
        self.logger.warning(message)
        self._node.get_logger().warning(message)

    def log_info(self, message):
        self.logger.info(message)
        self._node.get_logger().info(message)



class OffboardCommander:
    """
    Класс для управления командами в режиме Offboard.
    """
    def __init__(self, controller: X500BaseController, timer_offboard=0.1):
        """
        Инициализация командера.

        :param controller: Ссылка на базовый контроллер дрона.
        """
        self._controller = controller
        self._active = False
        
        self.publisher_offboard_mode = controller.create_publisher(
            OffboardControlMode,
            f'{controller.prefix_px}/fmu/in/offboard_control_mode',
            controller.qos_profile_reliable
        )
        self.publisher_trajectory = controller.create_publisher(
            TrajectorySetpoint,
            f'{controller.prefix_px}/fmu/in/trajectory_setpoint',
            controller.qos_profile_reliable
        )
        
        self._position = np.array([0., 0., self._controller.params.takeoff_hight])
        self._velocity = np.array([np.nan, np.nan, np.nan])
        self._acceleration = np.array([np.nan, np.nan, np.nan])
        self._yaw = np.nan
        self._yaw_speed = np.nan
        
        self.mode: Literal['position', 'velocity', 'mixed'] = 'mixed'

        self.command_timer = self._controller.create_timer(
            timer_offboard, self._timer_offboard_callback
        )
        
    def update(
        self, 
        position:np.ndarray=None, 
        velocity:np.ndarray=None, 
        acceleration:np.ndarray=None, 
        yaw:float=None, 
        yaw_speed:float=None,
        mode: Literal['position', 'velocity', 'mixed'] = None,
        system: Literal[
            "local_NED", "local_ENU", 'global_ENU', 'global_NED'
        ] = 'global_ENU'
    ):
        """
        Обновляет параметры состояния объекта. Если параметр не указан, он будет установлен в np.nan.
        
        :param position: np.ndarray или список из 3 элементов [x, y, z], позиция.
        :param velocity: np.ndarray или список из 3 элементов [vx, vy, vz], скорость.
        :param acceleration: np.ndarray или список из 3 элементов [ax, ay, az], ускорение.
        :param yaw: float, ориентация в радианах.
        :param yaw_speed: float, скорость изменения yaw в радианах/сек.
        """
        
        if position is not None:
            position = np.array(position)
            self._controller.target_position.update_position(position, system=system)
            self._position = self._controller.target_position.get_position(system='local_NED')
        else:
            self._position = np.array([np.nan, np.nan, np.nan])
            
        if velocity is not None:
            velocity = np.array(velocity)
            self._controller.target_position.update_velocity(velocity, system=system)
            self._velocity = self._controller.target_position.get_velocity(system='local_NED')
        else:
            self._velocity = np.array([np.nan, np.nan, np.nan])
            
        if acceleration is not None:
            pass
        
        if yaw is not None:
            self._yaw = float(yaw)
        else:
            self._yaw = np.nan
        
        if yaw_speed is not None:
            self._yaw_speed = float(yaw_speed)
        else:
            self._yaw_speed = np.nan
        
        if mode:
            self.mode = mode

    def send_offboard_mode(self, position=False, velocity=False, acceleration=False):
        """
        Отправляет команду для включения Offboard режима.
        """
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = position
        offboard_msg.velocity = velocity
        offboard_msg.acceleration = acceleration
        self.publisher_offboard_mode.publish(offboard_msg)

    def activate(self):
        self._active = True
        
    def desactivate(self):
        self._active = False
    
    def send_trajectory_setpoint(self):
        """
        Отправляет траекторную точку в режиме Offboard.
        """
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        # TODO: возможно стоить убрать проверку на nan
        
        trajectory_msg.position[:3] = self._position
        
        if not np.isnan(self._velocity).all():
            trajectory_msg.velocity[:3] = self._velocity
        if not np.isnan(self._acceleration).all():
            trajectory_msg.acceleration[:3] = self._acceleration
        
        trajectory_msg.yaw = self._yaw
        
        if not np.isnan(self._yaw_speed):
            trajectory_msg.yawspeed = self._yaw_speed

        self.publisher_trajectory.publish(trajectory_msg)
    
    def _timer_offboard_callback(self):
        """
        Периодически отправляет команды в режиме Offboard.
        """
        if not self._active:
            return
        
        if self.mode == "mixed":
            self.send_offboard_mode(position=True, velocity=True)
            self.send_trajectory_setpoint()
        elif self.mode == "position":
            self.send_offboard_mode(position=True)
            self.send_trajectory_setpoint()
        elif self.mode == "velocity":
            self.send_offboard_mode(velocity=True)
            self.send_trajectory_setpoint()    
