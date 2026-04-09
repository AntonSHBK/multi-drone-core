import json
from typing import Optional, Sequence, Dict

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


class Heartbeat(MAVLink_heartbeat_message):
    def __init__(
        self,
        type: Optional[int] = None,
        autopilot: Optional[int] = None,
        base_mode: Optional[int] = None,
        custom_mode: Optional[int] = None,
        system_status: Optional[int] = None,
        mavlink_version: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - type: Тип системы (например, квадрокоптер, фиксированное крыло и т.д.).
        - autopilot: Тип автопилота (PX4, ArduPilot и др.).
        - base_mode: Битовая маска базового режима (armed, guided, auto и т.д.).
        - custom_mode: Пользовательский режим автопилота (зависит от реализации, например PX4/ArduPilot).
        - system_status: Текущее состояние системы (BOOT, STANDBY, ACTIVE, CRITICAL, EMERGENCY и т.д.).
        - mavlink_version: Версия протокола MAVLink.
        """
        super().__init__(
            type=type,
            autopilot=autopilot,
            base_mode=base_mode,
            custom_mode=custom_mode,
            system_status=system_status,
            mavlink_version=mavlink_version,
        )

    def update_from_message(self, message: MAVLink_heartbeat_message) -> None:
        self.type = message.type
        self.autopilot = message.autopilot
        self.base_mode = message.base_mode
        self.custom_mode = message.custom_mode
        self.system_status = message.system_status
        self.mavlink_version = message.mavlink_version

    def __repr__(self) -> str:
        return (
            f"Heartbeat(type={self.type}, "
            f"autopilot={self.autopilot}, "
            f"base_mode={self.base_mode}, "
            f"custom_mode={self.custom_mode}, "
            f"system_status={self.system_status}, "
            f"mavlink_version={self.mavlink_version})"
        )
    
    # def to_dict(self) -> dict:
    #     return {
    #         "type": self.type,
    #         "autopilot": self.autopilot,
    #         "base_mode": self.base_mode,
    #         "custom_mode": self.custom_mode,
    #         "system_status": self.system_status,
    #         "mavlink_version": self.mavlink_version,
    #     }
        
    # def from_dict(self, data: dict) -> None:
    #     self.type = data.get("type", self.type)
    #     self.autopilot = data.get("autopilot", self.autopilot)
    #     self.base_mode = data.get("base_mode", self.base_mode)
    #     self.custom_mode = data.get("custom_mode", self.custom_mode)
    #     self.system_status = data.get("system_status", self.system_status)
    #     self.mavlink_version = data.get("mavlink_version", self.mavlink_version)
        
    # def to_json(self) -> str:
    #     return json.dumps(self.to_dict())
    
    # def from_json(self, json_str: str) -> None:
    #     data = json.loads(json_str)
    #     self.from_dict(data)


class SysStatus(MAVLink_sys_status_message):
    def __init__(
        self,
        onboard_control_sensors_present: Optional[int] = None,
        onboard_control_sensors_enabled: Optional[int] = None,
        onboard_control_sensors_health: Optional[int] = None,
        load: Optional[int] = None,
        voltage_battery: Optional[int] = None,
        current_battery: Optional[int] = None,
        battery_remaining: Optional[int] = None,
        drop_rate_comm: Optional[int] = None,
        errors_comm: Optional[int] = None,
        errors_count1: Optional[int] = None,
        errors_count2: Optional[int] = None,
        errors_count3: Optional[int] = None,
        errors_count4: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - onboard_control_sensors_present: Битовая маска датчиков и подсистем, физически присутствующих на борту.
        - onboard_control_sensors_enabled: Битовая маска датчиков и подсистем, которые в данный момент включены.
        - onboard_control_sensors_health: Битовая маска состояния датчиков и подсистем, отражающая их исправность.
        - load: Загрузка основной системы в десятках процента (d%), например 500 = 50.0%.
        - voltage_battery: Напряжение батареи в милливольтах.
        - current_battery: Ток батареи в сантиамперах (cA), например 123 = 1.23 A.
        - battery_remaining: Оставшийся заряд батареи в процентах, -1 если значение неизвестно.
        - drop_rate_comm: Процент потерь пакетов связи в сантипроцентах (c%).
        - errors_comm: Общее количество ошибок связи.
        - errors_count1: Счётчик ошибок №1, назначение зависит от автопилота.
        - errors_count2: Счётчик ошибок №2, назначение зависит от автопилота.
        - errors_count3: Счётчик ошибок №3, назначение зависит от автопилота.
        - errors_count4: Счётчик ошибок №4, назначение зависит от автопилота.
        """
        super().__init__(
            onboard_control_sensors_present=onboard_control_sensors_present,
            onboard_control_sensors_enabled=onboard_control_sensors_enabled,
            onboard_control_sensors_health=onboard_control_sensors_health,
            load=load,
            voltage_battery=voltage_battery,
            current_battery=current_battery,
            battery_remaining=battery_remaining,
            drop_rate_comm=drop_rate_comm,
            errors_comm=errors_comm,
            errors_count1=errors_count1,
            errors_count2=errors_count2,
            errors_count3=errors_count3,
            errors_count4=errors_count4,
        )

    def update_from_message(self, message: MAVLink_sys_status_message) -> None:
        self.onboard_control_sensors_present = message.onboard_control_sensors_present
        self.onboard_control_sensors_enabled = message.onboard_control_sensors_enabled
        self.onboard_control_sensors_health = message.onboard_control_sensors_health
        self.load = message.load
        self.voltage_battery = message.voltage_battery
        self.current_battery = message.current_battery
        self.battery_remaining = message.battery_remaining
        self.drop_rate_comm = message.drop_rate_comm
        self.errors_comm = message.errors_comm
        self.errors_count1 = message.errors_count1
        self.errors_count2 = message.errors_count2
        self.errors_count3 = message.errors_count3
        self.errors_count4 = message.errors_count4

    def __repr__(self) -> str:
        return (
            f"SysStatus("
            f"onboard_control_sensors_present={self.onboard_control_sensors_present}, "
            f"onboard_control_sensors_enabled={self.onboard_control_sensors_enabled}, "
            f"onboard_control_sensors_health={self.onboard_control_sensors_health}, "
            f"load={self.load}, "
            f"voltage_battery={self.voltage_battery}, "
            f"current_battery={self.current_battery}, "
            f"battery_remaining={self.battery_remaining}, "
            f"drop_rate_comm={self.drop_rate_comm}, "
            f"errors_comm={self.errors_comm}, "
            f"errors_count1={self.errors_count1}, "
            f"errors_count2={self.errors_count2}, "
            f"errors_count3={self.errors_count3}, "
            f"errors_count4={self.errors_count4})"
        )


class ExtendedSysState(MAVLink_extended_sys_state_message):
    def __init__(
        self,
        vtol_state: Optional[int] = None,
        landed_state: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - vtol_state: Состояние VTOL (например: MULTICOPTER, FIXED_WING, TRANSITION и т.д.).
        - landed_state: Состояние посадки (на земле, в воздухе, посадка, взлёт и т.д.).
        """
        super().__init__(
            vtol_state=vtol_state,
            landed_state=landed_state,
        )

    def update_from_message(self, message: MAVLink_extended_sys_state_message) -> None:
        self.vtol_state = message.vtol_state
        self.landed_state = message.landed_state

    def __repr__(self) -> str:
        return (
            f"ExtendedSysState("
            f"vtol_state={self.vtol_state}, "
            f"landed_state={self.landed_state})"
        )
    

class LocalPositionNED(MAVLink_local_position_ned_message):
    def __init__(
        self,
        time_boot_ms: Optional[int] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None
    ) -> None:
        """
        Параметры:
        - time_boot_ms: Время в миллисекундах с момента загрузки системы.
        - x, y, z: Позиция в метрах относительно начальной точки (обычно взлетной площадки) в системе координат NED (North-East-Down).
        - vx, vy, vz: Скорость в метрах в секунду относительно начальной точки в системе координат NED.
        """
        super().__init__(
            time_boot_ms=time_boot_ms,
            x=x,
            y=y,
            z=z,
            vx=vx,
            vy=vy,
            vz=vz
        )
        
    def update_from_message(self, message: MAVLink_local_position_ned_message) -> None:
        self.time_boot_ms = message.time_boot_ms
        self.x = message.x
        self.y = message.y
        self.z = message.z
        self.vx = message.vx
        self.vy = message.vy
        self.vz = message.vz
        
    def __repr__(self) -> str:
        return (
            f"LocalPositionNED(time_boot_ms={self.time_boot_ms}, "
            f"x={self.x}, y={self.y}, z={self.z}, "
            f"vx={self.vx}, vy={self.vy}, vz={self.vz})"
        )


class PositionTargetLocalNED(MAVLink_position_target_local_ned_message):
    def __init__(
        self,
        time_boot_ms: Optional[int] = None,
        coordinate_frame: Optional[int] = None,
        type_mask: Optional[int] = None,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        vx: Optional[float] = None,
        vy: Optional[float] = None,
        vz: Optional[float] = None,
        afx: Optional[float] = None,
        afy: Optional[float] = None,
        afz: Optional[float] = None,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None,
    ) -> None:
        """
        Параметры:
        - time_boot_ms: Время в миллисекундах с момента загрузки системы.
        - coordinate_frame: Система координат, в которой задана цель.
        - type_mask: Битовая маска, определяющая какие поля цели следует игнорировать.
        - x, y, z: Целевая позиция в локальной системе координат NED, в метрах.
        - vx, vy, vz: Целевая скорость по осям в м/с.
        - afx, afy, afz: Целевое ускорение или сила по осям в м/с².
        - yaw: Целевой угол рыскания в радианах.
        - yaw_rate: Целевая угловая скорость по рысканию в рад/с.
        """
        super().__init__(
            time_boot_ms=time_boot_ms,
            coordinate_frame=coordinate_frame,
            type_mask=type_mask,
            x=x,
            y=y,
            z=z,
            vx=vx,
            vy=vy,
            vz=vz,
            afx=afx,
            afy=afy,
            afz=afz,
            yaw=yaw,
            yaw_rate=yaw_rate,
        )

    def update_from_message(self, message: MAVLink_position_target_local_ned_message) -> None:
        self.time_boot_ms = message.time_boot_ms
        self.coordinate_frame = message.coordinate_frame
        self.type_mask = message.type_mask
        self.x = message.x
        self.y = message.y
        self.z = message.z
        self.vx = message.vx
        self.vy = message.vy
        self.vz = message.vz
        self.afx = message.afx
        self.afy = message.afy
        self.afz = message.afz
        self.yaw = message.yaw
        self.yaw_rate = message.yaw_rate

    def __repr__(self) -> str:
        return (
            f"PositionTargetLocalNED(time_boot_ms={self.time_boot_ms}, "
            f"coordinate_frame={self.coordinate_frame}, "
            f"type_mask={self.type_mask}, "
            f"x={self.x}, y={self.y}, z={self.z}, "
            f"vx={self.vx}, vy={self.vy}, vz={self.vz}, "
            f"afx={self.afx}, afy={self.afy}, afz={self.afz}, "
            f"yaw={self.yaw}, yaw_rate={self.yaw_rate})"
        )


class Attitude(MAVLink_attitude_message):
    def __init__(
        self,
        time_boot_ms: Optional[int] = None,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        yaw: Optional[float] = None,
        rollspeed: Optional[float] = None,
        pitchspeed: Optional[float] = None,
        yawspeed: Optional[float] = None,
    ) -> None:
        """
        Параметры:
        - time_boot_ms: Время в миллисекундах с момента загрузки системы.
        - roll: Угол крена в радианах.
        - pitch: Угол тангажа в радианах.
        - yaw: Угол рыскания в радианах.
        - rollspeed: Угловая скорость по крену в радианах в секунду.
        - pitchspeed: Угловая скорость по тангажу в радианах в секунду.
        - yawspeed: Угловая скорость по рысканию в радианах в секунду.
        """
        super().__init__(
            time_boot_ms=time_boot_ms,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            rollspeed=rollspeed,
            pitchspeed=pitchspeed,
            yawspeed=yawspeed,
        )

    def update_from_message(self, message: MAVLink_attitude_message) -> None:
        self.time_boot_ms = message.time_boot_ms
        self.roll = message.roll
        self.pitch = message.pitch
        self.yaw = message.yaw
        self.rollspeed = message.rollspeed
        self.pitchspeed = message.pitchspeed
        self.yawspeed = message.yawspeed

    def __repr__(self) -> str:
        return (
            f"Attitude(time_boot_ms={self.time_boot_ms}, "
            f"roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}, "
            f"rollspeed={self.rollspeed}, pitchspeed={self.pitchspeed}, "
            f"yawspeed={self.yawspeed})"
        ) 
 
       
class AttitudeQuaternion(MAVLink_attitude_quaternion_message):
    def __init__(
        self,
        time_boot_ms: Optional[int] = None,
        q1: Optional[float] = None,
        q2: Optional[float] = None,
        q3: Optional[float] = None,
        q4: Optional[float] = None,
        rollspeed: Optional[float] = None,
        pitchspeed: Optional[float] = None,
        yawspeed: Optional[float] = None,
    ) -> None:
        """
        Параметры:
        - time_boot_ms: Время в миллисекундах с момента загрузки системы.
        - q1, q2, q3, q4: Кватернион ориентации (w, x, y, z).
          Единичная ориентация (без вращения) задаётся как (1, 0, 0, 0).
        - rollspeed: Угловая скорость по крену в радианах в секунду.
        - pitchspeed: Угловая скорость по тангажу в радианах в секунду.
        - yawspeed: Угловая скорость по рысканию в радианах в секунду.
        """
        super().__init__(
            time_boot_ms=time_boot_ms,
            q1=q1,
            q2=q2,
            q3=q3,
            q4=q4,
            rollspeed=rollspeed,
            pitchspeed=pitchspeed,
            yawspeed=yawspeed,
        )

    def update_from_message(self, message: MAVLink_attitude_quaternion_message) -> None:
        self.time_boot_ms = message.time_boot_ms
        self.q1 = message.q1
        self.q2 = message.q2
        self.q3 = message.q3
        self.q4 = message.q4
        self.rollspeed = message.rollspeed
        self.pitchspeed = message.pitchspeed
        self.yawspeed = message.yawspeed

    def __repr__(self) -> str:
        return (
            f"AttitudeQuaternion(time_boot_ms={self.time_boot_ms}, "
            f"q1={self.q1}, q2={self.q2}, q3={self.q3}, q4={self.q4}, "
            f"rollspeed={self.rollspeed}, pitchspeed={self.pitchspeed}, "
            f"yawspeed={self.yawspeed})"
        )


class AttitudeTarget(MAVLink_attitude_target_message):
    def __init__(
        self,
        time_boot_ms: Optional[int] = None,
        type_mask: Optional[int] = None,
        q: Optional[Sequence[float]] = None,
        body_roll_rate: Optional[float] = None,
        body_pitch_rate: Optional[float] = None,
        body_yaw_rate: Optional[float] = None,
        thrust: Optional[float] = None,
    ) -> None:
        """
        Параметры:
        - time_boot_ms: Время в миллисекундах с момента загрузки системы.
        - type_mask: Битовая маска, определяющая какие компоненты целевой ориентации
          и угловых скоростей следует игнорировать.
        - q: Целевая ориентация в виде кватерниона (w, x, y, z).
        - body_roll_rate: Целевая угловая скорость по крену в радианах в секунду.
        - body_pitch_rate: Целевая угловая скорость по тангажу в радианах в секунду.
        - body_yaw_rate: Целевая угловая скорость по рысканию в радианах в секунду.
        - thrust: Целевая тяга.
        """
        super().__init__(
            time_boot_ms=time_boot_ms,
            type_mask=type_mask,
            q=q,
            body_roll_rate=body_roll_rate,
            body_pitch_rate=body_pitch_rate,
            body_yaw_rate=body_yaw_rate,
            thrust=thrust,
        )

    def update_from_message(self, message: MAVLink_attitude_target_message) -> None:
        self.time_boot_ms = message.time_boot_ms
        self.type_mask = message.type_mask
        self.q = message.q
        self.body_roll_rate = message.body_roll_rate
        self.body_pitch_rate = message.body_pitch_rate
        self.body_yaw_rate = message.body_yaw_rate
        self.thrust = message.thrust

    def __repr__(self) -> str:
        return (
            f"AttitudeTarget(time_boot_ms={self.time_boot_ms}, "
            f"type_mask={self.type_mask}, q={self.q}, "
            f"body_roll_rate={self.body_roll_rate}, "
            f"body_pitch_rate={self.body_pitch_rate}, "
            f"body_yaw_rate={self.body_yaw_rate}, "
            f"thrust={self.thrust})"
        )


class EstimatorStatus(MAVLink_estimator_status_message):
    def __init__(
        self,
        time_usec: Optional[int] = None,
        flags: Optional[int] = None,
        vel_ratio: Optional[float] = None,
        pos_horiz_ratio: Optional[float] = None,
        pos_vert_ratio: Optional[float] = None,
        mag_ratio: Optional[float] = None,
        hagl_ratio: Optional[float] = None,
        tas_ratio: Optional[float] = None,
        pos_horiz_accuracy: Optional[float] = None,
        pos_vert_accuracy: Optional[float] = None,
    ) -> None:
        """
        Параметры:
        - time_usec: Время в микросекундах.
        - flags: Битовая маска статуса оценщика, показывающая какие выходы EKF валидны.
        - vel_ratio: Отношение инновации по скорости к порогу проверки.
        - pos_horiz_ratio: Отношение инновации по горизонтальной позиции к порогу проверки.
        - pos_vert_ratio: Отношение инновации по вертикальной позиции к порогу проверки.
        - mag_ratio: Отношение инновации магнитометра к порогу проверки.
        - hagl_ratio: Отношение инновации по высоте над землёй к порогу проверки.
        - tas_ratio: Отношение инновации истинной воздушной скорости к порогу проверки.
        - pos_horiz_accuracy: Оценка точности горизонтального положения в метрах.
        - pos_vert_accuracy: Оценка точности вертикального положения в метрах.
        """
        super().__init__(
            time_usec=time_usec,
            flags=flags,
            vel_ratio=vel_ratio,
            pos_horiz_ratio=pos_horiz_ratio,
            pos_vert_ratio=pos_vert_ratio,
            mag_ratio=mag_ratio,
            hagl_ratio=hagl_ratio,
            tas_ratio=tas_ratio,
            pos_horiz_accuracy=pos_horiz_accuracy,
            pos_vert_accuracy=pos_vert_accuracy,
        )

    def update_from_message(self, message: MAVLink_estimator_status_message) -> None:
        self.time_usec = message.time_usec
        self.flags = message.flags
        self.vel_ratio = message.vel_ratio
        self.pos_horiz_ratio = message.pos_horiz_ratio
        self.pos_vert_ratio = message.pos_vert_ratio
        self.mag_ratio = message.mag_ratio
        self.hagl_ratio = message.hagl_ratio
        self.tas_ratio = message.tas_ratio
        self.pos_horiz_accuracy = message.pos_horiz_accuracy
        self.pos_vert_accuracy = message.pos_vert_accuracy

    def __repr__(self) -> str:
        return (
            f"EstimatorStatus(time_usec={self.time_usec}, "
            f"flags={self.flags}, "
            f"vel_ratio={self.vel_ratio}, "
            f"pos_horiz_ratio={self.pos_horiz_ratio}, "
            f"pos_vert_ratio={self.pos_vert_ratio}, "
            f"mag_ratio={self.mag_ratio}, "
            f"hagl_ratio={self.hagl_ratio}, "
            f"tas_ratio={self.tas_ratio}, "
            f"pos_horiz_accuracy={self.pos_horiz_accuracy}, "
            f"pos_vert_accuracy={self.pos_vert_accuracy})"
        )


class ScaledPressure(MAVLink_scaled_pressure_message):
    def __init__(
        self,
        time_boot_ms: Optional[int] = None,
        press_abs: Optional[float] = None,
        press_diff: Optional[float] = None,
        temperature: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - time_boot_ms: Время в миллисекундах с момента загрузки системы.
        - press_abs: Абсолютное давление в гектопаскалях (hPa).
        - press_diff: Дифференциальное давление в гектопаскалях (hPa).
        - temperature: Температура в сотых долях градуса Цельсия (cdegC).
        """
        super().__init__(
            time_boot_ms=time_boot_ms,
            press_abs=press_abs,
            press_diff=press_diff,
            temperature=temperature,
        )

    def update_from_message(self, message: MAVLink_scaled_pressure_message) -> None:
        self.time_boot_ms = message.time_boot_ms
        self.press_abs = message.press_abs
        self.press_diff = message.press_diff
        self.temperature = message.temperature

    def __repr__(self) -> str:
        return (
            f"ScaledPressure(time_boot_ms={self.time_boot_ms}, "
            f"press_abs={self.press_abs}, "
            f"press_diff={self.press_diff}, "
            f"temperature={self.temperature})"
        )


class MissionCurrent(MAVLink_mission_current_message):
    def __init__(
        self,
        seq: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - seq: Индекс (номер) текущей активной миссионной точки.
        """
        super().__init__(
            seq=seq,
        )

    def update_from_message(self, message: MAVLink_mission_current_message) -> None:
        self.seq = message.seq

    def __repr__(self) -> str:
        return f"MissionCurrent(seq={self.seq})"


class Ping(MAVLink_ping_message):
    def __init__(
        self,
        time_usec: Optional[int] = None,
        seq: Optional[int] = None,
        target_system: Optional[int] = None,
        target_component: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - time_usec: Время в микросекундах (обычно timestamp отправки).
        - seq: Последовательный номер ping-сообщения.
        - target_system: ID целевой системы.
        - target_component: ID целевого компонента.
        """
        super().__init__(
            time_usec=time_usec,
            seq=seq,
            target_system=target_system,
            target_component=target_component,
        )

    def update_from_message(self, message: MAVLink_ping_message) -> None:
        self.time_usec = message.time_usec
        self.seq = message.seq
        self.target_system = message.target_system
        self.target_component = message.target_component

    def __repr__(self) -> str:
        return (
            f"Ping(time_usec={self.time_usec}, "
            f"seq={self.seq}, "
            f"target_system={self.target_system}, "
            f"target_component={self.target_component})"
        )


class HighresIMU(MAVLink_highres_imu_message):
    def __init__(
        self,
        time_usec: Optional[int] = None,
        xacc: Optional[float] = None,
        yacc: Optional[float] = None,
        zacc: Optional[float] = None,
        xgyro: Optional[float] = None,
        ygyro: Optional[float] = None,
        zgyro: Optional[float] = None,
        xmag: Optional[float] = None,
        ymag: Optional[float] = None,
        zmag: Optional[float] = None,
        abs_pressure: Optional[float] = None,
        diff_pressure: Optional[float] = None,
        pressure_alt: Optional[float] = None,
        temperature: Optional[float] = None,
        fields_updated: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - time_usec: Время в микросекундах.
        - xacc, yacc, zacc: Линейные ускорения по осям в м/с² (NED система координат).
        - xgyro, ygyro, zgyro: Угловые скорости по осям в рад/с.
        - xmag, ymag, zmag: Показания магнитометра в гауссах.
        - abs_pressure: Абсолютное давление в гектопаскалях (hPa).
        - diff_pressure: Дифференциальное давление в гектопаскалях (hPa).
        - pressure_alt: Высота, рассчитанная по давлению.
        - temperature: Температура в градусах Цельсия.
        - fields_updated: Битовая маска, показывающая какие поля были обновлены.
        """
        super().__init__(
            time_usec=time_usec,
            xacc=xacc,
            yacc=yacc,
            zacc=zacc,
            xgyro=xgyro,
            ygyro=ygyro,
            zgyro=zgyro,
            xmag=xmag,
            ymag=ymag,
            zmag=zmag,
            abs_pressure=abs_pressure,
            diff_pressure=diff_pressure,
            pressure_alt=pressure_alt,
            temperature=temperature,
            fields_updated=fields_updated,
        )

    def update_from_message(self, message: MAVLink_highres_imu_message) -> None:
        self.time_usec = message.time_usec
        self.xacc = message.xacc
        self.yacc = message.yacc
        self.zacc = message.zacc
        self.xgyro = message.xgyro
        self.ygyro = message.ygyro
        self.zgyro = message.zgyro
        self.xmag = message.xmag
        self.ymag = message.ymag
        self.zmag = message.zmag
        self.abs_pressure = message.abs_pressure
        self.diff_pressure = message.diff_pressure
        self.pressure_alt = message.pressure_alt
        self.temperature = message.temperature
        self.fields_updated = message.fields_updated

    def __repr__(self) -> str:
        return (
            f"HighresIMU(time_usec={self.time_usec}, "
            f"xacc={self.xacc}, yacc={self.yacc}, zacc={self.zacc}, "
            f"xgyro={self.xgyro}, ygyro={self.ygyro}, zgyro={self.zgyro}, "
            f"xmag={self.xmag}, ymag={self.ymag}, zmag={self.zmag}, "
            f"abs_pressure={self.abs_pressure}, diff_pressure={self.diff_pressure}, "
            f"pressure_alt={self.pressure_alt}, temperature={self.temperature}, "
            f"fields_updated={self.fields_updated})"
        )


class VFRHUD(MAVLink_vfr_hud_message):
    def __init__(
        self,
        airspeed: Optional[float] = None,
        groundspeed: Optional[float] = None,
        heading: Optional[int] = None,
        throttle: Optional[int] = None,
        alt: Optional[float] = None,
        climb: Optional[float] = None,
    ) -> None:
        """
        Параметры:
        - airspeed: Воздушная скорость в м/с.
        - groundspeed: Путевая скорость в м/с.
        - heading: Курс в градусах.
        - throttle: Положение газа в процентах.
        - alt: Высота в метрах.
        - climb: Вертикальная скорость в м/с.
        """
        super().__init__(
            airspeed=airspeed,
            groundspeed=groundspeed,
            heading=heading,
            throttle=throttle,
            alt=alt,
            climb=climb,
        )

    def update_from_message(self, message: MAVLink_vfr_hud_message) -> None:
        self.airspeed = message.airspeed
        self.groundspeed = message.groundspeed
        self.heading = message.heading
        self.throttle = message.throttle
        self.alt = message.alt
        self.climb = message.climb

    def __repr__(self) -> str:
        return (
            f"VFRHUD(airspeed={self.airspeed}, "
            f"groundspeed={self.groundspeed}, "
            f"heading={self.heading}, "
            f"throttle={self.throttle}, "
            f"alt={self.alt}, "
            f"climb={self.climb})"
        )
        

class GPSRawInt(MAVLink_gps_raw_int_message):
    def __init__(
        self,
        time_usec: Optional[int] = None,
        fix_type: Optional[int] = None,
        lat: Optional[int] = None,
        lon: Optional[int] = None,
        alt: Optional[int] = None,
        eph: Optional[int] = None,
        epv: Optional[int] = None,
        vel: Optional[int] = None,
        cog: Optional[int] = None,
        satellites_visible: Optional[int] = None,
    ) -> None:
        """
        Параметры:
        - time_usec: Время в микросекундах.
        - fix_type: Тип GPS-фиксации.
        - lat: Широта в формате degE7.
        - lon: Долгота в формате degE7.
        - alt: Высота над средним уровнем моря в миллиметрах.
        - eph: Горизонтальная погрешность позиционирования.
        - epv: Вертикальная погрешность позиционирования.
        - vel: Скорость по земле в см/с.
        - cog: Курс по земле в сотых долях градуса (cdeg).
        - satellites_visible: Количество видимых спутников.
        """
        super().__init__(
            time_usec=time_usec,
            fix_type=fix_type,
            lat=lat,
            lon=lon,
            alt=alt,
            eph=eph,
            epv=epv,
            vel=vel,
            cog=cog,
            satellites_visible=satellites_visible,
        )

    def update_from_message(self, message: MAVLink_gps_raw_int_message) -> None:
        self.time_usec = message.time_usec
        self.fix_type = message.fix_type
        self.lat = message.lat
        self.lon = message.lon
        self.alt = message.alt
        self.eph = message.eph
        self.epv = message.epv
        self.vel = message.vel
        self.cog = message.cog
        self.satellites_visible = message.satellites_visible

    def __repr__(self) -> str:
        return (
            f"GPSRawInt(time_usec={self.time_usec}, "
            f"fix_type={self.fix_type}, "
            f"lat={self.lat}, lon={self.lon}, alt={self.alt}, "
            f"eph={self.eph}, epv={self.epv}, "
            f"vel={self.vel}, cog={self.cog}, "
            f"satellites_visible={self.satellites_visible})"
        )


class ParamValue(MAVLink_param_value_message):
    def __init__(
        self,
        param_id: Optional[bytes],
        param_value: Optional[float],
        param_type: Optional[int],
        param_count: Optional[int],
        param_index: Optional[int],
    ) -> None:
        """
        Параметры:
        - param_id: Имя параметра (строка, до 16 символов).
        - param_value: Значение параметра (float).
        - param_type: Тип параметра (MAV_PARAM_TYPE_*).
        - param_count: Общее количество параметров на автопилоте.
        - param_index: Индекс текущего параметра.
        """
        super().__init__(
            param_id=param_id,
            param_value=param_value,
            param_type=param_type,
            param_count=param_count,
            param_index=param_index,
        )

    def update_from_message(self, message: MAVLink_param_value_message) -> None:
        """
        Обновить объект на основе входящего MAVLink_param_value_message.
        """
        self.param_id = message.param_id
        self.param_value = message.param_value
        self.param_type = message.param_type
        self.param_count = message.param_count
        self.param_index = message.param_index

    def __repr__(self) -> str:
        return (
            f"ParamValue("
            f"id='{self.param_id}', "
            f"value={self.param_value}, "
            f"type={self.param_type}, "
            f"count={self.param_count}, "
            f"index={self.param_index}"
            f")"
        )


class MachineSystemData:
    def __init__(self) -> None:
        self.sys_status = SysStatus()
        self.estimator_status = EstimatorStatus()
        self.local_position_ned = LocalPositionNED()
        self.position_target_local_ned = PositionTargetLocalNED()
        self.attitude = Attitude()
        self.attitude_quaternion = AttitudeQuaternion()
        self.attitude_target = AttitudeTarget()
        self.heartbeat = Heartbeat()
        self.extended_sys_state = ExtendedSysState()
        self.scaled_pressure = ScaledPressure()
        self.mission_current = MissionCurrent()
        self.ping = Ping()
        self.highres_imu = HighresIMU()
        self.vfr_hud = VFRHUD()
        self.gps_raw_int = GPSRawInt()
        
        self.parameters: Dict[str, ParamValue] = {}

    def update_from_msg(self, msg_type: str, msg: MAVLink_message) -> None:
        if msg_type == "LOCAL_POSITION_NED":
            self.local_position_ned.update_from_message(msg)
        elif msg_type == "POSITION_TARGET_LOCAL_NED":
            self.position_target_local_ned.update_from_message(msg)
        elif msg_type == "SYS_STATUS":
            self.sys_status.update_from_message(msg)
        elif msg_type == "ATTITUDE":
            self.attitude.update_from_message(msg)
        elif msg_type == "Heartbeat":
            self.heartbeat.update_from_message(msg)
        elif msg_type == "ATTITUDE_QUATERNION":
            self.attitude_quaternion.update_from_message(msg)
        elif msg_type == "ATTITUDE_TARGET":
            self.attitude_target.update_from_message(msg)
        elif msg_type == "ESTIMATOR_STATUS":
            self.estimator_status.update_from_message(msg)
        elif msg_type == "EXTENDED_SYS_STATE":
            self.extended_sys_state.update_from_message(msg)
        elif msg_type == "SCALED_PRESSURE":
            self.scaled_pressure.update_from_message(msg)
        elif msg_type == "MISSION_CURRENT":
            self.mission_current.update_from_message(msg)
        elif msg_type == "PING":
            self.ping.update_from_message(msg)
        elif msg_type == "GPS_RAW_INT":
            self.gps_raw_int.update_from_message(msg)
        elif msg_type == "HIGHRES_IMU":
            self.highres_imu.update_from_message(msg)
        elif msg_type == "PARAM_VALUE":
            self._update_parameter(msg)
        else:
            return

    def _update_parameter(self, msg: MAVLink_param_value_message) -> None:
        """
        Обновление или добавление параметра в словарь.
        """
        param = ParamValue(
            param_id=msg.param_id,
            param_value=msg.param_value,
            param_type=msg.param_type,
            param_count=msg.param_count,
            param_index=msg.param_index,
        )
        self.parameters[param.param_id] = param

    def get_parameter(self, name: str) -> ParamValue | None:
        return self.parameters.get(name)

    def get_all_parameters(self) -> Dict[str, ParamValue]:
        return self.parameters.copy()

        