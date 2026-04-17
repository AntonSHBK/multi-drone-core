"""
Группа M-команд: команды управления миссией и состоянием дрона.

Этот пакет содержит команды, которые управляют логикой работы дрона:
- управление потоком миссии (стоп, пауза, завершение миссии);
- управление потоком миссии (стоп, пауза, продолжить);
- действия с аппаратом (arm, disarm, takeoff, land);
- переключение режимов полета (manual, altctl, posctl, offboard, loiter/hold);
- команды безопасности и аварийного управления (failsafe, emergency land, kill motors).
"""

from .m_00 import M00_Stop
from .m_01 import M01_Pause
from .m_02 import M02_Continue
from .m_10 import M10_Arm
from .m_11 import M11_Disarm
from .m_12 import M12_Takeoff
from .m_13 import M13_Land
from .m_20 import M20_Manual
from .m_21 import M21_AltCtl
from .m_22 import M22_Position
from .m_23 import M23_Offboard
from .m_24 import M24_HoldLoiter

__all__ = [
    "M00_Stop",
    "M01_Pause",
    "M02_Continue",
    "M02_EndMission",
    "M10_Arm",
    "M11_Disarm",
    "M12_Takeoff",
    "M13_Land",
    "M20_Manual",
    "M21_AltCtl",
    "M22_Position",
    "M23_Offboard",
    "M24_HoldLoiter",
    "M40_Failsafe",
    "M41_EmergencyLand",
    "M42_KillMotors",
]
