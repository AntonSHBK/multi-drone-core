"""
Группа G-команд: команды перемещения и траекторий.

Этот пакет содержит команды, отвечающие за профиль движения дрона:
- немедленная остановка или прерывание движения;
- базовые действия, связанные с перемещением (взлет, посадка, loiter, вход в offboard);
- геометрические примитивы траекторий (движение к точке, линейный сегмент, круг, орбита, спираль).
"""

from .g_00 import G00_MoveToPoint
from .g_01 import G01_LineMove
from .g_02 import G02_CircleMove
from .g_20 import G20_PolylineMove
from .g_21 import G21_SplineMove
from .g_22 import G22_SpiralMove
from .g_23 import G23_OrbitMove

__all__ = [
    "G00_MoveToPoint",
    "G01_LineMove",
    "G02_CircleMove",
    "G20_PolylineMove",
    "G21_SplineMove",
    "G22_SpiralMove",
    "G23_OrbitMove",
]
