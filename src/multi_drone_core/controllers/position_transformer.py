from typing import Literal

import numpy as np

from multi_drone_core.controllers.base_data import (
    PositionData,
    OrientationData,
    VelocityData,
    AccelerationData,
)

CoordinateSystem = Literal["local_NED", "local_ENU", "global_ENU", "global_NED"]

class DroneLocalityState():
    """
    Класс для управления состоянием дрона в различных системах координат (NED и ENU).

    Атрибуты:
    ----------
    - position_local_ENU: Позиция в локальной системе ENU.
    - velocity_local_ENU: Скорость в локальной системе ENU.
    - position_local_NED: Позиция в локальной системе NED.
    - velocity_local_NED: Скорость в локальной системе NED.
    - position_global_ENU: Позиция в глобальной системе ENU.
    - velocity_global_ENU: Скорость в глобальной системе ENU.
    - position_global_NED: Позиция в глобальной системе NED.
    - velocity_global_NED: Скорость в глобальной системе NED.
    - world_position_ENU: Позиция мировых координат в ENU.
    - world_orientation_ENU: Ориентация мировых координат в ENU.
    - orientation_local_NED: Ориентация в локальной системе NED.
    - orientation_local_ENU: Ориентация в локальной системе ENU.
    """
    
    def __init__(
        self,
        world_position = PositionData(x=0, y=0, z=0),
        world_orientation = OrientationData(roll=0, pitch=0, yaw=0)
    ):
        """
        Инициализирует состояние дрона с заданной позицией и ориентацией.

        Параметры:
        ----------
        - world_position (PositionData): Начальная позиция в глобальной системе ENU.
        - world_orientation (OrientationData): Начальная ориентация в глобальной системе ENU.
        """
        
        self.position_local_ENU = PositionData()
        self.velocity_local_ENU = VelocityData()
        self.acceleration_local_ENU = AccelerationData()
        self.orientation_local_ENU = OrientationData()
        
        self.position_local_NED = PositionData()
        self.velocity_local_NED = VelocityData()
        self.acceleration_local_NED = AccelerationData()
        self.orientation_local_NED = OrientationData()
        
        self.position_global_ENU = PositionData()
        self.velocity_global_ENU = VelocityData()
        self.acceleration_global_ENU = AccelerationData()
        self.orientation_global_ENU = OrientationData()
        
        self.position_global_NED = PositionData()
        self.velocity_global_NED = VelocityData()
        self.acceleration_global_NED = AccelerationData()
        self.orientation_global_NED = OrientationData()
        
        self.world_position_ENU = world_position
        self.world_orientation_ENU = world_orientation  
        
    def get_position(
        self, 
        system: CoordinateSystem = "local_NED"
    ) -> np.ndarray:
        
        if system == 'local_NED':
            return self.position_local_NED.to_array()
        elif system == 'local_ENU': 
            return self.position_local_ENU.to_array()
        elif system == 'global_ENU':
            return self.position_global_ENU.to_array()
        elif system == 'global_NED':
            return self.position_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")
    
    def get_velocity(
        self, 
        system: CoordinateSystem = "local_NED"
    ) -> np.ndarray:
        
        if system == 'local_NED':
            return self.velocity_local_NED.to_array()
        elif system == 'local_ENU': 
            return self.velocity_local_ENU.to_array()
        elif system == 'global_ENU':
            return self.velocity_global_ENU.to_array()
        elif system == 'global_NED':
            return self.velocity_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")

    def get_acceleration(
        self,
        system: CoordinateSystem = "local_NED"
    ) -> np.ndarray:
        
        if system == "local_NED":
            return self.acceleration_local_NED.to_array()
        elif system == "local_ENU":
            return self.acceleration_local_ENU.to_array()
        elif system == "global_ENU":
            return self.acceleration_global_ENU.to_array()
        elif system == "global_NED":
            return self.acceleration_global_NED.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")
    
    def get_orientation_euler(
        self,
        system: CoordinateSystem = "local_NED",
    ) -> np.ndarray:
        """
        Возвращает ориентацию в виде углов Эйлера в указанной системе координат.

        Параметры:
        ----------
        - system (CoordinateSystem): Система координат ("local_NED", "local_ENU", "global_ENU", "global_NED").

        Возвращает:
        ----------
        - np.ndarray: Массив углов [roll, pitch, yaw] в радианах.
        """
        if system == "local_NED":
            return self.orientation_local_NED.euler.to_array()
        elif system == "local_ENU":
            return self.orientation_local_ENU.euler.to_array()
        elif system == "global_ENU":
            return self.orientation_global_ENU.euler.to_array()
        elif system == "global_NED":
            return self.orientation_global_NED.euler.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")

    def get_orientation_quaternion(
        self,
        system: CoordinateSystem = "local_NED",
    ) -> np.ndarray:
        """
        Возвращает ориентацию в виде кватерниона в указанной системе координат.

        Параметры:
        ----------
        - system (CoordinateSystem): Система координат ("local_NED", "local_ENU", "global_ENU", "global_NED").

        Возвращает:
        ----------
        - np.ndarray: Кватернион [x, y, z, w].
        """
        if system == "local_NED":
            return self.orientation_local_NED.quaternion.to_array()
        elif system == "local_ENU":
            return self.orientation_local_ENU.quaternion.to_array()
        elif system == "global_ENU":
            return self.orientation_global_ENU.quaternion.to_array()
        elif system == "global_NED":
            return self.orientation_global_NED.quaternion.to_array()
        else:
            raise ValueError(f"Unknown system: {system}")

    def update_position(
            self, 
            array_p: np.ndarray, 
            system: CoordinateSystem = "local_NED"
        ):  
        if system == 'local_NED':
            self.position_local_NED.update_from_array(array_p)
            
            self.position_local_ENU.update_from_array(
                self.position_local_NED.to_ENU())
                
            self.position_global_ENU.update_from_array(
                self.position_local_ENU.to_global(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
            
        elif system == 'local_ENU':
            self.position_local_ENU.update_from_array(array_p)
            
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
                
            self.position_global_ENU.update_from_array(
                self.position_local_ENU.to_global(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
            
        elif system == 'global_ENU':
            self.position_global_ENU.update_from_array(array_p)
            
            self.position_local_ENU.update_from_array(
                self.position_global_ENU.to_local(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
            
            self.position_global_NED.update_from_array(
                self.position_global_ENU.to_NED())
            
        elif system == 'global_NED':
            self.position_global_NED.update_from_array(array_p)
            
            self.position_global_ENU.update_from_array(
                self.position_global_NED.to_ENU())
                
            self.position_local_ENU.update_from_array(
                self.position_global_ENU.to_local(
                    reference_position=self.world_position_ENU.to_array(),
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.position_local_NED.update_from_array(
                self.position_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")
    
    def reset_position(self):
        self.position_local_ENU.update(x=0.0, y=0.0, z=0.0)
        self.position_local_NED.update(x=0.0, y=0.0, z=0.0)
        self.position_global_ENU.update(x=0.0, y=0.0, z=0.0)
        self.position_global_NED.update(x=0.0, y=0.0, z=0.0)

    def update_velocity(
        self, 
        array_v: np.ndarray, 
        system: CoordinateSystem = "local_NED"
    ):
        if system == 'local_NED':
            self.velocity_local_NED.update_from_array(array_v)
            
            self.velocity_local_ENU.update_from_array(
                self.velocity_local_NED.to_ENU())
            
            self.velocity_global_ENU.update_from_array(
                self.velocity_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'local_ENU':
            self.velocity_local_ENU.update_from_array(array_v)
            
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
            
            self.velocity_global_ENU.update_from_array(
                self.velocity_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'global_ENU':
            self.velocity_global_ENU.update_from_array(array_v)
            
            self.velocity_local_ENU.update_from_array(
                self.velocity_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
            
            self.velocity_global_NED.update_from_array(
                self.velocity_global_ENU.to_NED())
        elif system == 'global_NED':
            self.velocity_global_NED.update_from_array(array_v)
            
            self.velocity_global_ENU.update_from_array(
                self.velocity_global_NED.to_ENU())
            
            self.velocity_local_ENU.update_from_array(
                self.velocity_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))
            
            self.velocity_local_NED.update_from_array(
                self.velocity_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")
        
    def reset_velocity(self):
        self.velocity_local_ENU.update(vx=0.0, vy=0.0, vz=0.0)
        self.velocity_local_NED.update(vx=0.0, vy=0.0, vz=0.0)
        self.velocity_global_ENU.update(vx=0.0, vy=0.0, vz=0.0)
        self.velocity_global_NED.update(vx=0.0, vy=0.0, vz=0.0)
        
    def update_acceleration(
        self,
        array_a: np.ndarray,
        system: CoordinateSystem = "local_NED"
    ):
        if system == "local_NED":
            self.acceleration_local_NED.update_from_array(array_a)

            self.acceleration_local_ENU.update_from_array(
                self.acceleration_local_NED.to_ENU())

            self.acceleration_global_ENU.update_from_array(
                self.acceleration_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))

            self.acceleration_global_NED.update_from_array(
                self.acceleration_global_ENU.to_NED())
        elif system == "local_ENU":
            self.acceleration_local_ENU.update_from_array(array_a)

            self.acceleration_local_NED.update_from_array(
                self.acceleration_local_ENU.to_NED())

            self.acceleration_global_ENU.update_from_array(
                self.acceleration_local_ENU.to_global(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))

            self.acceleration_global_NED.update_from_array(
                self.acceleration_global_ENU.to_NED())
        elif system == "global_ENU":
            self.acceleration_global_ENU.update_from_array(array_a)

            self.acceleration_local_ENU.update_from_array(
                self.acceleration_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))

            self.acceleration_local_NED.update_from_array(
                self.acceleration_local_ENU.to_NED())

            self.acceleration_global_NED.update_from_array(
                self.acceleration_global_ENU.to_NED())
        elif system == "global_NED":
            self.acceleration_global_NED.update_from_array(array_a)

            self.acceleration_global_ENU.update_from_array(
                self.acceleration_global_NED.to_ENU())

            self.acceleration_local_ENU.update_from_array(
                self.acceleration_global_ENU.to_local(
                    reference_orientation=self.world_orientation_ENU.quaternion.to_array()))

            self.acceleration_local_NED.update_from_array(
                self.acceleration_local_ENU.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")

    def reset_acceleration(self):
        self.acceleration_local_ENU.update(ax=0.0, ay=0.0, az=0.0)
        self.acceleration_local_NED.update(ax=0.0, ay=0.0, az=0.0)
        self.acceleration_global_ENU.update(ax=0.0, ay=0.0, az=0.0)
        self.acceleration_global_NED.update(ax=0.0, ay=0.0, az=0.0)
        
    def update_orientation_quaternion(
        self,
        array_q: np.ndarray,
        system: CoordinateSystem = "local_NED",
    ):
        
        if system == "local_NED":
            self.orientation_local_NED.update_from_quaternion_array(array_q)

            self.orientation_local_ENU.update_from_quaternion_array(
                self.orientation_local_NED.quaternion.to_ENU())

            self.orientation_global_ENU.update_from_quaternion_array(            
                self.orientation_local_ENU.quaternion.to_global(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_global_NED.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_NED())
        elif system == "local_ENU":
            self.orientation_local_ENU.update_from_quaternion_array(array_q)

            self.orientation_local_NED.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_NED())

            self.orientation_global_ENU.update_from_quaternion_array(            
                self.orientation_local_ENU.quaternion.to_global(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_global_NED.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_NED())
        elif system == "global_ENU":
            self.orientation_global_ENU.update_from_quaternion_array(array_q)

            self.orientation_local_ENU.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_local(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_local_NED.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_NED())

            self.orientation_global_NED.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_NED())
        elif system == "global_NED":
            self.orientation_global_NED.update_from_quaternion_array(array_q)

            self.orientation_global_ENU.update_from_quaternion_array(
                self.orientation_global_NED.quaternion.to_ENU())

            self.orientation_local_ENU.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_local(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_local_NED.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")

    def update_orientation_euler(
        self,
        array_euler: np.ndarray,
        system: CoordinateSystem = "local_NED",
    ):
        if array_euler.shape != (3,):
            raise ValueError(
                f"Ожидается массив размерности (3,), получено {array_euler.shape}"
            )
        if system == "local_NED":
            self.orientation_local_NED.update_from_euler_array(array_euler)

            self.orientation_local_ENU.update_from_quaternion_array(
                self.orientation_local_NED.quaternion.to_ENU())

            self.orientation_global_ENU.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_global(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_global_NED.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_NED())
        elif system == "local_ENU":
            self.orientation_local_ENU.update_from_euler_array(array_euler)

            self.orientation_local_NED.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_NED())

            self.orientation_global_ENU.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_global(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_global_NED.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_NED())
        elif system == "global_ENU":
            self.orientation_global_ENU.update_from_euler_array(array_euler)

            self.orientation_local_ENU.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_local(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_local_NED.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_NED())

            self.orientation_global_NED.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_NED())
        elif system == "global_NED":
            self.orientation_global_NED.update_from_euler_array(array_euler)

            self.orientation_global_ENU.update_from_quaternion_array(
                self.orientation_global_NED.quaternion.to_ENU())

            self.orientation_local_ENU.update_from_quaternion_array(
                self.orientation_global_ENU.quaternion.to_local(
                    self.world_orientation_ENU.quaternion.to_array()))

            self.orientation_local_NED.update_from_quaternion_array(
                self.orientation_local_ENU.quaternion.to_NED())
        else:
            raise ValueError(f"Unknown system: {system}")

    def update_orientation(
        self,
        array_q: np.ndarray,
        system: CoordinateSystem = "local_NED",
    ):
        self.update_orientation_quaternion(array_q, system=system)
    
    def reset_orientation(self):
        self.orientation_local_ENU.update_from_euler(roll=0.0, pitch=0.0, yaw=0.0)
        self.orientation_local_NED.update_from_euler(roll=0.0, pitch=0.0, yaw=0.0)
        self.orientation_global_ENU.update_from_euler(roll=0.0, pitch=0.0, yaw=0.0)
        self.orientation_global_NED.update_from_euler(roll=0.0, pitch=0.0, yaw=0.0)
        
    def to_dict(self) -> dict:
        return {
            "local_enu": {
                "position": self.position_local_ENU.to_array().copy(),
                "velocity": self.velocity_local_ENU.to_array().copy(),
                "acceleration": self.acceleration_local_ENU.to_array().copy(),
                "euler": self.orientation_local_ENU.euler.to_array().copy(),
                "quaternion": self.orientation_local_ENU.quaternion.to_array().copy(),
            },
            "local_ned": {
                "position": self.position_local_NED.to_array().copy(),
                "velocity": self.velocity_local_NED.to_array().copy(),
                "acceleration": self.acceleration_local_NED.to_array().copy(),
                "euler": self.orientation_local_NED.euler.to_array().copy(),
                "quaternion": self.orientation_local_NED.quaternion.to_array().copy(),
            },
            "global_enu": {
                "position": self.position_global_ENU.to_array().copy(),
                "velocity": self.velocity_global_ENU.to_array().copy(),
                "acceleration": self.acceleration_global_ENU.to_array().copy(),
                "euler": self.orientation_global_ENU.euler.to_array().copy(),
                "quaternion": self.orientation_global_ENU.quaternion.to_array().copy(),
            },
            "global_ned": {
                "position": self.position_global_NED.to_array().copy(),
                "velocity": self.velocity_global_NED.to_array().copy(),
                "acceleration": self.acceleration_global_NED.to_array().copy(),
                "euler": self.orientation_global_NED.euler.to_array().copy(),
                "quaternion": self.orientation_global_NED.quaternion.to_array().copy(),
            },
        }

    def __repr__(self) -> str:
        return f"DroneLocalityState(local_enu={self.position_local_ENU}, local_ned={self.position_local_NED}, global_enu={self.position_global_ENU}, global_ned={self.position_global_NED})"
