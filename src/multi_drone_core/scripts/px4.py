import os
import shlex
import subprocess
from typing import List, Literal, Optional, Sequence


class PX4Process:
    """
    Класс для создания и управления процессами PX4 с гибкими настройками.

    Параметры:
    ----------
    - drone_id : int
        Уникальный идентификатор для экземпляра дрона. По умолчанию 1.
    - drone_type : str
        Тип дрона (например, 'x500'). По умолчанию 'x500'.
    - gz_world : str
        Название файла мира Gazebo. По умолчанию 'default'.
    - spawn_position : List[float]
        Начальная позиция и ориентация дрона в формате [x, y, z, roll, pitch, yaw].
        По умолчанию [0, 0, 0, 0, 0, 0].
    - px4_autostart : int
        Идентификатор PX4 SYS_AUTOSTART для конфигурации. По умолчанию 4001.
    - px4_dir : str
        Путь к директории установки PX4. По умолчанию "/workspace/src/PX4-Autopilot/".
    - px4_simulator : str
        Тип симулятора (например, 'GZ' для Gazebo). По умолчанию 'GZ'.
    - px4_model_name : Optional[str]
        Имя существующей модели Gazebo, к которой подключается PX4. Взаимоисключается с px4_sim_model. Если оно указано, сценарий запуска пытается привязать новый экземпляр PX4 к ресурсу Gazebo с таким же именем.
    - px4_sim_model : Optional[str]
        Имя новой модели Gazebo, которая будет создана. Взаимоисключается с px4_model_name.  Если указано, сценарий запуска ищет модель в пути к ресурсам Gazebo, которая соответствует заданной переменной, создаёт её и привязывает к ней новый экземпляр PX4.
    - px4_render_engine : Optional[str]
        Механизм рендеринга для Gazebo. По умолчанию 'ogre'.
    - terminal : Literal['gnome-terminal', 'xterm', 'konsole', 'bash']
        Эмулятор терминала для запуска процесса. По умолчанию 'gnome-terminal'.

    Исключения:
    -----------
    - ValueError:
        Если px4_model_name и px4_sim_model указаны одновременно, так как они взаимно исключают друг друга.
    - ValueError:
        Если указанный эмулятор терминала не поддерживается.
    """

    def __init__(
        self,
        drone_id: int = 1,
        gz_world: str = "default",
        px4_autostart: int = 4001,
        px4_dir: str = "/workspace/src/PX4-Autopilot",
        px4_bin: Optional[str] = None,
        px4_simulator: str = "GZ",
        px4_model_name: Optional[str] = None,
        px4_sim_model: Optional[str] = None,
        px4_gz_model_pose: Sequence[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        px4_render_engine: Optional[str] = "ogre",
        standalone: bool = True,
    ):
        if px4_model_name is None and px4_sim_model is None:
            px4_sim_model = "gz_x500"

        if px4_model_name is not None and px4_sim_model is not None:
            raise ValueError("px4_model_name и px4_sim_model взаимно исключают друг друга.")

        self.drone_id = drone_id
        self.gz_world = gz_world
        self.px4_gz_model_pose = ",".join(map(str, px4_gz_model_pose))
        self.px4_autostart = px4_autostart
        self.px4_dir = px4_dir.rstrip("/")

        if px4_bin is None:
            self.px4_bin = f"{self.px4_dir}/build/px4_sitl_default/bin/px4"
        else:
            self.px4_bin = px4_bin

        self.px4_simulator = px4_simulator
        self.px4_model_name = px4_model_name
        self.px4_sim_model = px4_sim_model.strip() if px4_sim_model is not None else None
        self.px4_render_engine = px4_render_engine
        self.standalone = standalone

    def build_command(self):
        env = os.environ.copy()
        env["PX4_SYS_AUTOSTART"] = str(self.px4_autostart)
        env["PX4_SIMULATOR"] = self.px4_simulator
        env["PX4_GZ_WORLD"] = self.gz_world

        if self.standalone:
            env["PX4_GZ_STANDALONE"] = "1"

        if self.px4_model_name is not None:
            env["PX4_GZ_MODEL_NAME"] = self.px4_model_name
        elif self.px4_sim_model is not None:
            env["PX4_SIM_MODEL"] = self.px4_sim_model
            env["PX4_GZ_MODEL_POSE"] = self.px4_gz_model_pose

        if self.px4_render_engine is not None:
            env["PX4_GZ_SIM_RENDER_ENGINE"] = self.px4_render_engine

        px4_cmd = [
            self.px4_bin,
            "-i",
            str(self.drone_id),
        ]

        # Строка команды для терминалов, которые ожидают shell-команду
        px4_cmd = " ".join(shlex.quote(arg) for arg in px4_cmd)
        
        return px4_cmd, env

    def run(
        self,
        terminal: Literal["gnome-terminal", "xterm", "konsole", "bash"] = "gnome-terminal",
    ) -> subprocess.Popen:
        
        px4_cmd, env = self.build_command()
        
        if terminal == "gnome-terminal":
            launch_cmd = ["gnome-terminal", "--", "bash", "-c", f"{px4_cmd}; exec bash"]
        elif terminal == "xterm":
            launch_cmd = ["xterm", "-hold", "-e", px4_cmd]
        elif terminal == "konsole":
            launch_cmd = ["konsole", "--hold", "-e", "bash", "-c", px4_cmd]
        elif terminal == "bash":
            launch_cmd = ["bash", "-c", px4_cmd]
        else:
            raise ValueError(f"Неподдерживаемый терминал: {terminal}")

        return subprocess.Popen(
            launch_cmd,
            env=env,
            cwd=self.px4_dir
        )

if __name__ == "__main__":
    px4 = PX4Process()
    px4.run()