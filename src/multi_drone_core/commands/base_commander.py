from abc import ABC, abstractmethod
from typing import Type
from threading import Lock
from collections import deque

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController
    from multi_drone_core.commands.base_command import BaseCommand


class BaseCommander(ABC):
    """
    Базовый класс для командеров, управляющих командами дрона.
    """
    def __init__(self, controller: "BaseController"):
        self.controller = controller
        self.logger = controller.loggers.commander
        self.command_classes = {}
        self.command_queue = deque()
        self.command_history = []
        self.lock = Lock()
        self.active_command: "BaseCommand" = None
        self._execution_paused = False
        
    @abstractmethod
    def start(self):
        pass    
    
    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def process_new_command(self, data):
        """
        Метод для обработки входящей команды.
        :param data: Словарь данных команды или готовый объект команды.
        """
        pass

    def stop_command_execution(self) -> None:
        """
        Full stop of command execution pipeline.
        """
        self._execution_paused = True
        self.clear_commands()
        try:
            self.controller.send_offboard_setpoint(velocity=[0.0, 0.0, 0.0])
        except Exception as exc:
            self.log_warning(f"Failed to send hold setpoint on stop: {exc}")

    def pause_command_execution(self) -> None:
        """
        Pause current command execution.
        """
        self._execution_paused = True
        try:
            self.controller.send_offboard_setpoint(velocity=[0.0, 0.0, 0.0])
        except Exception as exc:
            self.log_warning(f"Failed to send hold setpoint on pause: {exc}")
        self.log_info("Command execution paused.")

    def resume_command_execution(self) -> None:
        """
        Resume command execution after pause.
        """
        self._execution_paused = False
        self.log_info("Command execution resumed.")

    def add_command(self, command: "BaseCommand"):
        """
        Добавляет команду в очередь.
        :param command: Объект команды.
        """
        with self.lock:
            self.command_queue.append(command)
        self.log_info(f"Добавлена команда в очередь: {command.name}")
        
    def get_command(self):
        return self.command_queue.popleft()

    def before_command(self, command: "BaseCommand") -> None:
        """
        Хук перед выполнением команды.
        """
        command.before_execute()

    def after_command(self, command: "BaseCommand") -> None:
        """
        Хук после выполнения команды.
        """
        command.after_execute()

    def clear_command_queue(self):
        """
        Метод очистки команд из очереди.
        """
        with self.lock:
            self.command_queue.clear()        
        self.log_info("Очередь команд очищена.")
        
    def clear_command_history(self):
        """
        Метод очистки команд из истории.
        """
        self.command_history.clear()        
        self.log_info("История команд очищена.")

    def clear_commands(self) -> None:
        """
        Полная очистка: сбрасывает активную команду и очередь.
        """
        with self.lock:
            if self.active_command is not None:
                try:
                    self.active_command.interrupt()
                except Exception:
                    pass
                self.active_command = None
            self.clear_command_queue()
        self.log_info("Активная команда и очередь очищены.")
        
    def add_command_class(self, name: str, class_type: Type[object]) -> None:
        """
        Добавляет ключ и класс в словарь.

        :param class_dict: Словарь, где ключи - это названия классов, а значения - классы.
        :param name: Название класса для ключа.
        :param class_type: Класс Python, который будет добавлен в словарь.
        """
        if name in self.command_classes:
            raise ValueError(f"Класс с именем '{name}' уже существует в словаре.")
        self.command_classes[name] = class_type

    def log_info(self, message: str) -> None:
        self.logger.info(message)

    def log_warning(self, message: str) -> None:
        self.logger.warning(message)

    def log_error(self, message: str) -> None:
        self.logger.error(message)
