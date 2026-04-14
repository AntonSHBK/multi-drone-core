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
