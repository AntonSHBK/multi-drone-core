# base_command.py

from abc import ABC, abstractmethod

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController

class BaseCommand(ABC):
    def __init__(self, name, counter=0, current_step=0, is_special_command: bool = False):
        self.name = name
        self.counter = counter
        self.complete = False
        self.interrupt = False
        self.current_step = current_step
        self.is_special_command = is_special_command
        self.description: str = "Base class"
        self.is_ready = False

    def get_description(self):
        return self.description
    
    @abstractmethod
    def execute(self, controller):
        """
        Выполняет текущую команду, используя предоставленный контроллер.
        :param controller: Экземпляр контроллера для отправки команд на дрон.
        """
        pass

    @abstractmethod
    def is_complete(self, controller: 'BaseController') -> bool:
        """
        Проверяет, завершена ли команда.
        :param controller: Экземпляр контроллера для проверки состояния дрона.
        :return: True, если команда завершена, иначе False.
        """
        pass

    @abstractmethod
    def can_execute(self, controller: 'BaseController') -> bool:
        """
        Проверяет, можно ли выполнить команду.
        :param controller: Экземпляр контроллера для проверки условий выполнения.
        :return: True, если команда может быть выполнена, иначе False.
        """
        pass

    def safe_execute(self, controller: 'BaseController'):
        """
        Проверяет возможность выполнения команды и завершённость перед вызовом execute.
        :param controller: Экземпляр контроллера.
        """
        if self._check_finish():
            return
        if not self.can_execute(controller):
            self.mark_as_interrupted()
            return
        self.execute(controller)
        
    def _check_finish(self):
        if self.complete or self.interrupt:
            return True
        else:
            return False
    
    def _check_complete(self):
        return True if self.complete else False
        
    def _check_interrupt(self):
        return True if self.interrupt else False
    
    def ready(self):
        self.is_ready = True
    
    def mark_as_interrupted(self):
        """
        Устанавливает флаг, что команда должна быть прервана.
        """
        self.interrupt = True
        
    def complete_command(self):
        """
        Устанавливает флаг, что команда завершена.
        """
        self.complete = True

    def to_dict(self) -> dict:
        """
        Формирует словарь с параметрами команды.
        :return: Словарь с параметрами команды.
        """
        return {
            "name": self.name,
            "counter": self.counter,
            "current_step": self.current_step,
            "is_special_command": self.is_special_command,
        }
        
    def before_execute(self):
        return
    
    def after_execute(self):
        return

    @classmethod
    def from_dict(cls, data: dict):
        """
        Создает экземпляр команды из словаря.
        :param data: Словарь с параметрами команды.
        :return: Экземпляр команды.
        """
        return cls(
            name=data["name"], 
            counter=data["counter"], 
            current_step=data["current_step"],
            is_special_command=data.get("is_special_command", False),
        )

    def __repr__(self):
        """
        Представление команды в строковом формате для отладки.
        """
        return f"{self.__class__.__name__}(name={self.name}, counter={self.counter}, complete={self.complete})"
