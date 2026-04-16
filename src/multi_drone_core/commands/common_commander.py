import inspect
import re
import threading
from typing import TYPE_CHECKING

import multi_drone_core.commands.g_command as g_code_module
import multi_drone_core.commands.m_command as m_code_module
import multi_drone_core.commands.p_command as p_code_module
import multi_drone_core.commands.s_command as s_code_module
import multi_drone_core.commands.t_command as t_code_module
from multi_drone_core.commands.base_command import BaseCommand
from multi_drone_core.commands.base_commander import BaseCommander

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class CommonCommander(BaseCommander):
    def __init__(
        self,
        controller: "BaseController",
        execute_command_timer=0.1,
    ):
        super().__init__(controller)

        self.command_classes = {}
        self._register_module_commands(g_code_module)
        self._register_module_commands(m_code_module)
        self._register_module_commands(p_code_module)
        self._register_module_commands(s_code_module)
        self._register_module_commands(t_code_module)

        self.execute_command_timer = float(execute_command_timer)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def _extract_command_name(self, command_class: type[BaseCommand]) -> str | None:
        """
        Пытается получить код команды (например, M23).
        """
        try:
            command = command_class()
            if isinstance(command.name, str) and command.name:
                return command.name
        except Exception:
            pass

        match = re.match(r"^([A-Z]\d{2})_", command_class.__name__)
        if match is not None:
            return match.group(1)
        return None

    def _register_module_commands(self, module) -> None:
        seen_classes = set()

        for _, cls in inspect.getmembers(module, inspect.isclass):
            if cls in seen_classes:
                continue
            seen_classes.add(cls)

            if not issubclass(cls, BaseCommand) or cls == BaseCommand:
                continue
            if inspect.isabstract(cls):
                continue

            command_name = self._extract_command_name(cls)
            if not command_name:
                self.log_warning(
                    f"Пропущена регистрация класса {cls.__name__}: не удалось определить имя команды."
                )
                continue

            if command_name in self.command_classes:
                existed = self.command_classes[command_name].__name__
                self.log_warning(
                    f"Дубликат команды {command_name}: {cls.__name__} пропущен, уже зарегистрирован {existed}."
                )
                continue

            self.command_classes[command_name] = cls

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._execution_loop,
            name="common-commander-loop",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        self._thread = None

    def _execution_loop(self) -> None:
        period = max(0.01, float(self.execute_command_timer))
        while not self._stop_event.is_set():
            try:
                self.execute_command()
            except Exception as exc:
                self.log_error(f"Ошибка цикла команд: {exc}")
            self._stop_event.wait(period)

    def execute_command(self) -> None:
        """
        Выполняет обработку очереди команд.
        """
        if self._execution_paused:
            return

        if self.active_command:
            try:
                if not self.active_command.is_complete(self.controller):
                    self.active_command.safe_execute(self.controller)
                    if not self.active_command.is_complete(self.controller):
                        return
                self.after_command(self.active_command)
                self.active_command = None
            except Exception as e:
                self.log_error(
                    f"Ошибка при проверке завершения команды: {e}"
                )
                self.active_command = None

        if self.command_queue:
            self.active_command = self.get_command()
            self.log_info(
                f"Выполнение команды: {self.active_command.name}"
            )
            self.before_command(self.active_command)
            self.active_command.safe_execute(self.controller)

    def process_new_command(self, data):
        """
        Обработка входящей команды.
        Поддерживает dict с данными команды и готовый объект BaseCommand.
        """
        if isinstance(data, BaseCommand):
            command = data
            self.command_history.append(command.to_dict())
        elif isinstance(data, dict):
            command_name = data.get("name")
            if not command_name:
                self.log_error("Отсутствует имя команды в данных.")
                return

            command_class: BaseCommand = self.command_classes.get(command_name)
            if command_class is None:
                self.log_error(f"Неизвестная команда: {command_name}")
                return

            command = command_class.from_dict(data)
            
            if not command.is_ready:
                raise RuntimeError(
                    f"Команда {command_name} не готова к выполнению: is_ready=False."
                )
            
            self.command_history.append(data)
        else:
            self.log_error(
                f"Неподдерживаемый тип команды: {type(data).__name__}. "
                "Ожидается dict или BaseCommand."
            )
            return

        if command.is_special_command:
            self.execute_special_command(command)
        else:
            self.add_command(command)

    def before_command(self, command):
        self.log_info(f"Команда {command.name} начата.")

    def after_command(self, command: BaseCommand) -> None:
        if command._check_complete():
            self.log_info(f"Команда {command.name} завершена успешно.")
        if command._check_interrupt():
            self.log_warning(f"Команда {command.name} прервана. Очередь очищена.")
            self.clear_command_queue()

    def execute_special_command(self, command: BaseCommand) -> None:
        self.before_command(command)
        command.safe_execute(self.controller)
        self.after_command(command)
        
