from __future__ import annotations

from typing import TYPE_CHECKING

from multi_drone_core.commands.base_command import BaseCommand

if TYPE_CHECKING:
    from multi_drone_core.controllers.base_controller import BaseController


class M02_Continue(BaseCommand):
    """
    Команда продолжения выполнения после STOP.

    Если активная команда была помечена как прерванная, снимает флаг interrupt.
    Команда служебная и завершается сразу после обработки.
    """

    def __init__(self, counter: int = 0):
        super().__init__(name="M02", counter=counter, is_special_command=True)
        self.description = "Continue command execution"

    def can_execute(self, controller: "BaseController") -> bool:
        return True

    def execute(self, controller: "BaseController") -> None:
        # commander = controller.commander
        # active_command = commander.active_command

        # if active_command is None:
        #     controller.log_info("M02_Continue: активная команда отсутствует, продолжать нечего.")
        #     self.complete_command()
        #     return

        # if active_command.interrupt:
        #     active_command.interrupt = False
        #     controller.log_info(
        #         f"M02_Continue: выполнение команды {active_command.name} продолжено."
        #     )
        # else:
        #     controller.log_info(
        #         f"M02_Continue: команда {active_command.name} уже выполняется."
        #     )

        # self.complete_command()
        pass

    def is_complete(self, controller: "BaseController") -> bool:
        return self._check_finish()

    def to_dict(self) -> dict:
        return super().to_dict()

    @classmethod
    def from_dict(cls, data: dict) -> "M02_Continue":
        return cls(counter=data.get("counter", 0))

    def __repr__(self) -> str:
        return f"M02_Continue(counter={self.counter}, complete={self.complete})"
