from __future__ import annotations

from os import PathLike
from typing import Any, Callable

from pymavlink import mavutil


def mavlink_connection(
    device: str | PathLike[str],
    baud: int = 115200,
    source_system: int = 255,
    source_component: int = 0,
    planner_format: str | None = None,
    write: bool = False,
    append: bool = False,
    robust_parsing: bool = True,
    notimestamps: bool = False,
    input: bool = True,
    dialect: str | None = None,
    autoreconnect: bool = False,
    zero_time_base: bool = False,
    retries: int = 3,
    use_native: bool = mavutil.default_native,
    force_connected: bool = False,
    progress_callback: Callable[..., Any] | None = None,
    udp_timeout: float = 0,
    **opts: Any,
) -> mavutil.mavfile:
    """
    Создаёт MAVLink-соединение или открывает лог/источник телеметрии через
    `pymavlink.mavutil.mavlink_connection`.

    Функция является типизированной обёрткой над `mavutil.mavlink_connection()`
    и поддерживает разные типы источников данных:
    - последовательный порт;
    - TCP-клиент;
    - TCP-сервер;
    - UDP-входящий/исходящий сокет;
    - UDP broadcast/multicast;
    - WebSocket;
    - MAVLink-логи и DataFlash-логи;
    - CSV-источники.

    Параметры:
        device:
            Источник подключения или путь к файлу.

            Поддерживаемые варианты:
            - имя последовательного порта:
              `COM3`, `/dev/ttyUSB0`, `/dev/serial0`;
            - TCP-клиент:
              `tcp:127.0.0.1:5760`;
            - TCP-сервер:
              `tcpin:0.0.0.0:5760`;
            - UDP, универсальный режим:
              `udp:127.0.0.1:14550`;
            - UDP входящий:
              `udpin:0.0.0.0:14550`;
            - UDP исходящий:
              `udpout:192.168.1.10:14550`;
            - UDP broadcast:
              `udpbcast:255.255.255.255:14550`;
            - multicast:
              `mcast:239.255.145.50:14550`;
            - WebSocket сервер:
              `wsserver:0.0.0.0:8080`;
            - WebSocket клиент:
              `ws://host:port`, `wss://host:port`;
            - путь к лог-файлу:
              `flight.tlog`, `data.bin`, `data.px4log`, `data.log`;
            - CSV-источник:
              `csv:path/to/file.csv[:key=value...]`.

            Допускается передавать как строку, так и объект `PathLike[str]`.
            Внутри значение будет приведено к строке через `str(device)`.

        baud:
            Скорость последовательного порта в бодах.
            Используется только для serial-подключений.

            Типичные значения:
            - `57600`
            - `115200`
            - `230400`
            - `460800`
            - `921600`

        source_system:
            MAVLink System ID отправителя.
            Это идентификатор системы, от имени которой будут отправляться
            сообщения в сеть MAVLink.

            Типичные значения:
            - `1` — автопилот;
            - `255` — наземная станция / companion computer.

        source_component:
            MAVLink Component ID отправителя.

            Примеры:
            - `0` — общий компонент / значение по умолчанию;
            - `1` — автопилот;
            - другие значения — камера, payload, companion и т.д.

        planner_format:
            Формат логов/совместимости с planner-specific представлением.
            В большинстве случаев оставляется `None`.

        write:
            Если `True`, соединение/лог открывается в режиме записи.
            Обычно используется при работе с лог-файлами.

        append:
            Если `True`, данные будут дописываться в существующий файл,
            а не перезаписывать его. Имеет смысл только вместе с `write=True`.

        robust_parsing:
            Если `True`, включает более устойчивый разбор MAVLink-пакетов.
            Полезно при нестабильной связи или повреждённых логах.

        notimestamps:
            Если `True`, отключает использование временных меток при работе
            с логами. Обычно не требуется и оставляется `False`.

        input:
            Направление работы UDP для формата `udp:...`.

            Значения:
            - `True` — входящий режим, сокет слушает пакеты;
            - `False` — исходящий режим, сокет отправляет пакеты.

            Для `udpin:` и `udpout:` обычно задаётся автоматически самим
            префиксом `device`.

        dialect:
            MAVLink dialect, который нужно использовать.

            Типичные значения:
            - `common`
            - `ardupilotmega`

            Может использоваться и кастомный диалект, если он доступен в
            окружении `pymavlink`.

        autoreconnect:
            Если `True`, библиотека будет пытаться автоматически
            переподключаться после разрыва соединения.
            Особенно полезно для serial и TCP.

        zero_time_base:
            Если `True`, при чтении логов временная шкала будет начинаться с нуля.
            Полезно для офлайн-анализа и воспроизведения.

        retries:
            Количество попыток повторного подключения или инициализации
            для поддерживаемых транспортов.

        use_native:
            Флаг использования нативной реализации парсинга MAVLink.
            Обычно берётся из `mavutil.default_native`.

            Значения:
            - `True` — использовать нативную реализацию;
            - `False` — использовать Python-реализацию.

        force_connected:
            Если `True`, соединение принудительно переводится в режим
            «считать подключённым». В исходной реализации это также
            подразумевает `autoreconnect=True`.

            Используется в специфических сценариях, когда требуется
            агрессивное удержание соединения.

        progress_callback:
            Callback-функция для отслеживания прогресса при чтении больших
            логов или файлов. Сигнатура зависит от конкретного reader’а.

        udp_timeout:
            Таймаут UDP в секундах.

            Значения:
            - `0` — без таймаута;
            - положительное число — ограничение ожидания пакетов.

        **opts:
            Дополнительные параметры, которые будут переданы дальше в
            `mavutil.mavlink_connection()` или в соответствующий reader/backend.

            Часто используется для специальных настроек CSV reader’ов
            и других нестандартных сценариев.

    Возвращает:
        `mavutil.mavfile`:
            Объект подключения/источника данных, через который можно читать
            и отправлять MAVLink-сообщения.

    Примеры:
        Подключение к serial:
        ```python
        master = mavlink_connect("/dev/ttyUSB0", baud=921600)
        ```

        Подключение к PX4/ArduPilot SITL по UDP:
        ```python
        master = mavlink_connect("udp:127.0.0.1:14550")
        ```

        Входящий UDP-сокет:
        ```python
        master = mavlink_connect("udpin:0.0.0.0:14550")
        ```

        TCP-клиент:
        ```python
        master = mavlink_connect("tcp:127.0.0.1:5760", autoreconnect=True)
        ```

        Чтение лог-файла:
        ```python
        master = mavlink_connect("flight.tlog", zero_time_base=True)
        ```
    """
    return mavutil.mavlink_connection(
        device=str(device),
        baud=baud,
        source_system=source_system,
        source_component=source_component,
        planner_format=planner_format,
        write=write,
        append=append,
        robust_parsing=robust_parsing,
        notimestamps=notimestamps,
        input=input,
        dialect=dialect,
        autoreconnect=autoreconnect,
        zero_time_base=zero_time_base,
        retries=retries,
        use_native=use_native,
        force_connected=force_connected,
        progress_callback=progress_callback,
        udp_timeout=udp_timeout,
        **opts,
    )