from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from logging.handlers import RotatingFileHandler


DEFAULT_MAX_BYTES = 10 * 1024 * 1024
DEFAULT_BACKUP_COUNT = 5


@dataclass(slots=True)
class CoreLoggers:
    general: logging.Logger
    backend: logging.Logger
    commander: logging.Logger
    controller: logging.Logger


def _build_formatter() -> logging.Formatter:
    return logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def _ensure_file_handler(
    logger: logging.Logger,
    file_path: Path,
    *,
    max_bytes: int,
    backup_count: int,
) -> None:
    target = str(file_path.resolve())
    for handler in logger.handlers:
        if isinstance(handler, RotatingFileHandler):
            existing = getattr(handler, "baseFilename", None)
            if existing and Path(existing).resolve() == Path(target):
                return

    file_handler = RotatingFileHandler(
        file_path,
        maxBytes=max_bytes,
        backupCount=backup_count,
        encoding="utf-8",
    )
    file_handler.setFormatter(_build_formatter())
    logger.addHandler(file_handler)


def _build_logger(name: str, level: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(level.upper())
    logger.propagate = False
    return logger


def _safe_logger(name: str, level: str) -> logging.Logger:
    logger = _build_logger(name, level)
    if not logger.handlers:
        logger.addHandler(logging.NullHandler())
    return logger


def get_core_loggers(
    *,
    node_id: str,
    log_dir: Path = Path("logs"),
    log_level: str = "INFO",
    max_bytes: int = DEFAULT_MAX_BYTES,
    backup_count: int = DEFAULT_BACKUP_COUNT,
) -> CoreLoggers:
    """
    Возвращает набор логгеров core:
    - общий логгер в logs/core.log
    - отдельные channel-логгеры в logs/core.<channel>.log

    Channel-логгеры пишут только в свои файлы (propagate=False).
    """
    node_id = node_id.strip()
    if not node_id:
        raise ValueError("node_id must not be empty")

    try:
        log_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        null = _safe_logger(f"core.{node_id}", log_level)
        return CoreLoggers(
            general=null,
            backend=null,
            commander=null,
            controller=null,
        )

    general = _build_logger(f"core.{node_id}", log_level)
    backend = _build_logger(f"core.{node_id}.backend", log_level)
    commander = _build_logger(f"core.{node_id}.commander", log_level)
    controller = _build_logger(f"core.{node_id}.controller", log_level)

    _ensure_file_handler(
        general,
        log_dir / "core.log",
        max_bytes=max_bytes,
        backup_count=backup_count,
    )
    _ensure_file_handler(
        backend,
        log_dir / "backend.log",
        max_bytes=max_bytes,
        backup_count=backup_count,
    )
    _ensure_file_handler(
        commander,
        log_dir / "commander.log",
        max_bytes=max_bytes,
        backup_count=backup_count,
    )
    _ensure_file_handler(
        controller,
        log_dir / "controller.log",
        max_bytes=max_bytes,
        backup_count=backup_count,
    )

    return CoreLoggers(
        general=general,
        backend=backend,
        commander=commander,
        controller=controller,
    )

def get_logger(
    name: str,
    file_path: Path,
    *,
    log_level: str = "INFO",
    max_bytes: int = DEFAULT_MAX_BYTES,
    backup_count: int = DEFAULT_BACKUP_COUNT,
) -> logging.Logger:
    logger = _build_logger(name, log_level)

    try:
        file_path.parent.mkdir(parents=True, exist_ok=True)
        _ensure_file_handler(
            logger,
            file_path,
            max_bytes=max_bytes,
            backup_count=backup_count,
        )
    except Exception:
        return _safe_logger(name, log_level)

    return logger
