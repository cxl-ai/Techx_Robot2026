"""TECHx_vision 集中式日志系统。

默认只写一个主日志文件，便于 Jetson 现场排故：
    logs/techx_current.log

每次启动默认覆盖这个文件，避免 logs/ 目录堆满很多 techx_时间戳.log。
需要保留历史时可设置：
    TECHX_LOG_HISTORY=1
需要追加而不是覆盖时可设置：
    TECHX_LOG_APPEND=1
需要同时进 journalctl/syslog 时可设置：
    TECHX_SYSLOG=1
"""

from __future__ import annotations

import logging
import os
import sys
import warnings
from datetime import datetime
from logging.handlers import SysLogHandler
from typing import Optional

_log_initialised: bool = False
_log_file_path: Optional[str] = None


class _ColourFormatter(logging.Formatter):
    """轻量级彩色日志输出 —— 无外部依赖。"""

    COLOURS = {
        logging.DEBUG: "\033[36m",
        logging.INFO: "\033[32m",
        logging.WARNING: "\033[33m",
        logging.ERROR: "\033[31m",
        logging.CRITICAL: "\033[1;31m",
    }
    RESET = "\033[0m"

    def format(self, record: logging.LogRecord) -> str:
        orig_level = record.levelname
        orig_name = record.name
        record.levelname = f"{self.COLOURS.get(record.levelno, '')}{record.levelname}{self.RESET}"
        record.name = f"\033[1m{record.name}\033[0m"
        try:
            return super().format(record)
        finally:
            record.levelname = orig_level
            record.name = orig_name


class _SyslogFormatter(logging.Formatter):
    """syslog/journalctl 使用的短格式。"""

    def format(self, record: logging.LogRecord) -> str:
        return f"{record.levelname} {record.name}: {record.getMessage()}"


def setup_logging(
    level: int = logging.INFO,
    log_dir: Optional[str] = None,
    console: bool = True,
    syslog: Optional[bool] = None,
) -> str:
    """在启动时进行一次性日志配置。

    默认行为：
      - 写入一个固定文件 logs/techx_current.log
      - 每次启动覆盖，保证现场只看一个文件即可
      - 控制台同步输出
      - syslog 默认关闭，除非 TECHX_SYSLOG=1
      - 第三方 warning/error 也尽量集中写入同一个文件
    """
    global _log_initialised, _log_file_path

    if _log_initialised:
        return _log_file_path or ""

    techx_logger = logging.getLogger("techx")
    techx_logger.setLevel(logging.DEBUG)
    techx_logger.handlers.clear()
    techx_logger.propagate = False

    file_fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d [%(levelname)-8s] %(name)-22s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    if log_dir is None:
        log_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "logs",
        )
    os.makedirs(log_dir, exist_ok=True)

    explicit_file = os.getenv("TECHX_LOG_FILE", "").strip()
    if explicit_file:
        _log_file_path = explicit_file if os.path.isabs(explicit_file) else os.path.join(log_dir, explicit_file)
    elif os.getenv("TECHX_LOG_HISTORY", "0") == "1":
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        _log_file_path = os.path.join(log_dir, f"techx_{stamp}.log")
    else:
        _log_file_path = os.path.join(log_dir, "techx_current.log")

    os.makedirs(os.path.dirname(os.path.abspath(_log_file_path)), exist_ok=True)
    mode = "a" if os.getenv("TECHX_LOG_APPEND", "0") == "1" else "w"
    fh = logging.FileHandler(_log_file_path, mode=mode, encoding="utf-8")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(file_fmt)
    techx_logger.addHandler(fh)

    if console:
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(level)
        ch.setFormatter(_ColourFormatter(
            "%(asctime)s.%(msecs)03d [%(levelname)-27s] %(name)-22s | %(message)s",
            datefmt="%H:%M:%S",
        ))
        techx_logger.addHandler(ch)
    else:
        ch = None

    enable_syslog = bool(os.getenv("TECHX_SYSLOG", "0") == "1") if syslog is None else bool(syslog)
    if enable_syslog:
        _attach_syslog_handler(techx_logger)

    _attach_external_warning_capture(fh)

    _log_initialised = True
    techx_logger.info("日志系统初始化完成 → %s", _log_file_path)
    techx_logger.info("日志策略: single_file=%s mode=%s history=%s syslog=%s", _log_file_path, mode, os.getenv("TECHX_LOG_HISTORY", "0"), "1" if enable_syslog else "0")
    if enable_syslog:
        techx_logger.info("系统日志已启用，可用 journalctl -t techx_vision -f 查看")
    return _log_file_path


def _attach_external_warning_capture(file_handler: logging.Handler) -> None:
    """Route Python warnings and third-party WARNING+ logs into the same file.

    This keeps the normal console readable while preserving important environment
    warnings such as torch/torchvision mismatch in logs/techx_current.log.
    """
    if os.getenv("TECHX_CAPTURE_EXTERNAL_LOGS", "1") != "1":
        return
    try:
        logging.captureWarnings(True)
        warnings.simplefilter("default")
        root_logger = logging.getLogger()
        root_logger.setLevel(logging.WARNING)
        if not any(getattr(h, "baseFilename", None) == getattr(file_handler, "baseFilename", None) for h in root_logger.handlers):
            root_logger.addHandler(file_handler)
    except Exception:
        # Logging setup must never prevent robot startup.
        return


def _attach_syslog_handler(root: logging.Logger) -> None:
    """Try to attach Linux syslog without breaking normal file logging."""
    address = "/dev/log" if os.path.exists("/dev/log") else ("localhost", 514)
    try:
        sh = SysLogHandler(address=address)
        sh.setLevel(logging.INFO)
        sh.ident = "techx_vision "
        sh.setFormatter(_SyslogFormatter())
        root.addHandler(sh)
    except Exception as exc:
        root.warning("syslog handler 初始化失败，继续只写文件日志: %s", exc)


def get_logger(name: str) -> logging.Logger:
    """获取指定名称的 logger 实例。"""
    if not name.startswith("techx."):
        name = f"techx.{name}"
    return logging.getLogger(name)
