"""Jetson 与 GMK 时钟对齐的时间同步工具。

在 Jetson Orin NX 上，通过 Chrony 将系统时钟与 GMK 主时钟同步。
本模块提供启动时检查，确保在进入比赛流水线之前及时发现时间戳异常。

用法示例：
    from utils.time_sync import check_time_sync
    check_time_sync()   # 若时钟未同步则输出警告日志
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from typing import Optional, Tuple

from utils.logger import get_logger

log = get_logger(__name__)


def _parse_chrony_tracking() -> Optional[dict]:
    """运行 chronyc tracking 命令并解析关键字段。

    返回值：
        成功时返回包含各状态字段的字典，失败返回 None。
    """
    try:
        # 调用 chronyc tracking 获取时间同步状态
        out = subprocess.check_output(
            ["chronyc", "tracking"], stderr=subprocess.DEVNULL, timeout=5
        ).decode("utf-8", errors="replace")
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
        return None

    info: dict = {}
    for line in out.splitlines():
        line = line.strip()
        if ":" not in line:
            continue
        # 按冒号分割为键值对
        key, _, val = line.partition(":")
        key = key.strip().replace(" ", "_").lower()
        val = val.strip()
        # 提取数值部分（去除单位）
        parts = val.split()
        if parts:
            try:
                info[key] = float(parts[0])
            except ValueError:
                info[key] = val
    return info


def check_time_sync() -> Tuple[bool, str]:
    """验证系统时钟是否已同步。

    按优先级依次检查：
      1. Chrony          （Jetson 默认方案）
      2. ntpd            （传统备选）
      3. systemd-timesyncd （systemd 自带方案）

    返回值：
        (ok, summary) —— ok 为 True 表示时间源可信赖；
        summary 为人类可读的同步状态描述。
    """
    # ── 1. Chrony ──
    chrony = _parse_chrony_tracking()
    if chrony:
        ref = chrony.get("reference_id", "unknown")
        offset = chrony.get("last_offset", None)
        stratum = chrony.get("stratum", None)

        issues = []
        # stratum >= 10 说明可能使用的是本地时钟而非远程时钟源
        if stratum is not None and stratum >= 10:
            issues.append(f"stratum={stratum} (>=10, 可能是本地时钟)")
        # 偏移超过 50ms 说明同步质量较差
        if offset is not None and abs(offset) > 0.050:  # 50 毫秒
            issues.append(f"offset={offset*1000:.1f}ms (>50ms)")

        if not issues:
            log.info("Chrony 已同步 — ref=%s offset=%.3fms stratum=%s",
                     ref, (offset or 0) * 1000, stratum)
            return True, f"Chrony OK (ref={ref})"
        else:
            log.warning("Chrony 存在问题: %s", "; ".join(issues))
            return False, f"Chrony: {', '.join(issues)}"

    # ── 2. ntpd ──
    try:
        out = subprocess.check_output(
            ["ntpq", "-c", "rv 0 offset"],
            stderr=subprocess.DEVNULL, timeout=3,
        ).decode("utf-8", errors="replace")
        if "offset=" in out:
            log.info("检测到 ntpd 同步")
            return True, "ntpd OK"
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
        pass

    # ── 3. systemd-timesyncd ──
    try:
        out = subprocess.check_output(
            ["timedatectl", "show"], stderr=subprocess.DEVNULL, timeout=3,
        ).decode("utf-8", errors="replace")
        if "NTPSynchronized=yes" in out:
            log.info("检测到 systemd-timesyncd 同步")
            return True, "systemd-timesyncd OK"
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError):
        pass

    # ── 4. 无任何同步服务 ──
    log.warning(
        "未检测到任何时间同步守护进程！UDP 时间戳将不可靠。 "
        "请安装 chrony: sudo apt install chrony"
    )
    return False, "无时间同步 — 请安装 chrony!"


def format_timestamp(ts: float) -> str:
    """将 float64 时间戳格式化为可显示的字符串。

    参数：
        ts: 系统时间（秒）。

    返回值：
        类 ISO 格式的字符串，精度到微秒。
    """
    if ts <= 0:
        return "N/A"
    # 使用 UTC 时间使显示结果与时区无关
    gm = time.gmtime(ts)
    us = int((ts - int(ts)) * 1_000_000)
    return time.strftime("%H:%M:%S", gm) + f".{us:06d}"


def get_clock_offset_estimate() -> float:
    """返回 Chrony 报告的估计时钟偏移量（秒），不可用时返回 0.0。

    返回值：
        时钟偏移量（秒）。
    """
    chrony = _parse_chrony_tracking()
    if chrony:
        return float(chrony.get("last_offset", 0.0))
    return 0.0
