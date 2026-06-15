#!/usr/bin/env python3
"""Summarize one TECHX Jetson runtime log file.

Default input is logs/techx_current.log. This tool keeps debugging simple: one
runtime log file in, one compact diagnosis summary out.
"""

from __future__ import annotations

import argparse
import os
import re
from collections import deque
from typing import Iterable, List

IMPORTANT_PATTERNS = [
    "TECHX_vision 启动",
    "日志系统初始化完成",
    "日志策略:",
    "配置:",
    "推理调参:",
    "输出策略:",
    "模型加入组合检测器",
    "YOLO loaded",
    "YOLO class map",
    "Orbbec started",
    "UDP sender ready",
]

SECTION_PATTERNS = {
    "recent_health": "HEALTH ",
    "recent_yolo_diag": "YOLO boxes diag",
    "recent_output_filter": "OUTPUT filter",
    "recent_udp": "UDP V2",
}

WARN_RE = re.compile(r"\[(WARNING|ERROR|CRITICAL)\s*\]|运行保护触发|异常|失败|ERROR|CRITICAL")


def tail_matches(lines: Iterable[str], pattern: str, limit: int) -> List[str]:
    q: deque[str] = deque(maxlen=limit)
    for line in lines:
        if pattern in line:
            q.append(line.rstrip())
    return list(q)


def collect_sections(path: str, limit: int) -> dict:
    sections = {name: deque(maxlen=limit) for name in SECTION_PATTERNS}
    important = deque(maxlen=80)
    warnings = deque(maxlen=80)
    total = 0
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            total += 1
            line = raw.rstrip()
            if any(p in line for p in IMPORTANT_PATTERNS):
                important.append(line)
            if WARN_RE.search(line):
                warnings.append(line)
            for name, pattern in SECTION_PATTERNS.items():
                if pattern in line:
                    sections[name].append(line)
    return {
        "total": total,
        "important": list(important),
        "warnings": list(warnings),
        **{name: list(buf) for name, buf in sections.items()},
    }


def print_block(title: str, lines: List[str]) -> None:
    print(f"\n=== {title} ===")
    if not lines:
        print("(none)")
        return
    for line in lines:
        print(line)


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize TECHX Jetson single runtime log")
    parser.add_argument("log", nargs="?", default="logs/techx_current.log", help="runtime log path")
    parser.add_argument("--limit", type=int, default=12, help="lines per recent section")
    args = parser.parse_args()

    path = os.path.abspath(args.log)
    if not os.path.exists(path):
        print(f"log not found: {path}")
        print("start once with: TECHX_GUI=1 ./start_jetson.sh")
        return 2

    data = collect_sections(path, max(1, args.limit))
    print("TECHX Jetson log summary")
    print(f"log: {path}")
    print(f"lines: {data['total']}")

    print_block("startup / config", data["important"][-args.limit:])
    print_block("warnings / errors", data["warnings"][-args.limit:])
    print_block("health", data["recent_health"])
    print_block("yolo boxes", data["recent_yolo_diag"])
    print_block("output filter", data["recent_output_filter"])
    print_block("udp", data["recent_udp"])

    print("\nDiagnosis hints:")
    print("- many YOLO raw/kept/final boxes: inspect conf threshold, cross-class dedup, or model artifact")
    print("- OUTPUT raw>0 sent=0 reject_depth>0: detection works but depth is invalid")
    print("- HEALTH avg_infer_ms high or infer_drop>0: inference is the bottleneck")
    print("- HEALTH avg_display_ms high: UI rendering is the bottleneck")
    print("- UDP seq increases but GMK has no /frame: check network/GMK bridge")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
