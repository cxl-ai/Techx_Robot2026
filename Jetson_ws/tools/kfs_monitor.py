#!/usr/bin/env python3
"""KFS 数据下发监听器 —— 实时监控 UDP 29 字节帧数据。

监听 TECHx_vision 流水线发送的 UDP 数据包，实时解析并展示：
  - 帧头魔数 (Magic: 0x55AA)
  - 序列号 (Sequence)
  - 相机硬件时间戳 (Timestamp)
  - 追踪 ID (Track ID)
  - 相机坐标系 3D 坐标 (Xc, Yc, Zc) — 单位：米
  - CRC16-CCITT 校验结果
  - 数据统计（帧率、丢包率、坐标范围）

用法:
    # 1. 监听本地 12345 端口（默认）
    python tools/kfs_monitor.py

    # 2. 监听指定 IP:Port
    python tools/kfs_monitor.py --ip 0.0.0.0 --port 12345

    # 3. 输出 JSON 格式（方便日志记录）
    python tools/kfs_monitor.py --json

    # 4. 保存到文件
    python tools/kfs_monitor.py --save kfs_data.csv

协议帧格式（29 字节，小端序）:
  ┌────────┬──────────┬───────────┬──────────┬──────────────────────┬───────┐
  │ Magic  │ Sequence │ Timestamp │ Track ID │  Xc    Yc    Zc      │ CRC16 │
  │ uint16 │  uint32  │  float64  │  uint8   │ f32    f32    f32    │uint16 │
  │ 0x55AA │  自增    │   秒      │  追踪ID  │  米    米    米      │ 校验  │
  │ [0:2]  │  [2:6]   │  [6:14]   │  [14]    │ [15:27]             │[27:29]│
  └────────┴──────────┴───────────┴──────────┴──────────────────────┴───────┘
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import struct
import sys
import time
from collections import deque
from datetime import datetime
from typing import Optional

# ── 协议常量（必须与 communication/udp.py 一致）──
_MAGIC: int = 0x55AA
_FRAME_FMT: str = "<H I d B 3f H"       # 29 字节帧格式
_FRAME_SIZE: int = struct.calcsize(_FRAME_FMT)  # = 29
_PAYLOAD_SIZE: int = 27                          # 校验覆盖前 27 字节

# ── ANSI 颜色 ──
GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
CYAN = "\033[0;36m"
RED = "\033[0;31m"
BOLD = "\033[1m"
NC = "\033[0m"


def crc16_ccitt(data: bytes) -> int:
    """CCITT-CRC16 校验（多项式 0x1021，初始值 0xFFFF）。

    与 communication/udp.py 和 GMK STM32 HAL_CRC 算法一致。
    """
    crc: int = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


class KfsMonitor:
    """KFS 数据下发监听器。"""

    def __init__(self, bind_ip: str = "0.0.0.0", bind_port: int = 12345,
                 json_mode: bool = False, save_path: Optional[str] = None):
        self._addr = (bind_ip, bind_port)
        self._json_mode = json_mode
        self._save_path = save_path
        self._csv_file = None
        self._csv_writer = None

        # 统计信息
        self._total_frames = 0
        self._valid_frames = 0
        self._crc_errors = 0
        self._magic_errors = 0
        self._last_seq: Optional[int] = None
        self._lost_packets = 0
        self._start_time: Optional[float] = None

        # KFS 类别映射
        self._kfs_classes = {
            "fake_kfs": "假目标",
            "r1_kfs_red": "R1红方KFS",
            "r1_kfs_blue": "R1蓝方KFS",
            "r2_kfs_red": "R2红方KFS",
            "r2_kfs_blue": "R2蓝方KFS",
        }

        # 滑动窗口统计（最近 100 帧）
        self._recent_x: deque = deque(maxlen=100)
        self._recent_y: deque = deque(maxlen=100)
        self._recent_z: deque = deque(maxlen=100)
        self._recent_ts: deque = deque(maxlen=100)

    def start(self) -> None:
        """启动监听循环。"""
        # 创建 UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)  # 1 秒超时（允许优雅退出）

        try:
            sock.bind(self._addr)
        except OSError as e:
            print(f"{RED}绑定失败 {self._addr}: {e}{NC}")
            sys.exit(1)

        print(f"{CYAN}{'=' * 60}{NC}")
        print(f"{CYAN}  KFS 数据下发监听器 v1.0{NC}")
        print(f"{CYAN}  监听地址: {self._addr[0]}:{self._addr[1]}{NC}")
        print(f"{CYAN}  帧格式:   {_FRAME_SIZE} 字节 (Magic + Seq + TS + TID + XYZ + CRC16){NC}")
        print(f"{CYAN}  输出模式: {'JSON' if self._json_mode else '人类可读'}{NC}")
        if self._save_path:
            print(f"{CYAN}  保存路径: {self._save_path}{NC}")
        print(f"{CYAN}{'=' * 60}{NC}")

        # 初始化 CSV 写入
        if self._save_path:
            self._csv_file = open(self._save_path, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "sequence", "timestamp", "track_id", "class_name",
                "Xc_m", "Yc_m", "Zc_m", "crc_ok"
            ])

        if not self._json_mode:
            print(f"\n{'Seq':>6} {'时间戳':>16} {'Track':>5} {'类别':>16} "
                  f"{'Xc(m)':>8} {'Yc(m)':>8} {'Zc(m)':>8} {'CRC':>5}")
            print("-" * 85)

        self._start_time = time.time()
        try:
            self._listen_loop(sock)
        except KeyboardInterrupt:
            print(f"\n\n{YELLOW}用户中断{NC}")
        finally:
            sock.close()
            if self._csv_file:
                self._csv_file.close()
            self._print_summary()

    def _listen_loop(self, sock: socket.socket) -> None:
        """主监听循环。"""
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if data:
                    self._process_frame(data, addr)
            except socket.timeout:
                continue  # 静默处理超时
            except OSError as e:
                print(f"{RED}Socket 错误: {e}{NC}")
                break

    def _process_frame(self, data: bytes, addr: tuple) -> None:
        """处理单个收到的数据帧。"""
        self._total_frames += 1

        # 长度检查
        if len(data) != _FRAME_SIZE:
            if self._total_frames <= 3:
                print(f"{YELLOW}[警告] 收到非标准长度帧: {len(data)} 字节 (期望 {_FRAME_SIZE}){NC}")
            return

        # 解包
        try:
            magic, seq, ts, tid, xc, yc, zc, crc_received = struct.unpack(_FRAME_FMT, data)
        except struct.error as e:
            print(f"{RED}[错误] 解包失败: {e}{NC}")
            return

        # 魔数检查
        if magic != _MAGIC:
            self._magic_errors += 1
            if self._magic_errors <= 1:
                print(f"{YELLOW}[警告] 帧头魔数不匹配: 0x{magic:04X} (期望 0x{_MAGIC:04X}){NC}")
            return

        # CRC16 校验
        payload = data[:27]
        crc_calc = crc16_ccitt(payload)
        crc_ok = (crc_calc == crc_received)

        if crc_ok:
            self._valid_frames += 1
        else:
            self._crc_errors += 1
            if self._crc_errors <= 1:
                print(f"{RED}[错误] CRC16 校验失败: 收到=0x{crc_received:04X} 计算=0x{crc_calc:04X}{NC}")
            return  # CRC 失败不继续处理

        # 序列号丢包检测
        if self._last_seq is not None:
            expected = (self._last_seq + 1) & 0xFFFFFFFF
            if seq != expected:
                lost = (seq - expected) & 0xFFFFFFFF
                self._lost_packets += lost
        self._last_seq = seq

        # 滑动窗口统计
        self._recent_x.append(xc)
        self._recent_y.append(yc)
        self._recent_z.append(zc)
        self._recent_ts.append(ts)

        # 解析类别名称
        class_name = f"Track-{tid}"  # 默认
        # 尝试匹配 KFS 类别
        for k, v in self._kfs_classes.items():
            if k in str(tid):
                class_name = v
                break

        # 转换时间戳为可读格式
        try:
            dt = datetime.fromtimestamp(ts)
            ts_str = dt.strftime("%H:%M:%S") + f".{int((ts % 1) * 1000):03d}"
        except (ValueError, OSError):
            ts_str = f"{ts:.3f}"

        # 输出
        if self._json_mode:
            self._output_json(seq, ts, tid, class_name, xc, yc, zc, crc_ok)
        else:
            self._output_text(seq, ts_str, tid, class_name, xc, yc, zc, crc_ok)

        # 保存到 CSV
        if self._csv_writer:
            self._csv_writer.writerow([seq, ts, tid, class_name, xc, yc, zc, crc_ok])

    def _output_text(self, seq: int, ts_str: str, tid: int, class_name: str,
                     xc: float, yc: float, zc: float, crc_ok: bool) -> None:
        """人类可读格式输出。"""
        crc_str = f"{GREEN}✓{NC}" if crc_ok else f"{RED}✗{NC}"
        print(f"{seq:>6} {ts_str:>16} {tid:>5} {class_name:>16} "
              f"{xc:>8.3f} {yc:>8.3f} {zc:>8.3f} {crc_str}")

        # 每 50 帧打印一次简要统计
        if self._valid_frames > 0 and self._valid_frames % 50 == 0:
            self._print_inline_stats()

    def _output_json(self, seq: int, ts: float, tid: int, class_name: str,
                     xc: float, yc: float, zc: float, crc_ok: bool) -> None:
        """JSON 格式输出。"""
        record = {
            "sequence": seq,
            "timestamp": ts,
            "track_id": tid,
            "class_name": class_name,
            "Xc_m": round(xc, 4),
            "Yc_m": round(yc, 4),
            "Zc_m": round(zc, 4),
            "crc_ok": crc_ok,
            "time": datetime.fromtimestamp(ts).isoformat() if 0 < ts < 9999999999 else "invalid",
        }
        print(json.dumps(record, ensure_ascii=False))

    def _print_inline_stats(self) -> None:
        """打印内联统计信息（不中断数据流）。"""
        if not self._recent_ts:
            return
        elapsed = time.time() - self._start_time if self._start_time else 1
        fps = self._valid_frames / elapsed if elapsed > 0 else 0
        print(f"{YELLOW}  ── [统计] {self._valid_frames}帧 | "
              f"{fps:.1f} FPS | 丢包:{self._lost_packets} | "
              f"CRC错:{self._crc_errors} | 魔数错:{self._magic_errors} "
              f"──{NC}")

    def _print_summary(self) -> None:
        """打印最终汇总统计。"""
        elapsed = time.time() - self._start_time if self._start_time else 1
        fps = self._valid_frames / elapsed if elapsed > 0 else 0

        print(f"\n{CYAN}{'=' * 60}{NC}")
        print(f"{BOLD}  KFS 监听统计汇总{NC}")
        print(f"{CYAN}{'=' * 60}{NC}")
        print(f"  监听时长:       {elapsed:.1f} 秒")
        print(f"  总收到帧数:     {self._total_frames}")
        print(f"  有效帧数:       {self._valid_frames}")
        print(f"  平均帧率:       {fps:.1f} FPS")
        print(f"  CRC 校验错误:   {self._crc_errors}")
        print(f"  魔数不匹配:     {self._magic_errors}")
        print(f"  累计丢包:       {self._lost_packets}")

        if self._recent_x:
            print(f"\n  坐标范围统计（最近 {len(self._recent_x)} 帧）:")
            print(f"    Xc: [{min(self._recent_x):.3f}, {max(self._recent_x):.3f}] m")
            print(f"    Yc: [{min(self._recent_y):.3f}, {max(self._recent_y):.3f}] m")
            print(f"    Zc: [{min(self._recent_z):.3f}, {max(self._recent_z):.3f}] m")

        print(f"\n  🟢 KFS 数据下发状态: ", end="")
        if self._valid_frames > 0:
            print(f"{GREEN}正常 — 已检测到 {self._valid_frames} 帧有效数据{NC}")
        elif self._total_frames > 0:
            print(f"{YELLOW}异常 — 收到 {self._total_frames} 帧但全部校验失败{NC}")
        else:
            print(f"{RED}无数据 — 未收到任何 UDP 数据帧{NC}")
        print(f"{CYAN}{'=' * 60}{NC}")


# ── 命令行入口 ──

def main() -> None:
    parser = argparse.ArgumentParser(
        description="KFS 数据下发监听器 — 实时监控 TECHx_vision UDP 29 字节帧"
    )
    parser.add_argument("--ip", type=str, default="0.0.0.0",
                        help="监听 IP 地址（默认: 0.0.0.0）")
    parser.add_argument("--port", type=int, default=12345,
                        help="监听端口号（默认: 12345）")
    parser.add_argument("--json", action="store_true",
                        help="以 JSON 格式输出")
    parser.add_argument("--save", type=str, default=None,
                        help="保存数据到 CSV 文件")
    args = parser.parse_args()

    monitor = KfsMonitor(
        bind_ip=args.ip,
        bind_port=args.port,
        json_mode=args.json,
        save_path=args.save,
    )
    monitor.start()


if __name__ == "__main__":
    main()
