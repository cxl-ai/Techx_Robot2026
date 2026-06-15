"""Device readiness checks for camera and GMK network link."""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from utils.logger import get_logger

log = get_logger(__name__)


@dataclass
class DeviceStatus:
    name: str
    ready: bool
    detail: str = ""
    last_check: float = 0.0


class DeviceMonitor:
    """Small non-blocking device monitor used before starting the pipeline."""

    def __init__(self):
        self._status: Dict[str, DeviceStatus] = {}
        self._callbacks: Dict[str, List[Callable]] = {}
        self._wired_iface: Optional[str] = None

    def check_camera(self) -> DeviceStatus:
        now = time.time()
        found: List[str] = []

        if sys.platform.startswith("linux"):
            try:
                found.extend(f"/dev/{v}" for v in os.listdir("/dev") if v.startswith("video"))
            except OSError:
                pass

        orbbec_usb = False
        try:
            result = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=3)
            if "Orbbec" in result.stdout or "2bc5" in result.stdout:
                orbbec_usb = True
                found.append("Orbbec Gemini 335L (USB)")
        except Exception:
            pass

        sdk_ok = False
        try:
            import pyorbbecsdk  # noqa: F401
            sdk_ok = True
        except ImportError:
            pass

        ready = bool(found) or sdk_ok
        detail = ", ".join(found) if found else "无相机设备"
        detail += " [Orbbec SDK OK]" if sdk_ok else " [UVC/SDK fallback possible]"
        if orbbec_usb and not sdk_ok:
            detail += " [USB found but SDK import failed]"

        status = DeviceStatus("camera", ready, detail, now)
        self._status["camera"] = status
        return status

    def wait_for_camera(self, timeout: float = 60.0, interval: float = 2.0) -> bool:
        log.info("等待相机就绪 (超时: %ss)...", timeout if timeout > 0 else "无限")
        start = time.time()
        while True:
            status = self.check_camera()
            if status.ready:
                log.info("相机就绪: %s", status.detail)
                return True
            if timeout > 0 and time.time() - start > timeout:
                log.error("相机检测超时: %s", status.detail)
                return False
            time.sleep(interval)

    def _find_wired_iface(self) -> Optional[str]:
        if self._wired_iface:
            return self._wired_iface

        try:
            import netifaces
            for iface in netifaces.interfaces():
                if iface.startswith(("en", "eth")) and iface != "lo":
                    self._wired_iface = iface
                    return iface
        except ImportError:
            pass

        try:
            result = subprocess.run(["ip", "-br", "link", "show"], capture_output=True, text=True, timeout=5)
            for line in result.stdout.splitlines():
                parts = line.split()
                if parts:
                    name = parts[0]
                    if name.startswith(("en", "eth")) and name != "lo":
                        self._wired_iface = name
                        return name
        except Exception:
            pass
        return None

    def check_network(self, target_ip: str = "", target_port: int = 0) -> DeviceStatus:
        """Return ready only when the wired interface has an IP and the target route is usable.

        UDP cannot prove that the remote application is listening, but a connect error or
        missing local wired IP must not be treated as ready. GMK liveness is still verified
        by the receiver using V2 packet timeout/sequence checks.
        """
        now = time.time()
        details: List[str] = []

        iface = self._find_wired_iface()
        if not iface:
            status = DeviceStatus("network", False, "未找到有线网口 (eno*/eth*)", now)
            self._status["network"] = status
            return status
        details.append(f"接口={iface}")

        try:
            result = subprocess.run(["ip", "-4", "addr", "show", iface], capture_output=True, text=True, timeout=5)
            ip = ""
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.startswith("inet "):
                    ip = line.split()[1].split("/")[0]
                    break
            if not ip:
                status = DeviceStatus("network", False, f"{iface} 无 IPv4 地址", now)
                self._status["network"] = status
                return status
            details.append(f"IP={ip}")
        except Exception as e:
            status = DeviceStatus("network", False, f"无法获取 {iface} 状态: {e}", now)
            self._status["network"] = status
            return status

        if target_ip:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(1.0)
                sock.connect((target_ip, target_port or 12345))
                sock.close()
                details.append(f"GMK={target_ip} route OK")
            except OSError as e:
                status = DeviceStatus("network", False, f"{', '.join(details)} | GMK={target_ip} 不可达: {e}", now)
                self._status["network"] = status
                return status

        status = DeviceStatus("network", True, ", ".join(details), now)
        self._status["network"] = status
        return status

    def wait_for_network(self, target_ip: str = "", target_port: int = 0, timeout: float = 30.0, interval: float = 2.0) -> bool:
        log.info("等待网络就绪 (超时: %ss)...", timeout if timeout > 0 else "无限")
        start = time.time()
        while True:
            status = self.check_network(target_ip, target_port)
            if status.ready:
                log.info("网络就绪: %s", status.detail)
                return True
            if timeout > 0 and time.time() - start > timeout:
                log.warning("网络检测超时: %s", status.detail)
                return False
            time.sleep(interval)

    def check_all(self, target_ip: str = "", target_port: int = 0) -> Dict[str, DeviceStatus]:
        return {
            "camera": self.check_camera(),
            "network": self.check_network(target_ip, target_port),
        }

    def on_change(self, device: str, callback: Callable[[DeviceStatus], None]) -> None:
        self._callbacks.setdefault(device, []).append(callback)

    def get_status(self, device: str) -> Optional[DeviceStatus]:
        return self._status.get(device)

    @property
    def all_ready(self) -> bool:
        return bool(self._status) and all(s.ready for s in self._status.values())


if __name__ == "__main__":
    monitor = DeviceMonitor()
    print(monitor.check_all())
