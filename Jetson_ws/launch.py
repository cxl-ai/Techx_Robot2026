#!/usr/bin/env python3
"""Preflight launcher for TECHX_vision."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Set


EXPECTED_GLOBAL_IDS: Dict[int, str] = {
    0: "fake_kfs",
    1: "r1_kfs_red",
    2: "r1_kfs_blue",
    3: "r2_kfs_red",
    4: "r2_kfs_blue",
    100: "weapon_head_fist",
    101: "weapon_head_palm",
    102: "weapon_head_spear",
    200: "qr_code",
}


def _supports_colour() -> bool:
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


_HAS_COLOUR = _supports_colour()
RED = "\033[0;31m" if _HAS_COLOUR else ""
GREEN = "\033[0;32m" if _HAS_COLOUR else ""
YELLOW = "\033[1;33m" if _HAS_COLOUR else ""
CYAN = "\033[0;36m" if _HAS_COLOUR else ""
BOLD = "\033[1m" if _HAS_COLOUR else ""
NC = "\033[0m" if _HAS_COLOUR else ""


def _ok(msg: str) -> None:
    print(f"  {GREEN}✓{NC} {msg}")


def _warn(msg: str) -> None:
    print(f"  {YELLOW}⚠{NC} {msg}")


def _err(msg: str) -> None:
    print(f"  {RED}✗{NC} {msg}")


def _info(msg: str) -> None:
    print(f"  {CYAN}○{NC} {msg}")


def _hdr(msg: str) -> None:
    print(f"\n{BOLD}{msg}{NC}")


# ASCII status labels keep the Windows GBK console from crashing before
# PYTHONUTF8/chcp settings take effect.
def _ok(msg: str) -> None:
    print(f"  {GREEN}OK{NC} {msg}")


def _warn(msg: str) -> None:
    print(f"  {YELLOW}WARN{NC} {msg}")


def _err(msg: str) -> None:
    print(f"  {RED}ERR{NC} {msg}")


def _info(msg: str) -> None:
    print(f"  {CYAN}INFO{NC} {msg}")


def _mapped_ids(model: dict) -> Set[int]:
    out: Set[int] = set()
    raw = model.get("class_id_map", {}) or {}
    if isinstance(raw, dict):
        for value in raw.values():
            try:
                out.add(int(value))
            except (TypeError, ValueError):
                pass
    return out


def _runtime_bool(config: Optional[dict], key: str, default: bool) -> bool:
    return bool(((config or {}).get("runtime", {}) or {}).get(key, default))


def _resolve_stage() -> str:
    stage = os.environ.get("TECHX_STAGE", "all").strip().lower() or "all"
    return stage if stage in {"all", "head", "gesture", "assembly", "kfs", "qr"} else "all"


def _stage_wants_model(stage: str, model: dict) -> bool:
    if not bool(model.get("enabled", True)):
        return False
    folder = str(model.get("folder", "")).lower()
    name = str(model.get("name", "")).lower()
    if stage == "all":
        return True
    if stage == "head":
        return "head" in folder or "weapon" in name
    if stage == "gesture":
        return "gesture" in folder or "gesture" in name
    if stage == "kfs":
        return "kfs" in folder or "kfs" in name or "gesture" in folder or "gesture" in name
    return False


def _expected_ids_for_stage(stage: str) -> Dict[int, str]:
    if stage == "kfs":
        return {
            cid: name for cid, name in EXPECTED_GLOBAL_IDS.items()
            if 0 <= cid <= 4 or 100 <= cid <= 102
        }
    if stage in {"head", "gesture"}:
        return {cid: name for cid, name in EXPECTED_GLOBAL_IDS.items() if 100 <= cid <= 102}
    if stage == "qr":
        return {200: EXPECTED_GLOBAL_IDS[200]}
    if stage == "assembly":
        return {}
    return EXPECTED_GLOBAL_IDS


class Launcher:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.config_path: Optional[str] = None
        self.config: Optional[dict] = None

    def run(self) -> int:
        t0 = time.monotonic()
        print(f"{CYAN}{'=' * 50}{NC}")
        print(f"{CYAN}  TECHX_vision 一键启动器 v2.6{NC}")
        print(f"{CYAN}  平台: {sys.platform} | Python: {sys.version.split()[0]}{NC}")
        if self.args.fast_start:
            print(f"{CYAN}  模式: fast-start{NC}")
        print(f"{CYAN}  保护: wait_camera={self.args.camera_wait_timeout}s "
              f"no_frame={self.args.no_camera_frame_timeout}s "
              f"no_infer={self.args.no_infer_result_timeout}s{NC}")
        print(f"{CYAN}{'=' * 50}{NC}")

        self._check_python()
        self._check_config()
        self._check_models()

        if not self.args.fast_start:
            self._check_conda()
            self._check_dependencies()
            self._check_camera()
            self._check_time_sync_and_gpu()
        else:
            _info("fast-start: 跳过重型 import/GPU/Chrony 自检")

        elapsed = time.monotonic() - t0
        print(f"\n{CYAN}{'=' * 50}{NC}")
        if self.errors:
            print(f"{RED}  发现 {len(self.errors)} 个错误:{NC}")
            for e in self.errors:
                print(f"    {RED}✗{NC} {e}")
        if self.warnings:
            print(f"{YELLOW}  发现 {len(self.warnings)} 个警告:{NC}")
            for w in self.warnings:
                print(f"    {YELLOW}⚠{NC} {w}")
        if not self.errors:
            print(f"{GREEN}  启动前检查通过 ✓ ({elapsed:.2f}s){NC}")

        if self.args.check_only:
            return 1 if self.errors else 0
        if self.errors:
            print(f"\n{RED}无法启动 — 请先修复以上错误{NC}")
            return 1
        return self._launch()

    def _check_python(self) -> None:
        _hdr("[1/3] Python 环境")
        _ok(f"Python {sys.version.split()[0]}")
        _ok(f"路径: {sys.executable}")

    def _check_conda(self) -> None:
        _hdr("[Full] Conda 环境")
        env = os.environ.get("CONDA_DEFAULT_ENV", "")
        _ok(f"conda 环境: {env}") if env else _info("未使用 conda 环境")

    def _check_dependencies(self) -> None:
        _hdr("[Full] 关键依赖")
        critical = {"ultralytics": "YOLO", "cv2": "OpenCV", "numpy": "NumPy"}
        require_orbbec = _runtime_bool(self.config, "require_orbbec", True) and not self.args.allow_uvc_fallback
        if require_orbbec:
            critical["pyorbbecsdk"] = "Orbbec/Gemini SDK"
        if not self.args.headless:
            critical["PIL"] = "UI"
        for pkg, desc in critical.items():
            try:
                __import__(pkg)
                _ok(f"{pkg} ({desc})")
            except ImportError:
                _err(f"{pkg} ({desc}) — 未安装")
                self.errors.append(f"缺少 Python 包: {pkg}")

        optional = ["torch", "onnxruntime", "tensorrt"]
        if not require_orbbec:
            optional.append("pyorbbecsdk")
        for pkg in optional:
            try:
                __import__(pkg)
                _ok(f"{pkg}")
            except ImportError:
                _info(f"{pkg} — 未安装/可选")

    def _check_config(self) -> None:
        _hdr("[2/3] 配置文件")
        config_path = self.args.config or os.path.join(self.base_dir, "config.json")
        self.config_path = config_path
        if not os.path.exists(config_path):
            _err(f"配置文件不存在: {config_path}")
            self.errors.append("config.json 缺失；请使用仓库默认 config.json 或指定 --config")
            return
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.config = cfg
            udp = cfg.get("udp", {})
            inf = cfg.get("inference", {})
            runtime = cfg.get("runtime", {}) or {}
            target_ip = str(udp.get("target_ip", "")).strip()
            try:
                target_port = int(udp.get("target_port", 0))
            except (TypeError, ValueError):
                target_port = 0
            if not target_ip:
                _err("udp.target_ip 为空")
                self.errors.append("udp.target_ip 为空")
            if target_port <= 0 or target_port > 65535:
                _err(f"udp.target_port 无效: {udp.get('target_port')}")
                self.errors.append("udp.target_port 无效")
            if target_ip and 0 < target_port <= 65535:
                _ok(f"UDP → {target_ip}:{target_port}")
            _ok(f"backend={inf.get('backend', 'auto')} imgsz={inf.get('img_size', 640)}")
            _ok(f"require_orbbec={bool(runtime.get('require_orbbec', True))} strict_detector_load={bool(runtime.get('strict_detector_load', True))}")
            models = cfg.get("models", []) or []
            _ok(f"模型配置: {len(models)} 个") if models else _warn("无模型配置，仅 QR 可用")
        except json.JSONDecodeError as e:
            _err(f"config.json JSON 格式错误: {e}")
            self.errors.append("config.json JSON 格式错误")
        except Exception as e:
            _err(f"config.json 读取异常: {e}")
            self.errors.append(f"config.json 读取异常: {e}")

    def _check_models(self) -> None:
        _hdr("[3/3] 模型文件 / class_id")
        if self.config is None:
            _warn("配置未加载，跳过模型精确检查")
            return
        models_dir = os.path.join(self.base_dir, "models")
        if not os.path.isdir(models_dir):
            _err("models/ 文件夹不存在")
            self.errors.append("models/ 文件夹缺失")
            return

        seen: Set[int] = set()
        enabled_models = 0
        stage = _resolve_stage()
        _ok(f"TECHX_STAGE={stage}")
        models = self.config.get("models", []) or []
        for model in models:
            name = model.get("name", model.get("folder", "<unnamed>"))
            folder = str(model.get("folder", "")).strip()
            enabled = bool(model.get("enabled", True))
            selected = _stage_wants_model(stage, model)
            if not enabled:
                _info(f"{name} disabled")
                continue
            if not selected:
                _info(f"{name} skipped by TECHX_STAGE={stage}")
                continue
            mids = _mapped_ids(model)
            if mids & seen:
                dup = sorted(mids & seen)
                _err(f"{name} class_id 重复: {dup}")
                self.errors.append(f"class_id 重复: {dup}")
            seen |= mids
            enabled_models += 1
            if not folder:
                _err(f"启用模型 {name} 缺少 folder")
                self.errors.append(f"启用模型 {name} 缺少 folder")
                continue
            fp = os.path.join(models_dir, folder)
            if not os.path.isdir(fp):
                _err(f"启用模型 {name} 文件夹不存在: models/{folder}")
                self.errors.append(f"models/{folder} 缺失")
                continue
            preferred = ["best.engine", "best.onnx", "best.pt"]
            found = [f for f in preferred if os.path.exists(os.path.join(fp, f))]
            if not found:
                _err(f"启用模型 {name} 没有 best.engine / best.onnx / best.pt")
                self.errors.append(f"启用模型 {name} 没有模型文件")
                continue
            for f in found:
                size_mb = os.path.getsize(os.path.join(fp, f)) / (1024 * 1024)
                _ok(f"{folder}/ — {f} ({size_mb:.1f} MB)")
            if not mids:
                _warn(f"启用模型 {name} 没有 class_id_map，将使用 local_id + class_id_offset")
                self.warnings.append(f"启用模型 {name} 没有 class_id_map")

        qr = self.config.get("qr", {}) or {}
        qr_selected = stage in {"all", "qr"} and bool(qr.get("enabled", True))
        if qr_selected:
            try:
                seen.add(int(qr.get("class_id", 200)))
                _ok(f"QR enabled class_id={int(qr.get('class_id', 200))}")
            except (TypeError, ValueError):
                _err("qr.class_id 无效")
                self.errors.append("qr.class_id 无效")
        elif stage in {"all", "qr"} and enabled_models == 0:
            _err("没有启用模型，且 QR 也关闭")
            self.errors.append("没有任何检测器可用")

        if stage == "assembly":
            _ok("assembly stage uses color/event detector; no YOLO model required")
        elif stage != "qr" and enabled_models == 0 and not qr_selected:
            _err(f"no detector model selected for TECHX_STAGE={stage}")
            self.errors.append(f"no detector model selected for TECHX_STAGE={stage}")

        for cid, name in _expected_ids_for_stage(stage).items():
            if cid not in seen:
                _warn(f"当前配置不产生 class_id {cid} ({name})")
                self.warnings.append(f"当前配置不产生 class_id {cid} ({name})")

    def _check_camera(self) -> None:
        _hdr("[Full] 相机设备")
        require_orbbec = _runtime_bool(self.config, "require_orbbec", True) and not self.args.allow_uvc_fallback
        if require_orbbec:
            _ok("当前配置要求 Orbbec/Gemini RGB-D；若相机未连接，主程序会等待后退出")
        if sys.platform.startswith("linux"):
            try:
                videos = [f for f in os.listdir("/dev") if f.startswith("video")]
                _ok(f"检测到 {len(videos)} 个视频设备") if videos else _warn("未检测到 /dev/video*，Orbbec SDK 仍可能可用")
            except OSError as e:
                _warn(f"无法读取 /dev: {e}")

    def _check_time_sync_and_gpu(self) -> None:
        if not sys.platform.startswith("linux"):
            return
        _hdr("[Full] 时间同步 / GPU")
        try:
            subprocess.check_output(["chronyc", "tracking"], stderr=subprocess.DEVNULL, timeout=1.0)
            _ok("Chrony tracking 可用")
        except Exception:
            _warn("Chrony 未安装或未同步 — UDP 时间戳仍可用")
            self.warnings.append("建议配置 Chrony/NTP 与 GMK 同步")
        engine_count = 0
        for _, _, files in os.walk(os.path.join(self.base_dir, "models")):
            engine_count += sum(1 for f in files if f.endswith(".engine"))
        _ok(f".engine 文件数量: {engine_count}") if engine_count else _warn("未找到 .engine，Orin NX 上建议导出 TensorRT engine")

    def _launch(self) -> int:
        main_py = os.path.join(self.base_dir, "main.py")
        cmd = [sys.executable, main_py]
        if self.args.config:
            cmd.extend(["--config", self.args.config])
        if self.args.headless:
            cmd.append("--headless")
        if self.args.allow_uvc_fallback:
            cmd.append("--allow-uvc-fallback")
        cmd.extend(["--log-level", self.args.log_level])
        if self.args.camera_wait_timeout is not None:
            cmd.extend(["--camera-wait-timeout", str(self.args.camera_wait_timeout)])
        if self.args.no_camera_frame_timeout is not None:
            cmd.extend(["--no-camera-frame-timeout", str(self.args.no_camera_frame_timeout)])
        if self.args.no_infer_result_timeout is not None:
            cmd.extend(["--no-infer-result-timeout", str(self.args.no_infer_result_timeout)])
        env = os.environ.copy()
        if self.args.fast_start:
            env.setdefault("TECHX_SKIP_TIME_SYNC", "1")
            env.setdefault("TECHX_SKIP_JETSON_OPT", "1")
        try:
            return subprocess.call(cmd, env=env)
        except KeyboardInterrupt:
            print(f"\n{YELLOW}用户中断{NC}")
            return 0
        except Exception as e:
            _err(f"启动失败: {e}")
            return 1


def main() -> None:
    parser = argparse.ArgumentParser(description="TECHX_vision 一键启动器 — 自检 + 启动")
    parser.add_argument("--config", type=str, help="config.json 配置文件路径")
    parser.add_argument("--check-only", action="store_true", help="仅运行自检，不启动程序")
    parser.add_argument("--headless", action="store_true", help="无界面模式（仅 UDP 发送）")
    parser.add_argument("--fast-start", action="store_true", help="比赛快速启动：跳过重型自检")
    parser.add_argument("--allow-uvc-fallback", action="store_true", help="允许 Orbbec/Gemini 失败后使用 UVC，仅用于调试")
    parser.add_argument("--camera-wait-timeout", type=float, default=600.0, help="等待相机秒数；<=0 表示一直等待；默认600秒")
    parser.add_argument("--no-camera-frame-timeout", type=float, default=600.0, help="运行中无新相机帧多久自动退出；<=0禁用；默认600秒")
    parser.add_argument("--no-infer-result-timeout", type=float, default=600.0, help="运行中无新推理结果多久自动退出；<=0禁用；默认600秒")
    parser.add_argument("--log-level", type=str, default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()
    sys.exit(Launcher(args).run())


if __name__ == "__main__":
    main()
