"""Tkinter dashboard for the TECHX Jetson vision runtime.

This UI is an operator view for Jetson-side verification.  It deliberately
shows only camera_link coordinates; robot_base / arm1_base / arm2_base are
computed by the GMK bridge after UDP reception.
"""

from __future__ import annotations

import csv
import os
import subprocess
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Optional

import cv2
import numpy as np
from PIL import Image, ImageTk

from config.settings import Settings
from interface.types import Target3D
from tuning.tuning import TuningParams
from utils.logger import get_logger

log = get_logger(__name__)


class TechxVisionApp:
    """Competition-oriented Jetson vision dashboard.

    Left side: live image and detection boxes.
    Right side: beginner-readable status, target details, temporary tuning,
    and camera-to-robot calibration sampling.
    """

    def __init__(self, engine, settings: Settings, tuning: TuningParams):
        self._engine = engine
        self._settings = settings
        self._tuning = tuning
        self._window = tk.Tk()
        self._running = True
        self._frame_count = 0
        self._serial = None
        self._latest_target: Optional[Target3D] = None
        self._last_stats_time = 0.0
        self._last_table_refresh = 0.0
        self._ui_tick_ms = max(10, int(float(os.getenv("TECHX_UI_TICK_MS", "20"))))
        self._table_refresh_sec = max(0.05, float(os.getenv("TECHX_UI_TABLE_REFRESH_SEC", "0.20")))
        self._display_every_n = max(1, int(float(os.getenv("TECHX_DISPLAY_EVERY_N", str(self._tuning.display_every_n)))))

        self._build_ui()
        self._wire_callbacks()
        self._window.protocol("WM_DELETE_WINDOW", self._on_close)
        self._window.bind("<Key>", self._on_key)

    def run(self) -> None:
        self._tick()
        self._window.mainloop()

    # ------------------------------------------------------------------ UI build

    def _build_ui(self) -> None:
        self._window.title("TECHX Jetson 视觉上车调试面板")
        self._window.geometry("1360x820")
        self._window.minsize(1180, 720)

        style = ttk.Style(self._window)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Status.TLabel", font=("Arial", 10, "bold"))
        style.configure("Title.TLabel", font=("Arial", 12, "bold"))
        style.configure("Small.TLabel", font=("Arial", 9))

        root = ttk.Frame(self._window, padding=8)
        root.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(root)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(root, width=430)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right.pack_propagate(False)

        header = ttk.Frame(left)
        header.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(header, text="TECHX Jetson 视觉调试：Gemini 335L → 识别 → camera_link → UDP", style="Title.TLabel").pack(side=tk.LEFT)
        self._pipeline_state_var = tk.StringVar(value="运行中")
        ttk.Label(header, textvariable=self._pipeline_state_var, style="Status.TLabel").pack(side=tk.RIGHT)

        video_box = ttk.LabelFrame(left, text="实时画面：这里只验证 Jetson 识别和 camera_link 深度，不显示 GMK 坐标")
        video_box.pack(fill=tk.BOTH, expand=True)
        self._video_label = tk.Label(video_box, bg="black")
        self._video_label.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        footer = ttk.Frame(left)
        footer.pack(fill=tk.X, pady=(6, 0))
        self._hint_var = tk.StringVar(
            value="快捷键：Q退出 | Space暂停/继续 | D切换深度图 | C复制当前目标的 camera_link 坐标"
        )
        ttk.Label(footer, textvariable=self._hint_var).pack(side=tk.LEFT)

        self._notebook = ttk.Notebook(right)
        self._notebook.pack(fill=tk.BOTH, expand=True)
        self._build_status_tab()
        self._build_targets_tab()
        self._build_tuning_tab()
        self._build_calibration_tab()

    def _build_status_tab(self) -> None:
        tab = ttk.Frame(self._notebook, padding=8)
        self._notebook.add(tab, text="总览")

        self._health_var = tk.StringVar(value="系统状态：等待第一帧统计…")
        ttk.Label(tab, textvariable=self._health_var, style="Status.TLabel", wraplength=390).pack(fill=tk.X, pady=(0, 8))

        self._camera_var = tk.StringVar(value=self._camera_text())
        self._backend_var_text = tk.StringVar(value=f"推理后端：{self._safe_backend_name()}")
        self._infer_var = tk.StringVar(value="推理速度：-- ms")
        self._fps_var = tk.StringVar(value="画面刷新：-- FPS")
        self._target_var = tk.StringVar(value="本帧目标：0")
        self._udp_var = tk.StringVar(value=f"UDP发送：→ {self._settings.udp.target_ip}:{self._settings.udp.target_port}")
        self._age_var = tk.StringVar(value="数据新鲜度：--")

        for var in (
            self._camera_var,
            self._backend_var_text,
            self._infer_var,
            self._fps_var,
            self._target_var,
            self._udp_var,
            self._age_var,
        ):
            ttk.Label(tab, textvariable=var, anchor=tk.W, wraplength=390).pack(fill=tk.X, pady=3)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Label(tab, text="比赛上车检查", style="Title.TLabel").pack(anchor=tk.W)
        self._safety_text = tk.Text(tab, height=9, wrap=tk.WORD, relief=tk.FLAT, bg=self._window.cget("bg"))
        self._safety_text.insert("1.0", self._runtime_safety_text())
        self._safety_text.configure(state=tk.DISABLED)
        self._safety_text.pack(fill=tk.BOTH, expand=False, pady=4)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Label(tab, text="启用模型和类别", style="Title.TLabel").pack(anchor=tk.W)
        self._models_label = tk.Text(tab, height=10, wrap=tk.WORD, relief=tk.FLAT, bg=self._window.cget("bg"))
        self._models_label.insert("1.0", self._model_summary_text())
        self._models_label.configure(state=tk.DISABLED)
        self._models_label.pack(fill=tk.BOTH, expand=False, pady=4)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Label(
            tab,
            text=(
                "重要：Jetson UI 只显示 camera_link 相机坐标。底盘 robot_x/y/z、"
                "机械臂1 arm1_x/y/z、机械臂2 arm2_x/y/z 由 GMK bridge 输出，"
                "请在 GMK 的 /techx/vision/frame 或 /techx/vision/selected 中查看。"
            ),
            wraplength=390,
            foreground="#8a4b00",
        ).pack(fill=tk.X, pady=4)

    def _build_targets_tab(self) -> None:
        tab = ttk.Frame(self._notebook, padding=8)
        self._notebook.add(tab, text="目标")

        ttk.Label(
            tab,
            text="这里显示 Jetson 当前识别到的目标。坐标是 camera_link；不是底盘/机械臂坐标。",
            wraplength=390,
            foreground="#555555",
        ).pack(fill=tk.X, pady=(0, 6))

        cols = ("id", "cls", "role", "conf", "uv", "z", "state")
        self._target_table = ttk.Treeview(tab, columns=cols, show="headings", height=9)
        headings = {
            "id": "Track",
            "cls": "目标ID",
            "role": "物体含义",
            "conf": "置信度",
            "uv": "像素(u,v)",
            "z": "深度Z(m)",
            "state": "状态",
        }
        widths = {"id": 48, "cls": 54, "role": 118, "conf": 58, "uv": 84, "z": 72, "state": 64}
        for c in cols:
            self._target_table.heading(c, text=headings[c])
            self._target_table.column(c, width=widths[c], anchor=tk.CENTER)
        self._target_table.pack(fill=tk.BOTH, expand=True)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Label(tab, text="当前选中/最近目标解释", style="Title.TLabel").pack(anchor=tk.W)
        self._detail_text = tk.Text(tab, height=16, wrap=tk.WORD)
        self._detail_text.pack(fill=tk.BOTH, expand=False)
        self._set_detail_text(self._empty_target_text())

    def _build_tuning_tab(self) -> None:
        tab = ttk.Frame(self._notebook, padding=8)
        self._notebook.add(tab, text="调试")

        ttk.Label(
            tab,
            text="本页只用于临时调试，不会永久修改 config.json。比赛参数应写回 config.json 后重启验证。",
            wraplength=390,
            foreground="#8a4b00",
        ).pack(fill=tk.X, pady=(0, 8))

        ttk.Label(tab, text="识别置信度阈值", style="Title.TLabel").pack(anchor=tk.W)
        self._conf_var = tk.DoubleVar(value=self._tuning.conf_threshold)
        self._conf_value = tk.StringVar(value=f"当前：{self._tuning.conf_threshold:.2f}，比赛建议先用 0.40~0.50")
        ttk.Scale(tab, from_=0.01, to=1.0, variable=self._conf_var, command=self._on_change_conf).pack(fill=tk.X, pady=3)
        ttk.Label(tab, textvariable=self._conf_value, wraplength=390).pack(anchor=tk.W)

        ttk.Label(tab, text="NMS / IoU 阈值", style="Title.TLabel").pack(anchor=tk.W, pady=(12, 0))
        self._iou_var = tk.DoubleVar(value=self._tuning.iou_threshold)
        self._iou_value = tk.StringVar(value=f"当前：{self._tuning.iou_threshold:.2f}")
        ttk.Scale(tab, from_=0.01, to=1.0, variable=self._iou_var, command=self._on_change_iou).pack(fill=tk.X, pady=3)
        ttk.Label(tab, textvariable=self._iou_value).pack(anchor=tk.W)

        ttk.Label(tab, text="推理后端", style="Title.TLabel").pack(anchor=tk.W, pady=(12, 0))
        backend_items = ["auto", "tensorrt", "cuda", "pytorch", "onnx_gpu", "onnx"]
        initial_backend = self._settings.inference.backend if self._settings.inference.backend in backend_items else "auto"
        self._backend_choice = tk.StringVar(value=initial_backend)
        ttk.Combobox(tab, values=backend_items, textvariable=self._backend_choice, state="readonly").pack(fill=tk.X, pady=3)
        ttk.Button(tab, text="临时切换 YOLO 后端", command=self._on_change_backend).pack(fill=tk.X, pady=3)
        ttk.Label(
            tab,
            text=(
                "排查误识别时建议先切 cuda，让 Jetson 跑 .pt 对齐 Windows；"
                "确认准确后再切 auto / tensorrt 测速度。"
            ),
            wraplength=390,
        ).pack(fill=tk.X, pady=6)

        if not self._engine.camera.has_depth:
            ttk.Label(tab, text="UVC 调试模拟深度(m)", style="Title.TLabel").pack(anchor=tk.W, pady=(12, 0))
            self._depth_var = tk.StringVar(value="1.2")
            entry = ttk.Entry(tab, textvariable=self._depth_var)
            entry.pack(fill=tk.X, pady=3)
            entry.bind("<Return>", self._on_change_manual_depth)
            ttk.Label(tab, text="比赛使用 Gemini 335L 时不应该出现这个选项。", wraplength=390, foreground="#8a4b00").pack(fill=tk.X)

    def _build_calibration_tab(self) -> None:
        tab = ttk.Frame(self._notebook, padding=8)
        self._notebook.add(tab, text="标定")

        ttk.Label(
            tab,
            text=(
                "标定用途：采集 Jetson camera_link 点与人工测量的 robot_base 点，"
                "求 T_robot_camera。求出的外参要填到 GMK vision_bridge.yaml。"
            ),
            wraplength=390,
            foreground="#555555",
        ).pack(fill=tk.X, pady=(0, 8))

        steps = (
            "流程：1 固定 Gemini335L 和机器人坐标系 -> 2 放置棋盘格或目标物 -> "
            "3 输入同一点在 robot_base 下的真实坐标 -> 4 追加点对 -> "
            "5 至少采 6-9 个分散点 -> 6 导出 gmk_robot_camera.yaml。"
        )
        ttk.Label(tab, text=steps, wraplength=390, foreground="#064f7a").pack(fill=tk.X, pady=(0, 8))
        self._calib_status_var = tk.StringVar(value="标定状态：等待采样。建议点位覆盖左/中/右、近/中/远、不同高度。")
        ttk.Label(tab, textvariable=self._calib_status_var, wraplength=390, foreground="#555555").pack(fill=tk.X, pady=(0, 8))

        ttk.Label(tab, text="当前 camera_link 点", style="Title.TLabel").pack(anchor=tk.W)
        self._camera_point_var = tk.StringVar(value="无有效目标：请先让目标进入画面并保证深度有效")
        ttk.Label(tab, textvariable=self._camera_point_var, wraplength=390).pack(fill=tk.X, pady=4)
        ttk.Button(tab, text="复制当前 camera_link 坐标", command=self._copy_camera_point).pack(fill=tk.X, pady=3)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Label(tab, text="对应 robot_base 已知点，单位 m", style="Title.TLabel").pack(anchor=tk.W)
        ttk.Label(tab, text="建议约定：robot_x 前方为正，robot_y 左方为正，robot_z 上方为正。", wraplength=390).pack(fill=tk.X)
        grid = ttk.Frame(tab)
        grid.pack(fill=tk.X, pady=4)
        self._robot_x_var = tk.StringVar(value="0.0")
        self._robot_y_var = tk.StringVar(value="0.0")
        self._robot_z_var = tk.StringVar(value="0.0")
        for i, (label, var) in enumerate((("X前后", self._robot_x_var), ("Y左右", self._robot_y_var), ("Z高度", self._robot_z_var))):
            ttk.Label(grid, text=label).grid(row=0, column=i * 2, sticky=tk.W, padx=(0, 2))
            ttk.Entry(grid, textvariable=var, width=8).grid(row=0, column=i * 2 + 1, sticky=tk.W, padx=(0, 8))

        self._csv_path_var = tk.StringVar(value="runs/calib_robot_camera/robot_camera_points.csv")
        ttk.Label(tab, text="CSV 文件名").pack(anchor=tk.W, pady=(8, 0))
        ttk.Entry(tab, textvariable=self._csv_path_var).pack(fill=tk.X, pady=3)
        self._calib_count_var = tk.StringVar(value="当前 CSV：0 组点")
        ttk.Label(tab, textvariable=self._calib_count_var, wraplength=390).pack(fill=tk.X)
        ttk.Button(tab, text="方式A：追加当前识别目标中心点", command=self._append_calib_point).pack(fill=tk.X, pady=3)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Label(tab, text="棋盘格自动采样", style="Title.TLabel").pack(anchor=tk.W)
        ttk.Label(tab, text="输入内角点数量和格子边长；推荐 A4 9x6，square=0.025m。采样时棋盘必须完整入画。", wraplength=390).pack(fill=tk.X)
        board = ttk.Frame(tab)
        board.pack(fill=tk.X, pady=4)
        self._board_cols_var = tk.StringVar(value="9")
        self._board_rows_var = tk.StringVar(value="6")
        self._square_size_var = tk.StringVar(value="0.025")
        for i, (label, var) in enumerate((("列内角", self._board_cols_var), ("行内角", self._board_rows_var), ("格长m", self._square_size_var))):
            ttk.Label(board, text=label).grid(row=0, column=i * 2, sticky=tk.W, padx=(0, 2))
            ttk.Entry(board, textvariable=var, width=7).grid(row=0, column=i * 2 + 1, sticky=tk.W, padx=(0, 8))
        ttk.Button(tab, text="方式B：识别棋盘格并追加中心点", command=self._append_chessboard_calib_point).pack(fill=tk.X, pady=3)

        ttk.Separator(tab).pack(fill=tk.X, pady=8)
        ttk.Button(tab, text="导出 GMK YAML / 报告", command=self._export_calib_yaml).pack(fill=tk.X, pady=3)
        ttk.Button(tab, text="复制外参估计命令", command=self._copy_estimate_command).pack(fill=tk.X, pady=3)

        ttk.Label(
            tab,
            text=(
                "注意：camera_link 点可以自动获取，但 robot_base X/Y/Z 必须是你实测的同一个参考点。"
                "点位越分散，外参越稳；如果报告 RMSE 超过 1-2cm，应重新采样。"
            ),
            wraplength=390,
            foreground="#8a4b00",
        ).pack(fill=tk.X, pady=8)
        self._refresh_calib_count()

    # ---------------------------------------------------------------- callbacks

    def _wire_callbacks(self) -> None:
        self._engine.on_stats = self._on_stats_update
        self._engine.on_target = self._on_target_update

    def _on_stats_update(self, stats: dict) -> None:
        avg_infer = float(stats.get("avg_infer_ms", 0.0))
        avg_display = float(stats.get("avg_display_ms", 0.0))
        track_count = int(stats.get("track_count", 0))
        target_count = int(stats.get("target_count", 0))
        sender_ready = bool(stats.get("sender_ready", False))
        camera_age = float(stats.get("camera_frame_age_sec", 0.0))
        infer_age = float(stats.get("infer_result_age_sec", 0.0))
        fatal_reason = str(stats.get("fatal_reason", "") or "")
        display_fps = 1000.0 / avg_display if avg_display > 0 else 0.0
        detect_fps = 1000.0 / avg_infer if avg_infer > 0 else 0.0

        self._infer_var.set(f"推理速度：{avg_infer:.1f} ms / 帧，约 {detect_fps:.1f} Hz")
        self._fps_var.set(f"界面刷新：{display_fps:.1f} FPS")
        self._target_var.set(f"本帧目标：track={track_count}，可下发目标={target_count}")
        self._udp_var.set(
            f"UDP发送：{'已建立本机发送' if sender_ready else '未就绪'} → "
            f"{self._settings.udp.target_ip}:{self._settings.udp.target_port}；GMK 是否收到要看 /techx/vision/frame"
        )
        self._backend_var_text.set(f"推理后端：{self._safe_backend_name()}")
        self._age_var.set(f"数据新鲜度：相机 {camera_age:.2f}s，推理结果 {infer_age:.2f}s，更新时间 {time.strftime('%H:%M:%S')}")
        self._health_var.set(self._health_text(target_count, sender_ready, camera_age, infer_age, fatal_reason))
        self._last_stats_time = time.monotonic()

    def _on_target_update(self, target: Target3D) -> None:
        self._latest_target = target
        self._update_calibration_point(target)
        self._set_detail_for_target(target)

    def _on_change_conf(self, val) -> None:
        self._tuning.conf_threshold = float(val)
        self._conf_value.set(f"当前：{self._tuning.conf_threshold:.2f}，比赛建议先用 0.40~0.50")

    def _on_change_iou(self, val) -> None:
        self._tuning.iou_threshold = float(val)
        self._iou_value.set(f"当前：{self._tuning.iou_threshold:.2f}")

    def _on_change_backend(self) -> None:
        backend = self._backend_choice.get()
        fn = getattr(self._engine.detector, "set_backend", None)
        if not callable(fn):
            messagebox.showwarning("不能切换后端", "当前检测器不支持运行时后端切换。")
            return
        try:
            label = fn(backend)
            self._backend_var_text.set(f"推理后端：{label}")
            messagebox.showinfo("后端已切换", f"当前推理后端：{label}\n这是临时调试，不会改写 config.json。")
            log.info("检测后端已切换: %s", label)
        except Exception as e:
            messagebox.showerror("后端切换失败", str(e))

    def _on_change_manual_depth(self, _=None) -> None:
        try:
            self._engine.set_manual_depth(float(self._depth_var.get()))
        except Exception:
            pass

    def _on_key(self, event) -> None:
        key = event.keysym.lower()
        if key == "q":
            self._on_close()
        elif key == "space":
            self._engine.set_paused(not self._engine.is_paused)
            self._pipeline_state_var.set("已暂停" if self._engine.is_paused else "运行中")
        elif key == "d":
            show = self._engine.toggle_depth_view()
            self._hint_var.set("已显示深度图" if show else "已切回普通画面")
            log.info("深度图: %s", "ON" if show else "OFF")
        elif key == "c":
            self._copy_camera_point()

    # --------------------------------------------------------------- UI helpers

    def _tick(self) -> None:
        if not self._running:
            return
        ann = self._engine.tick()
        self._frame_count += 1

        if ann is not None and self._frame_count % self._display_every_n == 0:
            rgb = cv2.cvtColor(ann, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self._video_label.imgtk = imgtk
            self._video_label.configure(image=imgtk)

        now = time.monotonic()
        if now - self._last_table_refresh >= self._table_refresh_sec:
            self._last_table_refresh = now
            self._refresh_target_table()
        if not self._engine.camera.has_depth and hasattr(self, "_depth_var"):
            self._on_change_manual_depth()

        self._window.after(self._ui_tick_ms, self._tick)

    def _refresh_target_table(self) -> None:
        targets = list(getattr(self._engine, "targets", []) or [])
        for item in self._target_table.get_children():
            self._target_table.delete(item)
        for target in targets[:16]:
            u, v = target.pixel_uv
            self._target_table.insert("", tk.END, values=(
                target.track_id,
                target.class_id,
                self._class_description(target),
                f"{target.confidence:.2f}",
                f"{u:.0f},{v:.0f}",
                f"{target.depth_m:.2f}" if target.depth_m > 0 else "无",
                "可用" if target.depth_m > 0 else "无深度",
            ))
        if targets:
            self._latest_target = targets[0]
            self._update_calibration_point(targets[0])
            self._set_detail_for_target(targets[0])
        elif self._latest_target is None:
            self._set_detail_text(self._empty_target_text())

    def _set_detail_for_target(self, target: Target3D) -> None:
        u, v = target.pixel_uv
        xc, yc, zc = target.camera_xyz
        text = (
            f"当前目标（Jetson 侧）\n"
            f"  目标编号 class_id：{target.class_id}  {self._class_description(target)}\n"
            f"  目标类型说明：{self._target_role_text(target.class_id)}\n"
            f"  track_id：{target.track_id}\n"
            f"  原始类别名：{target.class_name}\n"
            f"  置信度 confidence：{target.confidence:.3f}\n"
            f"  颜色：{target.color or '未区分'}\n"
            f"  像素中心 pixel_uv：({u:.1f}, {v:.1f})\n"
            f"  camera_link 坐标：x={xc:.4f} m, y={yc:.4f} m, z={zc:.4f} m\n"
            f"  深度状态：{'有效' if target.depth_m > 0 else '无有效深度'}，depth={target.depth_m:.4f} m\n\n"
            f"下游使用提醒：\n"
            f"  Jetson 只负责识别和 camera_link。\n"
            f"  底盘 robot_x/y/z、机械臂1 arm1_x/y/z、机械臂2 arm2_x/y/z，"
            f"要到 GMK 的 /techx/vision/frame 或 /techx/vision/selected 中看。\n"
            f"  控制前必须检查 GMK 输出的 valid_robot_xyz / valid_arm1_xyz / valid_arm2_xyz。"
        )
        self._set_detail_text(text)

    def _empty_target_text(self) -> str:
        return (
            "等待目标…\n\n"
            "如果画面里有目标但这里没有数据，请按顺序检查：\n"
            "1. 目标是否在视野内且没有严重过曝/欠曝；\n"
            "2. 当前后端是否加载了正确模型，尤其是否误跑 last.pt；\n"
            "3. 置信度阈值是否过高；\n"
            "4. KFS/武器头模型是否启用；\n"
            "5. 深度是否有效，Gemini 335L 是否稳定出深度。"
        )

    def _set_detail_text(self, text: str) -> None:
        self._detail_text.configure(state=tk.NORMAL)
        self._detail_text.delete("1.0", tk.END)
        self._detail_text.insert("1.0", text)
        self._detail_text.configure(state=tk.DISABLED)

    def _update_calibration_point(self, target: Optional[Target3D]) -> None:
        if target is None or target.depth_m <= 0:
            self._camera_point_var.set("无有效 camera_link 点：目标必须有有效深度 z>0")
            return
        x, y, z = target.camera_xyz
        self._camera_point_var.set(
            f"class_id={target.class_id}  {self._class_description(target)}\n"
            f"camera_link = ({x:.6f}, {y:.6f}, {z:.6f}) m"
        )

    def _copy_camera_point(self) -> None:
        target = self._latest_target
        if target is None or target.depth_m <= 0:
            messagebox.showwarning("不能复制", "当前没有有效深度目标，不能复制 camera_link 点。")
            return
        x, y, z = target.camera_xyz
        text = f"{x:.9g},{y:.9g},{z:.9g}"
        self._window.clipboard_clear()
        self._window.clipboard_append(text)
        self._hint_var.set(f"已复制 camera_link 坐标：{text}")

    def _append_calib_point(self) -> None:
        target = self._latest_target
        if target is None or target.depth_m <= 0:
            messagebox.showwarning("不能追加", "当前没有有效 camera_link 点。")
            return
        try:
            to_xyz = (float(self._robot_x_var.get()), float(self._robot_y_var.get()), float(self._robot_z_var.get()))
        except ValueError:
            messagebox.showerror("输入错误", "robot_base X/Y/Z 必须是数字，单位米。")
            return
        self._append_calib_pair(target.camera_xyz, to_xyz, note=f"target_{target.track_id}")

    def _append_chessboard_calib_point(self) -> None:
        frame = getattr(self._engine, "_last_frame", None)
        depth = getattr(self._engine, "_last_depth", None)
        if frame is None:
            messagebox.showwarning("不能采样", "还没有相机画面。请确认 Gemini335L 已打开。")
            return
        if depth is None:
            messagebox.showwarning("不能采样", "当前没有深度图。棋盘格标定需要 Gemini335L 深度。")
            return
        try:
            cols = int(self._board_cols_var.get())
            rows = int(self._board_rows_var.get())
            square = float(self._square_size_var.get())
            to_xyz = (float(self._robot_x_var.get()), float(self._robot_y_var.get()), float(self._robot_z_var.get()))
        except ValueError:
            messagebox.showerror("输入错误", "棋盘格参数和 robot_base X/Y/Z 必须是数字。")
            return
        if cols <= 2 or rows <= 2 or square <= 0:
            messagebox.showerror("输入错误", "棋盘格内角点和格长不合理。")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern = (cols, rows)
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found or corners is None:
            self._calib_status_var.set("棋盘格未识别：请保证完整入画、光照均匀、不要太斜或太远。")
            messagebox.showwarning("未识别棋盘格", "没有找到完整棋盘格。请调整距离、角度和光照。")
            return

        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
        obj = np.zeros((rows * cols, 3), np.float32)
        obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square)
        fx, fy, cx, cy = self._engine.solver.fx, self._engine.solver.fy, self._engine.solver.cx, self._engine.solver.cy
        k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, corners, k, None, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            messagebox.showwarning("PnP失败", "棋盘格角点已找到，但 solvePnP 失败。")
            return

        ref = np.array([(cols - 1) * square * 0.5, (rows - 1) * square * 0.5, 0.0], dtype=np.float32)
        rmat, _ = cv2.Rodrigues(rvec)
        p_cam = (rmat @ ref.reshape(3, 1) + tvec.reshape(3, 1)).reshape(3)
        ref_img, _ = cv2.projectPoints(ref.reshape(1, 3), rvec, tvec, k, None)
        u, v = ref_img.reshape(2)
        depth_ok = self._chessboard_depth_ok(depth, float(u), float(v), float(p_cam[2]))
        if not depth_ok:
            self._calib_status_var.set("棋盘格PnP成功，但中心深度和PnP Z差异较大；建议调整角度/距离后重采。")
            if not messagebox.askyesno("深度一致性较差", "棋盘格PnP成功，但深度一致性较差。仍然追加这个点吗？"):
                return

        self._append_calib_pair(tuple(float(x) for x in p_cam), to_xyz, note="chessboard_center")
        overlay_dir = Path(os.getcwd()) / "runs" / "calib_robot_camera" / "overlays"
        overlay_dir.mkdir(parents=True, exist_ok=True)
        overlay = frame.copy()
        cv2.drawChessboardCorners(overlay, pattern, corners, True)
        cv2.circle(overlay, (int(round(u)), int(round(v))), 5, (255, 0, 0), -1)
        cv2.imwrite(str(overlay_dir / f"chessboard_{int(time.time())}.jpg"), overlay)

    def _append_calib_pair(self, from_xyz, to_xyz, note: str = "") -> None:
        csv_path = self._calib_csv_path()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        is_new = not csv_path.exists()
        with csv_path.open("a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            if is_new:
                writer.writerow(["from_x", "from_y", "from_z", "to_x", "to_y", "to_z", "note", "timestamp"])
            writer.writerow([*[f"{float(v):.9f}" for v in from_xyz], *[f"{float(v):.9f}" for v in to_xyz], note, f"{time.time():.6f}"])
        self._refresh_calib_count()
        self._calib_status_var.set(f"已追加点对：camera_link={tuple(round(float(v), 4) for v in from_xyz)} -> robot_base={tuple(round(float(v), 4) for v in to_xyz)}")
        self._hint_var.set(f"已追加标定点：{csv_path}")

    def _calib_csv_path(self) -> Path:
        csv_path = Path(self._csv_path_var.get()).expanduser()
        if not csv_path.is_absolute():
            csv_path = Path(os.getcwd()) / csv_path
        return csv_path

    @staticmethod
    def _chessboard_depth_ok(depth_mm: np.ndarray, u: float, v: float, pnp_z_m: float) -> bool:
        h, w = depth_mm.shape[:2]
        x0, x1 = max(0, int(round(u)) - 5), min(w, int(round(u)) + 6)
        y0, y1 = max(0, int(round(v)) - 5), min(h, int(round(v)) + 6)
        roi = depth_mm[y0:y1, x0:x1]
        valid = roi[(roi > 150) & (roi < 2500)]
        if valid.size < max(8, roi.size * 0.4):
            return False
        med_m = float(np.median(valid)) / 1000.0
        return abs(med_m - float(pnp_z_m)) <= 0.05

    def _refresh_calib_count(self) -> None:
        try:
            path = self._calib_csv_path()
            count = 0
            if path.exists():
                with path.open("r", encoding="utf-8", newline="") as f:
                    count = max(0, sum(1 for _ in csv.DictReader(f)))
            self._calib_count_var.set(f"当前 CSV：{count} 组点 | {path}")
        except Exception:
            pass

    def _export_calib_yaml(self) -> None:
        csv_path = self._calib_csv_path()
        if not csv_path.exists():
            messagebox.showwarning("不能导出", "还没有标定 CSV。请先采集点对。")
            return
        out_dir = csv_path.parent
        yaml_path = out_dir / "gmk_robot_camera.yaml"
        report_path = out_dir / "gmk_robot_camera_report.json"
        residual_path = out_dir / "gmk_robot_camera_residuals.csv"
        cmd = [
            sys.executable,
            "tools/export_handeye_yaml.py",
            "--csv", str(csv_path),
            "--name", "T_robot_camera",
            "--output-yaml", str(yaml_path),
            "--report-json", str(report_path),
            "--residual-csv", str(residual_path),
        ]
        try:
            proc = subprocess.run(cmd, cwd=os.getcwd(), capture_output=True, text=True, timeout=20)
        except Exception as exc:
            messagebox.showerror("导出失败", str(exc))
            return
        if proc.returncode != 0:
            messagebox.showerror("导出失败", (proc.stderr or proc.stdout or "unknown error")[:1500])
            return
        self._calib_status_var.set(f"导出完成：{yaml_path}。请把 YAML 内容复制到 GMK vision_bridge.yaml。")
        messagebox.showinfo("导出完成", f"已生成：\n{yaml_path}\n{report_path}\n{residual_path}")

    def _copy_estimate_command(self) -> None:
        csv_path = self._calib_csv_path()
        cmd = (
            f"{sys.executable} tools/export_handeye_yaml.py --csv {csv_path} --name T_robot_camera "
            f"--output-yaml {csv_path.parent / 'gmk_robot_camera.yaml'} "
            f"--report-json {csv_path.parent / 'gmk_robot_camera_report.json'} "
            f"--residual-csv {csv_path.parent / 'gmk_robot_camera_residuals.csv'}"
        )
        self._window.clipboard_clear()
        self._window.clipboard_append(cmd)
        self._hint_var.set("已复制外参估计命令")

    def _safe_backend_name(self) -> str:
        return str(getattr(self._engine.detector, "backend_name", self._engine.detector.__class__.__name__))

    def _camera_text(self) -> str:
        cam_type = "Gemini/Orbbec RGB-D" if self._engine.camera.has_depth else "UVC/无深度"
        return f"相机：{cam_type}，分辨率 {self._engine.camera.width}x{self._engine.camera.height}"

    def _runtime_safety_text(self) -> str:
        lines = []
        lines.append(f"Gemini/Orbbec 必需：{'是' if self._settings.runtime.require_orbbec else '否，允许UVC调试'}")
        lines.append(f"检测器严格加载：{'是' if self._settings.runtime.strict_detector_load else '否，允许部分模型缺失'}")
        lines.append(f"全局置信度阈值：{self._settings.inference.conf_threshold:.2f}")
        lines.append(f"配置后端：{self._settings.inference.backend}")
        if not self._settings.runtime.require_orbbec:
            lines.append("警告：比赛不建议关闭 require_orbbec，否则可能只有2D框没有可靠深度。")
        if not self._settings.runtime.strict_detector_load:
            lines.append("警告：比赛不建议关闭 strict_detector_load，否则可能只剩 QR/KFS 等部分检测器在跑。")
        return "\n".join(lines)

    def _model_summary_text(self) -> str:
        lines = []
        for model in self._settings.models:
            flag = "启用" if model.enabled else "关闭"
            files = []
            if model.engine_path:
                files.append(f"engine={Path(model.engine_path).name}")
            if model.onnx_path:
                files.append(f"onnx={Path(model.onnx_path).name}")
            if model.pt_path:
                files.append(f"pt={Path(model.pt_path).name}")
            lines.append(
                f"[{flag}] {model.name}\n"
                f"  folder={model.folder}, class_offset={model.class_id_offset}\n"
                f"  {'，'.join(files) if files else '未找到模型文件'}"
            )
        if self._settings.qr.enabled:
            lines.append(f"[启用] QR 二维码检测：class_id={self._settings.qr.class_id}")
        else:
            lines.append("[关闭] QR 二维码检测")
        return "\n".join(lines) if lines else "没有配置任何模型"

    def _health_text(self, target_count: int, sender_ready: bool, camera_age: float, infer_age: float, fatal_reason: str) -> str:
        if fatal_reason:
            return f"系统状态：停止 / {fatal_reason}"
        if camera_age > 1.0:
            return f"系统状态：相机数据过旧 {camera_age:.1f}s，请检查 Gemini 335L/USB/供电"
        if infer_age > 1.0:
            return f"系统状态：推理结果过旧 {infer_age:.1f}s，请检查模型后端或GPU负载"
        if not sender_ready:
            return "系统状态：识别链路运行中，但 UDP 发送未就绪"
        if target_count <= 0:
            return "系统状态：在线，无目标；这会向 GMK 发送 count=0 空帧"
        return f"系统状态：在线，当前可下发目标 {target_count} 个"

    def _class_description(self, target: Target3D) -> str:
        mapping = {
            0: "红方R1 KFS",
            1: "红方R2假KFS",
            2: "红方R2真KFS",
            3: "蓝方R1 KFS",
            4: "蓝方R2假KFS",
            5: "蓝方R2真KFS",
            100: "拳头武器头",
            101: "手掌武器头",
            102: "长矛武器头",
            200: "二维码",
        }
        return mapping.get(target.class_id, target.class_name or "未知目标")

    def _target_role_text(self, class_id: int) -> str:
        if 0 <= class_id <= 5:
            return "KFS：GMK 会同时输出 robot_base 给底盘、arm2_base 给机械臂2。"
        if 100 <= class_id <= 102:
            return "武器头：GMK 会同时输出 robot_base 给底盘、arm1_base 给机械臂1。"
        if class_id == 200:
            return "二维码：GMK 主要输出 robot_base 给底盘定位/对齐。"
        return "未知目标：请检查 class_id_map 和 GMK class_rules。"

    def _on_close(self) -> None:
        log.info("UI 请求关闭")
        self._running = False
        self._engine.stop()
        if self._serial is not None and getattr(self._serial, "is_open", False):
            self._serial.close()
        self._window.destroy()
