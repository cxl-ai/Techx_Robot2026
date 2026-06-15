"""TECHx_vision 自定义异常层级体系。

所有模块均从该层级中抛出异常，使得顶层启动器只需捕获
TechxVisionError 即可优雅地处理错误（记录日志并安全关闭），
而无需了解内部模块的具体实现。
"""

from __future__ import annotations


class TechxVisionError(Exception):
    """TECHx_vision 所有异常的统一基类。"""


# ── 相机相关异常 ──────────────────────────────────────────────────

class CameraError(TechxVisionError):
    """相机初始化或运行时故障。"""


class CameraNotFoundError(CameraError):
    """未找到可用的相机设备。"""


class CameraTimeoutError(CameraError):
    """相机 SDK 导入或帧等待超时。"""


# ── 检测器相关异常 ────────────────────────────────────────────────

class DetectorError(TechxVisionError):
    """模型加载或推理故障。"""


class ModelNotFoundError(DetectorError):
    """未找到模型文件（.pt / .onnx / .engine）。"""


class InferenceError(DetectorError):
    """推理运行时错误（如 GPU 显存不足等）。"""


# ── 追踪器相关异常 ────────────────────────────────────────────────

class TrackerError(TechxVisionError):
    """追踪逻辑错误。"""


# ── 发送器相关异常 ────────────────────────────────────────────────

class SenderError(TechxVisionError):
    """数据发送故障。"""


# ── 流水线相关异常 ────────────────────────────────────────────────

class PipelineError(TechxVisionError):
    """流水线编排错误。"""


# ── 配置相关异常 ──────────────────────────────────────────────────

class ConfigError(TechxVisionError):
    """配置加载或校验错误。"""
