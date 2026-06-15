"""所有可插拔组件的抽象基类。

设计原则：每个组件实现且仅实现一个接口。
若要更换相机 / 检测器 / 追踪器 / 发送器，只需实现对应的
抽象基类并将其传入 PipelineEngine 即可。

用法示例：
    class MyCamera(ICamera):
        def open(self) -> bool: ...
        def grab(self) -> Optional[Frame]: ...
        def close(self) -> None: ...
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Optional

# 前向引用 —— 具体类型定义在 types.py 中，但接口层不应导入具体模块，
# 以避免循环依赖。运行时通过 TYPE_CHECKING 守卫来解析类型。
from interface.types import Frame, Detection, Track, Target3D


# ═══════════════════════════════════════════════════════════════════
#  ICamera  — 图像 / 深度传感器接口
# ═══════════════════════════════════════════════════════════════════

class ICamera(ABC):
    """相机 / 深度传感器的抽象接口。

    相机必须提供：
      - 彩色帧      (BGR, uint8)
      - 深度帧      (float32, 单位毫米) —— 纯 RGB 相机可为 None
      - 系统时间戳  (float64 秒, 单调时钟 / NTP 同步)
    """

    @abstractmethod
    def open(self) -> bool:
        """初始化设备。成功返回 True。"""
        ...

    @abstractmethod
    def grab(self) -> Optional[Frame]:
        """获取最新的一帧数据（若不可用则返回 None）。

        必须是非阻塞的 —— 立即返回。
        帧的时间戳必须是相机 SDK 提供的系统时间戳，
        而非在用户代码中调用 time.time() 获得的时间。
        """
        ...

    @abstractmethod
    def close(self) -> None:
        """释放设备。幂等操作（可安全重复调用）。"""
        ...

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """设备当前是否可用。"""
        ...

    @property
    @abstractmethod
    def width(self) -> int:
        """帧宽度，单位像素。"""
        ...

    @property
    @abstractmethod
    def height(self) -> int:
        """帧高度，单位像素。"""
        ...

    @property
    @abstractmethod
    def intrinsics(self) -> tuple:
        """返回相机内参 (fx, fy, cx, cy)，未知时返回零值。"""
        ...

    @property
    @abstractmethod
    def has_depth(self) -> bool:
        """此相机是否产生深度图。"""
        ...


# ═══════════════════════════════════════════════════════════════════
#  IDetector  — 目标检测模型接口
# ═══════════════════════════════════════════════════════════════════

class IDetector(ABC):
    """目标检测器的抽象接口。

    接收一帧 Frame，返回一个 Detection 列表。
    具体实现可以是 YOLO、ONNX、TensorRT 或其他任意模型。
    """

    @abstractmethod
    def load(self) -> bool:
        """加载模型权重。成功返回 True。"""
        ...

    @abstractmethod
    def detect(self, frame: Frame) -> List[Detection]:
        """对给定帧执行推理，返回检测结果列表。

        frame 的时间戳由调用方保留在返回的 detection 中
        （即由调用方赋值，而非检测器自身）。
        """
        ...

    @property
    @abstractmethod
    def is_loaded(self) -> bool:
        """模型是否已加载完毕，可进行推理。"""
        ...

    @property
    @abstractmethod
    def class_names(self) -> dict:
        """返回 {类别ID: '类别名称'} 的映射字典。"""
        ...

    @property
    @abstractmethod
    def backend_name(self) -> str:
        """人类可读的后端标识，例如 'TensorRT (.engine)'。"""
        ...


# ═══════════════════════════════════════════════════════════════════
#  ITracker  — 多目标追踪接口
# ═══════════════════════════════════════════════════════════════════

class ITracker(ABC):
    """多目标追踪器的抽象接口。

    每帧接收一组 Detection，输出带持久 ID 的 Track 列表。
    """

    @abstractmethod
    def update(self, detections: List[Detection]) -> List[Track]:
        """将新的检测结果与已有追踪轨迹进行关联。

        返回当前活跃的追踪轨迹集合（包含已更新的轨迹）。
        """
        ...

    @abstractmethod
    def reset(self) -> None:
        """清除所有追踪轨迹。"""
        ...

    @property
    @abstractmethod
    def active_count(self) -> int:
        """当前活跃（已确认）的追踪轨迹数量。"""
        ...


# ═══════════════════════════════════════════════════════════════════
#  ISender  — 数据发送接口
# ═══════════════════════════════════════════════════════════════════

class ISender(ABC):
    """数据发送器的抽象接口（UDP、串口、CAN 等）。

    接收 Target3D 对象并将其传输至外部系统。
    """

    @abstractmethod
    def send(self, target: Target3D) -> bool:
        """发送一个目标数据。成功返回 True。

        必须是非阻塞的，并且不能抛出异常（崩溃安全）。
        """
        ...

    def send_many(self, targets: List[Target3D], timestamp: Optional[float] = None) -> int:
        """批量发送一帧目标列表。

        默认实现逐个调用 send()，保证旧 Sender 不需要改代码。
        UDP Sender 会覆盖此函数，用一个 V2 遥测包实时下发全部 KFS 目标；
        即使 targets 为空，也可使用 timestamp 发送 count=0 的状态包。
        """
        sent = 0
        for target in targets:
            if self.send(target):
                sent += 1
        return sent

    @abstractmethod
    def close(self) -> None:
        """释放资源。幂等操作（可安全重复调用）。"""
        ...

    @property
    @abstractmethod
    def is_ready(self) -> bool:
        """发送器当前是否可以接收数据。"""
        ...
