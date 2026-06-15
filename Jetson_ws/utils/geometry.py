"""几何工具函数 —— IoU 计算（统一给 detector 和 tracker 使用）。"""

from __future__ import annotations

import numpy as np


def box_iou(b1: np.ndarray, b2: np.ndarray) -> float:
    """计算两个 [x1, y1, x2, y2] 格式边界框的交并比 (IoU)。

    计算公式:
        IoU = 交集面积 / 并集面积
        交集面积 = max(0, x2_intersect - x1_intersect) × max(0, y2_intersect - y1_intersect)
        并集面积 = a1 + a2 - 交集面积

    参数:
        b1: 第一个边界框，格式为 [x1, y1, x2, y2]。
        b2: 第二个边界框，格式为 [x1, y1, x2, y2]。

    返回值:
        IoU 值，范围 [0.0, 1.0]。并集面积为 0 时返回 0.0。
    """
    # 计算交集矩形的左上角和右下角坐标
    x1 = max(b1[0], b2[0])
    y1 = max(b1[1], b2[1])
    x2 = min(b1[2], b2[2])
    y2 = min(b1[3], b2[3])

    # 交集面积 = 宽 × 高，负值截断为 0
    inter = max(0.0, float(x2 - x1)) * max(0.0, float(y2 - y1))

    # 两个框各自的面积
    a1 = float((b1[2] - b1[0]) * (b1[3] - b1[1]))
    a2 = float((b2[2] - b2[0]) * (b2[3] - b2[1]))

    # 并集面积 = a1 + a2 - 交集面积
    denom = a1 + a2 - inter
    return inter / denom if denom > 0 else 0.0
