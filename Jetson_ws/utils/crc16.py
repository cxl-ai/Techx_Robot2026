"""CRC16-CCITT 纯函数实现 —— 零外部依赖。

算法：  CRC-16/CCITT-FALSE
  多项式：   0x1021  (x^16 + x^12 + x^5 + 1)
  初始值：   0xFFFF
  输入/输出反转： 否
  异或输出值：    0x0000

本实现与 STM32 HAL_CRC、Python crcmod.mkCrcFun(0x11021, 0xFFFF, False)
以及 GMK 接收器的硬件 CRC 引擎完全一致。
"""

from __future__ import annotations


def crc16_ccitt(data: bytes) -> int:
    """对给定字节数据计算 CCITT-CRC16 校验值。

    参数：
        data: 需要计算校验和的原始字节串。

    返回值：
        16 位 CRC 值（范围 0–65535）。
    """
    crc: int = 0xFFFF               # 初始值
    for byte in data:
        crc ^= byte << 8            # 将当前字节移入 CRC 高 8 位
        for _ in range(8):          # 逐位处理
            if crc & 0x8000:        # 最高位为 1 时执行多项式异或
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF           # 保持 16 位宽度
    return crc
