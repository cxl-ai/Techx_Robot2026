# 三维目标检测信息
# 由 techx_vision_bridge 节点发布，携带上游视觉系统解算的单目标位姿

std_msgs/Header header  # ROS 2 标准头（含时间戳和 frame_id）

uint8 track_id          # 目标跟踪 ID（0~255），在 frame_id 中也有体现

float32 x               # 目标在相机坐标系下的 X 坐标（米）
float32 y               # 目标在相机坐标系下的 Y 坐标（米）
float32 z               # 目标在相机坐标系下的 Z 坐标（米）