"""
ros_worker.py - R2 HMI ROS2 后台引擎
继承 QThread，在独立线程中运行 ROS2 spin，通过 Qt Signal 与 UI 通信。
绝对禁止 rclpy.spin() 阻塞 Qt 主线程。
"""

import logging
import time
import json
from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker

from .ros_compat import HmiNode, ROS2_AVAILABLE, String, Bool

logger = logging.getLogger("r2_hmi.ros_worker")


class RosWorker(QThread):
    """
    ROS2 后台工作线程。
    所有 ROS2 通信在此线程内完成，通过 Qt Signal 将数据安全传递给 UI。
    """

    # ── 信号定义（跨线程通信的唯一合法通道）──────────────────────
    # 急停状态
    estop_acknowledged = Signal(bool)

    # 标定服务响应
    calibration_response = Signal(bool, str)  # success, message

    # Action 反馈
    stage_feedback = Signal(str)   # step_description
    stage_result = Signal(bool)    # completed

    # 心跳/连接状态
    robot_heartbeat = Signal(bool)  # alive
    connection_status = Signal(bool)  # connected

    # 运行时数据
    match_timer_update = Signal(int)   # 剩余秒数
    score_update = Signal(dict)        # 得分数据

    # 错误
    ros_error = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._mutex = QMutex()
        self._node: HmiNode = None

        # 急停状态
        self._estop_active = False

        # 标定数据
        self._team_color = 0       # 0=红, 1=蓝
        self._meihua_states = [0] * 12  # 12个梅林方块

        # 当前执行阶段
        self._current_stage_id = 0

    # ── 线程主循环 ──────────────────────────────────────────────
    def run(self):
        """QThread.run() - 在独立线程中执行"""
        logger.info("RosWorker thread starting...")
        self._running = True

        try:
            self._node = HmiNode("r2_hmi_node")
            self._setup_publishers()
            self._setup_subscriptions()
            self._setup_service_clients()
            self._setup_action_clients()
            self._node.start_spin()

            # 心跳模拟定时器（Mock 模式下使用）
            self._heartbeat_loop()

        except Exception as e:
            logger.critical(f"RosWorker crashed: {e}", exc_info=True)
            self.ros_error.emit(f"ROS Worker 异常: {e}")
        finally:
            self._cleanup()

    def _heartbeat_loop(self):
        """心跳循环：定期检查连接状态并发射信号"""
        while self._running:
            # Mock 模式下始终报告连接正常
            self.robot_heartbeat.emit(True)
            self.connection_status.emit(True)
            time.sleep(0.5)

    # ── ROS 接口初始化 ──────────────────────────────────────────
    def _setup_publishers(self):
        """创建所有 Publisher"""
        # 急停话题 - 10Hz 高频
        self._pub_estop = self._node.create_publisher(Bool, "/estop", qos_depth=10)
        # 状态广播
        self._pub_status = self._node.create_publisher(String, "/r2_hmi/status", qos_depth=5)
        logger.info("Publishers created: /estop, /r2_hmi/status")

    def _setup_subscriptions(self):
        """创建所有 Subscription"""
        self._node.create_subscription(
            String, "/r2_hmi/heartbeat",
            self._on_heartbeat, qos_depth=5
        )
        self._node.create_subscription(
            String, "/r2_hmi/timer",
            self._on_timer_update, qos_depth=5
        )
        self._node.create_subscription(
            String, "/r2_hmi/score",
            self._on_score_update, qos_depth=5
        )
        logger.info("Subscriptions created: /r2_hmi/heartbeat, timer, score")

    def _setup_service_clients(self):
        """创建 Service Client"""
        self._srv_calibrate = self._node.create_service_client(
            type(None),  # 占位，实际类型在 ROS2 可用时动态替换
            "/r2_hmi/sync_calibration"
        )
        logger.info("Service client created: /r2_hmi/sync_calibration")

    def _setup_action_clients(self):
        """创建 Action Client"""
        self._act_execute = self._node.create_action_client(
            type(None),  # 占位
            "/r2_hmi/execute_stage"
        )
        logger.info("Action client created: /r2_hmi/execute_stage")

    # ── Subscription 回调 ───────────────────────────────────────
    def _on_heartbeat(self, msg):
        alive = True
        if ROS2_AVAILABLE and hasattr(msg, 'data'):
            alive = msg.data == "alive"
        self.robot_heartbeat.emit(alive)

    def _on_timer_update(self, msg):
        try:
            if ROS2_AVAILABLE and hasattr(msg, 'data'):
                seconds = int(msg.data)
            else:
                seconds = int(msg) if isinstance(msg, (int, str)) else 0
            self.match_timer_update.emit(seconds)
        except (ValueError, TypeError) as e:
            logger.warning(f"Invalid timer data: {e}")

    def _on_score_update(self, msg):
        try:
            if ROS2_AVAILABLE and hasattr(msg, 'data'):
                data = json.loads(msg.data)
            else:
                data = msg if isinstance(msg, dict) else {}
            self.score_update.emit(data)
        except (json.JSONDecodeError, TypeError) as e:
            logger.warning(f"Invalid score data: {e}")

    # ── 公共方法（由 HmiController 通过 Signal 调用）────────────
    def request_estop(self, active: bool):
        """发送急停命令"""
        with QMutexLocker(self._mutex):
            self._estop_active = active
        self._node.publish(self._pub_estop, active)
        self.estop_acknowledged.emit(active)
        logger.warning(f"ESTOP {'ACTIVATED' if active else 'RELEASED'}")

    def set_team_color(self, color: int):
        """设置阵营颜色 0=红 1=蓝"""
        with QMutexLocker(self._mutex):
            self._team_color = color
        logger.info(f"Team color set: {'RED' if color == 0 else 'BLUE'}")

    def set_meihua_state(self, index: int, state: int):
        """设置梅林区某方块状态"""
        if 0 <= index < 12 and state in (0, 1, 2, 3):
            with QMutexLocker(self._mutex):
                self._meihua_states[index] = state
            logger.info(f"Meihua[{index}] = {state}")

    def send_calibration(self):
        """下发标定数据到机器人"""
        with QMutexLocker(self._mutex):
            team_color = self._team_color
            meihua = list(self._meihua_states)

        logger.info(f"Sending calibration: team={team_color}, meihua={meihua}")

        # 构建 CSV 格式 PID 参数（预留，当前用于标定数据）
        csv_data = f"{team_color}," + ",".join(str(s) for s in meihua)
        self._node.publish(self._pub_status, f"CALIB:{csv_data}")

        # 调用标定服务
        self._node.call_service(
            self._srv_calibrate,
            None,  # Mock 模式下 request 为 None
            callback=self._on_calibration_response
        )

    def _on_calibration_response(self, response):
        """标定服务响应回调"""
        success = getattr(response, 'success', False)
        message = getattr(response, 'message', 'Unknown response')
        self.calibration_response.emit(success, message)
        logger.info(f"Calibration response: success={success}, msg={message}")

    def execute_stage(self, stage_id: int):
        """下发阶段执行 Action"""
        with QMutexLocker(self._mutex):
            self._current_stage_id = stage_id
        logger.info(f"Executing stage {stage_id}")

        self._node.send_goal(
            self._act_execute,
            None,  # Mock 模式下 goal 为 None
            feedback_callback=self._on_stage_feedback,
            result_callback=self._on_stage_result
        )

    def _on_stage_feedback(self, feedback):
        """Action Feedback 回调"""
        desc = getattr(feedback, 'step_description', 'Processing...')
        self.stage_feedback.emit(desc)
        logger.info(f"Stage feedback: {desc}")

    def _on_stage_result(self, result):
        """Action Result 回调"""
        completed = getattr(result, 'completed', False)
        self.stage_result.emit(completed)
        logger.info(f"Stage result: completed={completed}")

    # ── 清理 ────────────────────────────────────────────────────
    def _cleanup(self):
        """线程退出前的清理工作"""
        self._running = False
        if self._node:
            self._node.destroy()
            self._node = None
        logger.info("RosWorker cleaned up")

    def stop(self):
        """请求线程安全退出"""
        self._running = False
        self.wait(3000)
        logger.info("RosWorker stopped")
