"""
main.py - R2 HMI 主入口
PySide6 应用启动器，包含 HmiController 桥接类。
"""

import sys
import os
import logging
import signal as posix_signal
from pathlib import Path

# 修复 Windows 下 Qt 与 Anaconda 自带 Qt 冲突的问题
import PySide6
_pyside6_dir = os.path.dirname(PySide6.__file__)
_plugins_path = os.path.join(_pyside6_dir, "plugins")
_pyside6_qml = os.path.join(_pyside6_dir, "qml")
if os.path.exists(_plugins_path):
    os.environ["QT_PLUGIN_PATH"] = _plugins_path
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = os.path.join(_plugins_path, "platforms")
if os.path.exists(_pyside6_qml):
    os.environ["QML_IMPORT_PATH"] = _pyside6_qml
    os.environ["QML2_IMPORT_PATH"] = _pyside6_qml

from PySide6.QtCore import (
    QObject, Signal, Slot, QUrl, QTimer
)
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from backend.ros_worker import RosWorker

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("r2_hmi.main")


class HmiController(QObject):
    """
    QML 与 Python 之间的数据桥梁。
    所有 QML 界面触发的操作路由到 RosWorker，所有 ROS 数据通过 Signal 发射给 QML。
    """

    estopChanged = Signal(bool)
    matchTimerUpdated = Signal(int)
    stageFeedbackUpdated = Signal(str)
    stageCompleted = Signal(bool)
    connectionChanged = Signal(bool)
    calibrationResponse = Signal(bool, str)
    timerStarted = Signal(bool)  # 通知 QML 倒计时是否已启动

    def __init__(self, parent=None):
        super().__init__(parent)
        self._team_color = -1
        self._meihua_states = [0] * 12
        self._estop_active = False
        self._match_timer = 180
        self._timer_running = False
        self._connected = False

        self._ros_worker = RosWorker(self)

        self._ros_worker.estop_acknowledged.connect(self._on_estop_ack)
        self._ros_worker.calibration_response.connect(self._on_calib_response)
        self._ros_worker.stage_feedback.connect(self._on_stage_feedback)
        self._ros_worker.stage_result.connect(self._on_stage_result)
        self._ros_worker.robot_heartbeat.connect(self._on_heartbeat)
        self._ros_worker.connection_status.connect(self._on_connection)
        self._ros_worker.match_timer_update.connect(self._on_timer_update)
        self._ros_worker.ros_error.connect(self._on_ros_error)

        self._timer = QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self._tick_timer)

        logger.info("HmiController initialized")

    @Slot()
    def startMatchTimer(self):
        """QML 调用：启动比赛倒计时（仅执行赛段时触发）"""
        if self._timer_running:
            return
        self._timer_running = True
        self._match_timer = 180
        self._timer.start()
        self.timerStarted.emit(True)
        self.matchTimerUpdated.emit(self._match_timer)
        logger.info("Match timer started (180s)")

    @Slot()
    def resetMatchTimer(self):
        """QML 调用：重置倒计时"""
        self._timer.stop()
        self._timer_running = False
        self._match_timer = 180
        self.timerStarted.emit(False)
        self.matchTimerUpdated.emit(self._match_timer)

    def _tick_timer(self):
        if self._match_timer > 0:
            self._match_timer -= 1
            self.matchTimerUpdated.emit(self._match_timer)
        else:
            self._timer.stop()
            self._timer_running = False
            logger.info("Match time expired!")

    @Slot(int)
    def setTeamColor(self, color: int):
        self._team_color = color
        self._ros_worker.set_team_color(color)

    @Slot(int, int)
    def setMeihuaState(self, index: int, state: int):
        if 0 <= index < 12:
            self._meihua_states[index] = state
            self._ros_worker.set_meihua_state(index, state)

    @Slot()
    def sendCalibration(self):
        self._ros_worker.send_calibration()

    @Slot(bool)
    def requestEstop(self, active: bool):
        self._ros_worker.request_estop(active)

    @Slot(int)
    def executeStage(self, stage_id: int):
        if not self._timer_running:
            self.startMatchTimer()
        self._ros_worker.execute_stage(stage_id)

    def _on_estop_ack(self, active: bool):
        self._estop_active = active
        self.estopChanged.emit(active)

    def _on_calib_response(self, success: bool, message: str):
        self.calibrationResponse.emit(success, message)

    def _on_stage_feedback(self, text: str):
        self.stageFeedbackUpdated.emit(text)

    def _on_stage_result(self, completed: bool):
        self.stageCompleted.emit(completed)

    def _on_heartbeat(self, alive: bool):
        pass

    def _on_connection(self, connected: bool):
        if self._connected != connected:
            self._connected = connected
            self.connectionChanged.emit(connected)

    def _on_timer_update(self, seconds: int):
        self._match_timer = seconds
        self.matchTimerUpdated.emit(seconds)

    def _on_ros_error(self, error_msg: str):
        logger.error(f"ROS Error: {error_msg}")

    def start_ros(self):
        self._ros_worker.start()
        logger.info("ROS Worker thread started")

    def shutdown(self):
        logger.info("Shutting down HMI...")
        self._timer.stop()
        self._ros_worker.stop()
        logger.info("HMI shutdown complete")


def main():
    app = QGuiApplication(sys.argv)
    app.setApplicationName("R2 HMI")
    app.setOrganizationName("RoboconTeam")

    controller = HmiController()
    engine = QQmlApplicationEngine()

    if os.path.exists(_pyside6_qml):
        engine.addImportPath(_pyside6_qml)

    frontend_dir = str(PROJECT_ROOT / "frontend")
    engine.addImportPath(frontend_dir)

    def on_qml_warnings(warnings):
        for w in warnings:
            logger.warning(f"QML: {w.toString()}")
    engine.warnings.connect(on_qml_warnings)

    engine.rootContext().setContextProperty("HmiController", controller)

    qml_file = QUrl.fromLocalFile(str(PROJECT_ROOT / "frontend" / "Main.qml"))
    engine.load(qml_file)

    if not engine.rootObjects():
        logger.critical("QML loading failed - no root objects created")
        sys.exit(1)

    logger.info("QML engine loaded successfully")
    controller.start_ros()

    def cleanup():
        controller.shutdown()
        engine.deleteLater()
    app.aboutToQuit.connect(cleanup)

    posix_signal.signal(posix_signal.SIGINT, posix_signal.SIG_DFL)
    ret = app.exec()
    logger.info(f"Application exited with code {ret}")
    sys.exit(ret)


if __name__ == "__main__":
    main()