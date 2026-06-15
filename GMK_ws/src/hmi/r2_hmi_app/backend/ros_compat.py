"""
ros_compat.py - ROS2 兼容层
自动检测 ROS2 环境：若 rclpy 可用则使用真实后端，否则切换为 Mock 模式。
保证 HMI 在无 ROS2 的开发机上也能独立运行和测试。
"""

import logging
import threading
import time
from typing import Optional, Callable, Any

logger = logging.getLogger("r2_hmi.ros_compat")

# ── 尝试导入 ROS2 ──────────────────────────────────────────────
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, Bool, Int8
    from action_msgs.msg import GoalStatus
    ROS2_AVAILABLE = True
    logger.info("ROS2 rclpy detected - running in LIVE mode")
except ImportError:
    logger.warning("ROS2 rclpy NOT found - running in MOCK mode (simulated backend)")

# ── Mock 消息类 (当 ROS2 不可用时) ─────────────────────────────
if not ROS2_AVAILABLE:
    class _MockMsg:
        """Mock 基础消息"""
        def __init__(self, data=None):
            self.data = data

    class String(_MockMsg):
        pass

    class Bool(_MockMsg):
        pass

    class Int8(_MockMsg):
        pass

    class GoalStatus:
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_SUCCEEDED = 4
        STATUS_ABORTED = 6

    class QoSProfile:
        def __init__(self, reliability=None, history=None, depth=10):
            self.depth = depth

    class ReliabilityPolicy:
        BEST_EFFORT = 1
        RELIABLE = 2

    class HistoryPolicy:
        KEEP_LAST = 1
        KEEP_ALL = 2


# ── 统一接口：ROS2 节点封装 ────────────────────────────────────
class HmiNode:
    """
    ROS2 节点封装，提供 Publisher / Service Client / Action Client 的统一接口。
    Mock 模式下所有操作模拟真实行为（含延迟和回调）。
    """

    def __init__(self, node_name: str = "r2_hmi_node"):
        self._node_name = node_name
        self._publishers = {}
        self._subscriptions = {}
        self._service_clients = {}
        self._action_clients = {}
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False

        if ROS2_AVAILABLE:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node(node_name)
            logger.info(f"ROS2 Node '{node_name}' created (LIVE)")
        else:
            self._node = None
            self._mock_state = {}
            logger.info(f"Mock Node '{node_name}' created (MOCK)")

    # ── Publisher ───────────────────────────────────────────────
    def create_publisher(self, msg_type, topic: str, qos_depth: int = 10):
        key = f"pub_{topic}"
        if ROS2_AVAILABLE:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=qos_depth
            )
            pub = self._node.create_publisher(msg_type, topic, qos)
            self._publishers[key] = pub
        else:
            self._publishers[key] = {"topic": topic, "msg_type": msg_type}
        return key

    def publish(self, key: str, data: Any):
        if key not in self._publishers:
            logger.error(f"Publisher key '{key}' not found")
            return
        if ROS2_AVAILABLE:
            msg_type = type(self._publishers[key].msg)
            msg = msg_type()
            if hasattr(msg, 'data'):
                msg.data = data
            self._publishers[key].publish(msg)
        else:
            logger.debug(f"[MOCK PUB] {self._publishers[key]['topic']} -> {data}")

    # ── Subscription ────────────────────────────────────────────
    def create_subscription(self, msg_type, topic: str, callback: Callable, qos_depth: int = 10):
        key = f"sub_{topic}"
        if ROS2_AVAILABLE:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=qos_depth
            )
            sub = self._node.create_subscription(msg_type, topic, callback, qos)
            self._subscriptions[key] = sub
        else:
            self._subscriptions[key] = {"topic": topic, "callback": callback}
        return key

    # ── Service Client ──────────────────────────────────────────
    def create_service_client(self, srv_type, service_name: str):
        key = f"srv_{service_name}"
        if ROS2_AVAILABLE:
            client = self._node.create_client(srv_type, service_name)
            self._service_clients[key] = client
        else:
            self._service_clients[key] = {"name": service_name, "srv_type": srv_type}
        return key

    def call_service(self, key: str, request: Any, callback: Optional[Callable] = None,
                     timeout_sec: float = 5.0):
        if key not in self._service_clients:
            logger.error(f"Service client key '{key}' not found")
            return None

        if ROS2_AVAILABLE:
            client = self._service_clients[key]
            if not client.wait_for_service(timeout_sec=timeout_sec):
                logger.error(f"Service '{key}' not available after {timeout_sec}s")
                return None
            future = client.call_async(request)
            if callback:
                future.add_done_callback(lambda f: callback(f.result()))
            return future
        else:
            # Mock: 模拟成功响应
            def _mock_response():
                time.sleep(0.3)  # 模拟网络延迟
                if callback:
                    response = type('MockResponse', (), {
                        'success': True,
                        'message': 'Mock: calibration synced successfully'
                    })()
                    callback(response)
            t = threading.Thread(target=_mock_response, daemon=True)
            t.start()
            return None

    # ── Action Client ───────────────────────────────────────────
    def create_action_client(self, action_type, action_name: str):
        key = f"act_{action_name}"
        if ROS2_AVAILABLE:
            from rclpy.action import ActionClient
            client = ActionClient(self._node, action_type, action_name)
            self._action_clients[key] = client
        else:
            self._action_clients[key] = {"name": action_name, "action_type": action_type}
        return key

    def send_goal(self, key: str, goal: Any,
                  feedback_callback: Optional[Callable] = None,
                  result_callback: Optional[Callable] = None):
        if key not in self._action_clients:
            logger.error(f"Action client key '{key}' not found")
            return

        if ROS2_AVAILABLE:
            client = self._action_clients[key]
            if not client.wait_for_server(timeout_sec=5.0):
                logger.error(f"Action server '{key}' not available")
                return
            send_goal_future = client.send_goal_async(
                goal, feedback_callback=feedback_callback
            )
            if result_callback:
                send_goal_future.add_done_callback(
                    lambda f: self._action_result_cb(f, result_callback)
                )
        else:
            # Mock: 模拟 Action 执行（含 feedback）
            def _mock_action():
                steps = ["初始化导航", "前往武馆区", "执行KFS操作", "返回起点"]
                for i, step in enumerate(steps):
                    time.sleep(0.8)
                    if feedback_callback:
                        fb = type('MockFeedback', (), {
                            'step_description': f"[Mock] {step} ({i+1}/{len(steps)})"
                        })()
                        feedback_callback(fb)
                if result_callback:
                    result = type('MockResult', (), {'completed': True})()
                    result_callback(result)
            t = threading.Thread(target=_mock_action, daemon=True)
            t.start()

    def _action_result_cb(self, future, callback):
        result_future = future.result().get_result_async()
        result_future.add_done_callback(lambda f: callback(f.result().result))

    # ── Spin (非阻塞) ──────────────────────────────────────────
    def start_spin(self):
        self._running = True
        if ROS2_AVAILABLE:
            def _spin_loop():
                while self._running and rclpy.ok():
                    rclpy.spin_once(self._node, timeout_sec=0.1)
            self._spin_thread = threading.Thread(target=_spin_loop, daemon=True)
            self._spin_thread.start()
            logger.info("ROS2 spin thread started")
        else:
            logger.info("Mock mode - no spin thread needed")

    def stop_spin(self):
        self._running = False
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        logger.info("Spin thread stopped")

    # ── 销毁 ────────────────────────────────────────────────────
    def destroy(self):
        self.stop_spin()
        if ROS2_AVAILABLE and self._node:
            self._node.destroy_node()
            logger.info("ROS2 Node destroyed")
