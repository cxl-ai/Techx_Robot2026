#pragma once

#include <optional>
#include <string>
#include <vector>

// Linus 风格：数据结构 + 明确的状态机接口
// - 不依赖 ROS2 类型
// - 不做复杂回调/继承
// - Node 负责“什么时候调用”，Runner 只负责“应该做什么”

namespace goal_sequence_runner {

enum class SeqState {
  // 未启用（floor_indices 为空） or 已经跑完
  Completed = 0,
  // 已配置序列，但还没等到首次 /Odometry
  WaitingForOdom,
  // 正在跑自动序列（当前 index 未完成前不会前进）
  Running,
  // 用户手动 select_goal 插队，自动序列暂停
  PausedForManual,
};

class GoalSequenceRunner {
public:
  GoalSequenceRunner() = default;
  explicit GoalSequenceRunner(std::vector<std::string> floor_indices);

  bool enabled() const { return !floor_indices_.empty(); }
  SeqState state() const { return state_; }
  bool completed() const { return state_ == SeqState::Completed; }

  // 首次 odom 到来时调用：让序列从 WaitingForOdom 进入 Running
  void onFirstOdom();

  // 手动 select_goal 成功时调用：自动序列暂停（仅在 Running/WaitingForOdom 有效）
  void onManualGoalSelected();

  // 手动目标完成时调用：若处于暂停，则恢复自动序列
  void onManualGoalCompleted();

  // 自动序列当前目标（floor-index）完成：推进到下一个
  void onAutoGoalCompleted();

  // 当前自动目标无法执行（解析失败/CSV不存在/规划失败）：跳过当前项推进
  void skipCurrentAutoGoal();

  // 如果当前应该发布一个自动目标（state==Running 且还有剩余），返回 floor-index
  std::optional<std::string> currentAutoFloorIndex() const;

  // 当前执行到序列的第几个（0-based）
  size_t currentIndex() const { return current_index_; }
  size_t total() const { return floor_indices_.size(); }

private:
  void advanceIndexOrComplete();

  std::vector<std::string> floor_indices_;
  size_t current_index_{0};
  SeqState state_{SeqState::Completed};
};

}  // namespace goal_sequence_runner

