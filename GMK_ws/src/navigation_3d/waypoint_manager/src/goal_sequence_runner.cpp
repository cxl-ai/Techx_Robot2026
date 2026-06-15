#include "waypoint_search/goal_sequence_runner.hpp"

#include <utility>

namespace goal_sequence_runner {

GoalSequenceRunner::GoalSequenceRunner(std::vector<std::string> floor_indices)
    : floor_indices_(std::move(floor_indices)) {
  if (floor_indices_.empty()) {
    state_ = SeqState::Completed;
    current_index_ = 0;
  } else {
    state_ = SeqState::WaitingForOdom;
    current_index_ = 0;
  }
}

void GoalSequenceRunner::onFirstOdom() {
  if (!enabled()) {
    state_ = SeqState::Completed;
    return;
  }
  if (state_ == SeqState::WaitingForOdom) {
    state_ = SeqState::Running;
  }
}

void GoalSequenceRunner::onManualGoalSelected() {
  if (!enabled()) {
    return;
  }
  // 手动插队：暂停自动序列（等待 odom 阶段也暂停，等手动完成后再继续）
  if (state_ == SeqState::Running || state_ == SeqState::WaitingForOdom) {
    state_ = SeqState::PausedForManual;
  }
}

void GoalSequenceRunner::onManualGoalCompleted() {
  if (!enabled()) {
    return;
  }
  if (state_ == SeqState::PausedForManual) {
    // 恢复自动序列：仍然需要 odom？我们假设能完成手动目标则 odom 已经就绪
    state_ = SeqState::Running;
  }
}

void GoalSequenceRunner::onAutoGoalCompleted() {
  if (!enabled()) {
    return;
  }
  if (state_ != SeqState::Running) {
    // 自动目标完成信号只有在 Running 时才推进
    return;
  }
  advanceIndexOrComplete();
}

void GoalSequenceRunner::skipCurrentAutoGoal() {
  if (!enabled()) {
    return;
  }
  // 无论当前处于什么状态，只要还没 Completed，就允许跳过（避免卡死）
  if (state_ == SeqState::Completed) {
    return;
  }
  advanceIndexOrComplete();
  if (!completed() && state_ != SeqState::PausedForManual) {
    // skip 后默认进入 Running（除非处于手动暂停）
    state_ = SeqState::Running;
  }
}

std::optional<std::string> GoalSequenceRunner::currentAutoFloorIndex() const {
  if (state_ != SeqState::Running) {
    return std::nullopt;
  }
  if (current_index_ >= floor_indices_.size()) {
    return std::nullopt;
  }
  return floor_indices_[current_index_];
}

void GoalSequenceRunner::advanceIndexOrComplete() {
  if (current_index_ < floor_indices_.size()) {
    ++current_index_;
  }
  if (current_index_ >= floor_indices_.size()) {
    state_ = SeqState::Completed;
  }
}

}  // namespace goal_sequence_runner

