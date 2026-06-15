#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <waypoint_manager/srv/select_point.hpp>
#include <waypoint_manager/srv/set_axis_control.hpp>

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

namespace waypoint_editor_rviz_plugin
{

class WaypointEditorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  WaypointEditorPanel(QWidget * parent = nullptr);
  ~WaypointEditorPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onRefreshClicked();
  void onTypeChanged(int);
  void onSelectClicked();

  void onXChanged(int);
  void onYChanged(int);
  void onZChanged(int);
  void onYawChanged(int);

  void onXReleased();
  void onYReleased();
  void onZReleased();
  void onYawReleased();

private:
  void ensureClients();
  void callSelect(const std::string &floor_index);
  void sendAxisControl(int8_t axis, float value);
  bool isGoalpoint() const;

private:
  rclcpp::Node::SharedPtr ros_node_;

  rclcpp::Client<waypoint_manager::srv::SelectPoint>::SharedPtr select_cli_;
  rclcpp::Client<waypoint_manager::srv::SetAxisControl>::SharedPtr control_cli_;

  QString select_srv_name_{"select_point"};
  QString control_srv_name_{"set_axis_control"};

  QComboBox * type_combo_{nullptr};
  QLineEdit * floor_index_edit_{nullptr};
  QPushButton * refresh_btn_{nullptr};
  QPushButton * select_btn_{nullptr};
  QLabel * status_label_{nullptr};

  QSlider * x_slider_{nullptr};
  QSlider * y_slider_{nullptr};
  QSlider * z_slider_{nullptr};
  QSlider * yaw_slider_{nullptr};

  QTimer * auto_refresh_timer_{nullptr};
};

}  // namespace waypoint_editor_rviz_plugin

