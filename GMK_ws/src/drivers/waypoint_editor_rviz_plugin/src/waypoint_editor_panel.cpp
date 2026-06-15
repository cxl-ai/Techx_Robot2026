#include "waypoint_editor_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace waypoint_editor_rviz_plugin
{

WaypointEditorPanel::WaypointEditorPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout();

  auto * row1 = new QHBoxLayout();
  row1->addWidget(new QLabel("类型:"));
  type_combo_ = new QComboBox();
  type_combo_->addItem("waypoint");
  type_combo_->addItem("goalpoint");
  row1->addWidget(type_combo_);
  row1->addStretch();
  root->addLayout(row1);

  auto * row2 = new QHBoxLayout();
  row2->addWidget(new QLabel("floor-index:"));
  floor_index_edit_ = new QLineEdit();
  floor_index_edit_->setPlaceholderText("例如 2-3");
  floor_index_edit_->setMinimumWidth(140);
  row2->addWidget(floor_index_edit_);
  refresh_btn_ = new QPushButton("刷新");
  row2->addWidget(refresh_btn_);
  select_btn_ = new QPushButton("选择");
  row2->addWidget(select_btn_);
  row2->addStretch();
  root->addLayout(row2);

  auto * grid = new QGridLayout();
  // 4个回中滑杆：连续值 [-1000, 1000] 映射到 control value [-1,1]
  x_slider_ = new QSlider(Qt::Horizontal);
  y_slider_ = new QSlider(Qt::Horizontal);
  z_slider_ = new QSlider(Qt::Horizontal);
  yaw_slider_ = new QSlider(Qt::Horizontal);

  for (auto * s : {x_slider_, y_slider_, z_slider_, yaw_slider_}) {
    s->setRange(-1000, 1000);
    s->setValue(0);
    s->setSingleStep(1);
    s->setPageStep(50);
    s->setTracking(true);  // valueChanged 连续触发
  }

  grid->addWidget(new QLabel("x"), 0, 0);
  grid->addWidget(x_slider_, 0, 1);
  grid->addWidget(new QLabel("y"), 1, 0);
  grid->addWidget(y_slider_, 1, 1);
  grid->addWidget(new QLabel("z"), 2, 0);
  grid->addWidget(z_slider_, 2, 1);
  grid->addWidget(new QLabel("yaw"), 3, 0);
  grid->addWidget(yaw_slider_, 3, 1);
  root->addLayout(grid);

  status_label_ = new QLabel("未初始化");
  root->addWidget(status_label_);

  setLayout(root);

  connect(refresh_btn_, SIGNAL(clicked()), this, SLOT(onRefreshClicked()));
  connect(type_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(onTypeChanged(int)));
  connect(select_btn_, SIGNAL(clicked()), this, SLOT(onSelectClicked()));
  connect(floor_index_edit_, SIGNAL(returnPressed()), this, SLOT(onSelectClicked()));

  connect(x_slider_, SIGNAL(valueChanged(int)), this, SLOT(onXChanged(int)));
  connect(y_slider_, SIGNAL(valueChanged(int)), this, SLOT(onYChanged(int)));
  connect(z_slider_, SIGNAL(valueChanged(int)), this, SLOT(onZChanged(int)));
  connect(yaw_slider_, SIGNAL(valueChanged(int)), this, SLOT(onYawChanged(int)));

  connect(x_slider_, SIGNAL(sliderReleased()), this, SLOT(onXReleased()));
  connect(y_slider_, SIGNAL(sliderReleased()), this, SLOT(onYReleased()));
  connect(z_slider_, SIGNAL(sliderReleased()), this, SLOT(onZReleased()));
  connect(yaw_slider_, SIGNAL(sliderReleased()), this, SLOT(onYawReleased()));

  auto_refresh_timer_ = new QTimer(this);
  auto_refresh_timer_->setInterval(1500);
  connect(auto_refresh_timer_, &QTimer::timeout, this, &WaypointEditorPanel::onRefreshClicked);
}

void WaypointEditorPanel::onInitialize()
{
  ros_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  ensureClients();
  status_label_->setText("已初始化：请先启动 waypoint_editor_node");
  auto_refresh_timer_->start();
}

bool WaypointEditorPanel::isGoalpoint() const
{
  return type_combo_ && type_combo_->currentIndex() == 1;
}

void WaypointEditorPanel::ensureClients()
{
  if (!ros_node_) {
    return;
  }
  if (!select_cli_) {
    select_cli_ = ros_node_->create_client<waypoint_manager::srv::SelectPoint>(
      select_srv_name_.toStdString());
  }
  if (!control_cli_) {
    control_cli_ = ros_node_->create_client<waypoint_manager::srv::SetAxisControl>(
      control_srv_name_.toStdString());
  }
}

void WaypointEditorPanel::onRefreshClicked()
{
  ensureClients();
  // floor-index 改为输入框，这里不再 list；仅提示服务状态
  if (!select_cli_ || !control_cli_) {
    status_label_->setText("ROS client 未就绪");
    return;
  }
  if (!select_cli_->wait_for_service(std::chrono::milliseconds(50))) {
    status_label_->setText("等待服务 select_point ...");
    return;
  }
  if (!control_cli_->wait_for_service(std::chrono::milliseconds(50))) {
    status_label_->setText("等待服务 set_axis_control ...");
    return;
  }
  status_label_->setText("服务就绪：输入 floor-index 后点“选择”，拖动滑杆调整（松手回零）");
}

void WaypointEditorPanel::onTypeChanged(int)
{
  onRefreshClicked();
}

void WaypointEditorPanel::onSelectClicked()
{
  if (!floor_index_edit_) {
    return;
  }
  const auto s = floor_index_edit_->text().trimmed().toStdString();
  if (s.empty()) {
    status_label_->setText("请输入 floor-index，例如 2-3");
    return;
  }
  callSelect(s);
}

void WaypointEditorPanel::callSelect(const std::string &floor_index)
{
  if (!select_cli_) {
    status_label_->setText("select_point client 未就绪");
    return;
  }
  if (!select_cli_->wait_for_service(std::chrono::milliseconds(100))) {
    status_label_->setText("等待服务 select_point ...");
    return;
  }

  auto req = std::make_shared<waypoint_manager::srv::SelectPoint::Request>();
  req->is_goalpoint = isGoalpoint();
  req->floor_index = floor_index;
  (void)select_cli_->async_send_request(
    req,
    [this](rclcpp::Client<waypoint_manager::srv::SelectPoint>::SharedFuture f) {
      const auto resp = f.get();
      QMetaObject::invokeMethod(
        this,
        [this, resp]() {
          status_label_->setText(QString::fromStdString(resp->message));
        },
        Qt::QueuedConnection);
    });
}

void WaypointEditorPanel::sendAxisControl(int8_t axis, float value)
{
  ensureClients();
  if (!control_cli_) {
    return;
  }
  if (!control_cli_->wait_for_service(std::chrono::milliseconds(50))) {
    status_label_->setText("等待服务 set_axis_control ...");
    return;
  }
  auto req = std::make_shared<waypoint_manager::srv::SetAxisControl::Request>();
  req->axis = axis;
  req->value = std::max(-1.0f, std::min(1.0f, value));
  (void)control_cli_->async_send_request(req);
}

static float sliderToValue(int v) { return static_cast<float>(v) / 1000.0f; }

void WaypointEditorPanel::onXChanged(int v) { sendAxisControl(0, sliderToValue(v)); }
void WaypointEditorPanel::onYChanged(int v) { sendAxisControl(1, sliderToValue(v)); }
void WaypointEditorPanel::onZChanged(int v) { sendAxisControl(2, sliderToValue(v)); }
void WaypointEditorPanel::onYawChanged(int v) { sendAxisControl(3, sliderToValue(v)); }

void WaypointEditorPanel::onXReleased() { x_slider_->setValue(0); sendAxisControl(0, 0.0f); }
void WaypointEditorPanel::onYReleased() { y_slider_->setValue(0); sendAxisControl(1, 0.0f); }
void WaypointEditorPanel::onZReleased() { z_slider_->setValue(0); sendAxisControl(2, 0.0f); }
void WaypointEditorPanel::onYawReleased() { yaw_slider_->setValue(0); sendAxisControl(3, 0.0f); }

}  // namespace waypoint_editor_rviz_plugin

PLUGINLIB_EXPORT_CLASS(waypoint_editor_rviz_plugin::WaypointEditorPanel, rviz_common::Panel)

