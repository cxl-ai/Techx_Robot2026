#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MapPcdPublisher : public rclcpp::Node
{
public:
  MapPcdPublisher()
  : Node("map_pcd_publisher")
  {
    this->declare_parameter<std::string>("pcd_path", "");
    this->declare_parameter<std::string>("frame_id", "world");

    frame_id_ = this->get_parameter("frame_id").as_string();

    auto pcd_path_param = this->get_parameter("pcd_path").as_string();
    if (pcd_path_param.empty()) {
      std::string share = ament_index_cpp::get_package_share_directory("relocalization");
      pcd_path_ = share + "/map/0_0.pcd";
    } else {
      pcd_path_ = pcd_path_param;
    }

    // transient_local: late-joiner (e.g., RViz) can receive the last map
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();
    
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", qos);

    if (!load_pcd_to_msg()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", pcd_path_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded map PCD: %s", pcd_path_.c_str());
      RCLCPP_INFO(this->get_logger(), "Publishing to topic: /Laser_map (frame_id=%s)", frame_id_.c_str());
      RCLCPP_INFO(this->get_logger(), "Map will be published at 0.5Hz");
      
      // 创建定时器，每2秒（0.5Hz）发布一次地图
      timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&MapPcdPublisher::publish_once, this)
      );
      
      // 立即发布一次
      publish_once();
    }
  }

private:
  bool load_pcd_to_msg()
  {
    // Try XYZINormal first (common for fast_lio style PointType)
    {
      pcl::PointCloud<pcl::PointXYZINormal> cloud;
      if (pcl::io::loadPCDFile(pcd_path_, cloud) == 0 && !cloud.empty()) {
        pcl::toROSMsg(cloud, cloud_msg_);
        loaded_ = true;
        return true;
      }
    }

    // Fallback to XYZI
    {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      if (pcl::io::loadPCDFile(pcd_path_, cloud) == 0 && !cloud.empty()) {
        pcl::toROSMsg(cloud, cloud_msg_);
        loaded_ = true;
        return true;
      }
    }

    loaded_ = false;
    return false;
  }

  void publish_once()
  {
    if (!loaded_) return;
    cloud_msg_.header.stamp = this->now();
    cloud_msg_.header.frame_id = frame_id_;
    pub_->publish(cloud_msg_);
  }

private:
  std::string pcd_path_;
  std::string frame_id_;

  bool loaded_ = false;
  sensor_msgs::msg::PointCloud2 cloud_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPcdPublisher>());
  rclcpp::shutdown();
  return 0;
}


