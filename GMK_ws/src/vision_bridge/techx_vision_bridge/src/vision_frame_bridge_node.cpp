#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <syslog.h>
#include <unistd.h>

#include "rcutils/logging.h"
#include "rclcpp/rclcpp.hpp"
#include "techx_vision_bridge/msg/target3_d.hpp"
#include "techx_vision_bridge/msg/vision_frame.hpp"
#include "techx_vision_bridge/msg/vision_object.hpp"
#include "techx_vision_bridge/msg/vision_request.hpp"
#include "techx_vision_bridge/msg/vision_selection.hpp"
#include "techx_vision_bridge/msg/vision_target.hpp"

namespace {
constexpr uint16_t MAGIC_LEGACY = 0x55AA;
constexpr uint16_t MAGIC_V2 = 0x55AB;
constexpr uint8_t VERSION_V2 = 2;
constexpr size_t MAX_TARGETS = 16;
constexpr uint8_t INVALID_INDEX = 255;

constexpr uint8_t ZONE_UNKNOWN = 0;
constexpr uint8_t ZONE_HEAD = 1;
constexpr uint8_t ZONE_KFS = 2;
constexpr uint8_t ZONE_QR = 3;

constexpr uint8_t TYPE_UNKNOWN = 0;
constexpr uint8_t TYPE_HEAD = 1;
constexpr uint8_t TYPE_KFS = 2;
constexpr uint8_t TYPE_QR = 3;

constexpr uint8_t FRAME_CAMERA_LINK = 1;
constexpr uint8_t FRAME_ROBOT_BASE = 2;
constexpr uint8_t FRAME_ARM1_BASE = 3;
constexpr uint8_t FRAME_ARM2_BASE = 4;

#pragma pack(push, 1)
struct LegacyPacket {
  uint16_t magic;
  uint32_t seq;
  double timestamp;
  uint8_t track_id;
  float x;
  float y;
  float z;
  uint16_t crc16;
};

struct V2Header {
  uint16_t magic;
  uint8_t version;
  uint8_t flags;
  uint32_t seq;
  double timestamp;
  uint8_t count;
};

struct V2Target {
  uint8_t track_id;
  uint8_t class_id;
  uint8_t color;
  float confidence;
  float u;
  float v;
  float x;
  float y;
  float z;
};
#pragma pack(pop)

static_assert(sizeof(LegacyPacket) == 29, "legacy size");
static_assert(sizeof(V2Header) == 17, "v2 header size");
static_assert(sizeof(V2Target) == 27, "v2 target size");

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Transform {
  std::array<std::array<double, 3>, 3> r{{
      std::array<double, 3>{1.0, 0.0, 0.0},
      std::array<double, 3>{0.0, 1.0, 0.0},
      std::array<double, 3>{0.0, 0.0, 1.0},
  }};
  Vec3 t{};
};

struct ClassRule {
  int min_id{0};
  int max_id{0};
  uint8_t zone_id{ZONE_UNKNOWN};
  uint8_t target_type{TYPE_UNKNOWN};
  uint8_t control_frame{FRAME_CAMERA_LINK};
  float priority_bias{0.0f};
};

struct DecodedTarget {
  uint8_t track_id{0};
  uint8_t class_id{255};
  uint8_t color{0};
  float confidence{0.0f};
  float u{0.0f};
  float v{0.0f};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  bool valid_xyz{false};

  bool valid_robot_xyz{false};
  float robot_x{0.0f};
  float robot_y{0.0f};
  float robot_z{0.0f};

  bool valid_arm1_xyz{false};
  float arm1_x{0.0f};
  float arm1_y{0.0f};
  float arm1_z{0.0f};

  bool valid_arm2_xyz{false};
  float arm2_x{0.0f};
  float arm2_y{0.0f};
  float arm2_z{0.0f};

  uint8_t zone_id{ZONE_UNKNOWN};
  uint8_t target_type{TYPE_UNKNOWN};
  uint8_t control_frame{FRAME_CAMERA_LINK};
  bool valid_control_xyz{false};
  float control_x{0.0f};
  float control_y{0.0f};
  float control_z{0.0f};

  float align_err_x{0.0f};
  float align_err_y{0.0f};
  float priority{0.0f};
  float priority_bias{0.0f};
};

struct DecodedFrame {
  uint8_t protocol_version{0};
  uint32_t seq{0};
  double timestamp{0.0};
  std::vector<DecodedTarget> targets;
};

constexpr std::array<uint16_t, 256> make_crc_table() {
  std::array<uint16_t, 256> table{};
  for (int i = 0; i < 256; ++i) {
    uint16_t crc = static_cast<uint16_t>(i << 8);
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    table[i] = crc;
  }
  return table;
}

constexpr auto CRC_TABLE = make_crc_table();

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(((crc << 8) & 0xFFFF) ^ CRC_TABLE[((crc >> 8) ^ data[i]) & 0xFF]);
  }
  return crc;
}

bool finite3(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

float finite_or_zero(float v) {
  return std::isfinite(v) ? v : 0.0f;
}

Transform make_transform_from_xyz_rpy(const std::vector<double> &p) {
  Transform tf{};
  if (p.size() != 6) return tf;
  tf.t = Vec3{p[0], p[1], p[2]};
  const double cr = std::cos(p[3]);
  const double sr = std::sin(p[3]);
  const double cp = std::cos(p[4]);
  const double sp = std::sin(p[4]);
  const double cy = std::cos(p[5]);
  const double sy = std::sin(p[5]);
  tf.r = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
           {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
           {-sp, cp * sr, cp * cr}}};
  return tf;
}

Vec3 apply_transform(const Transform &tf, const Vec3 &p) {
  return Vec3{
      tf.r[0][0] * p.x + tf.r[0][1] * p.y + tf.r[0][2] * p.z + tf.t.x,
      tf.r[1][0] * p.x + tf.r[1][1] * p.y + tf.r[1][2] * p.z + tf.t.y,
      tf.r[2][0] * p.x + tf.r[2][1] * p.y + tf.r[2][2] * p.z + tf.t.z,
  };
}

std::vector<std::string> split(const std::string &s, char sep) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, sep)) out.push_back(item);
  return out;
}

bool parse_range(const std::string &s, int &lo, int &hi) {
  auto parts = split(s, '-');
  try {
    if (parts.size() == 1) {
      lo = hi = std::stoi(parts[0]);
      return true;
    }
    if (parts.size() == 2) {
      lo = std::stoi(parts[0]);
      hi = std::stoi(parts[1]);
      if (hi < lo) std::swap(lo, hi);
      return true;
    }
  } catch (...) {
    return false;
  }
  return false;
}

ClassRule make_rule(int lo, int hi, uint8_t zone, uint8_t type, uint8_t frame, float bias = 0.0f) {
  ClassRule r;
  r.min_id = lo;
  r.max_id = hi;
  r.zone_id = zone;
  r.target_type = type;
  r.control_frame = frame;
  r.priority_bias = bias;
  return r;
}

std::vector<ClassRule> default_rules() {
  return {
      make_rule(0, 5, ZONE_KFS, TYPE_KFS, FRAME_ARM2_BASE, 0.0f),
      make_rule(100, 102, ZONE_HEAD, TYPE_HEAD, FRAME_ARM1_BASE, 0.0f),
      make_rule(200, 200, ZONE_QR, TYPE_QR, FRAME_ROBOT_BASE, 0.0f),
  };
}

std::vector<ClassRule> parse_rules(const std::vector<std::string> &raw) {
  std::vector<ClassRule> rules;
  for (const auto &line : raw) {
    auto parts = split(line, ':');
    if (parts.size() < 4) continue;
    int lo = 0;
    int hi = 0;
    if (!parse_range(parts[0], lo, hi)) continue;
    try {
      const auto zone = static_cast<uint8_t>(std::stoi(parts[1]));
      const auto type = static_cast<uint8_t>(std::stoi(parts[2]));
      const auto frame = static_cast<uint8_t>(std::stoi(parts[3]));
      const float bias = parts.size() >= 5 ? std::stof(parts[4]) : 0.0f;
      rules.push_back(make_rule(lo, hi, zone, type, frame, bias));
    } catch (...) {
      continue;
    }
  }
  return rules.empty() ? default_rules() : rules;
}
}  // namespace

class VisionFrameBridgeNode : public rclcpp::Node {
 public:
  VisionFrameBridgeNode() : Node("vision_bridge_node") {
    declare_parameters();
    load_parameters();
    create_publishers_and_subscribers();
    open_socket();
    last_valid_ = std::chrono::steady_clock::now();
    recv_timer_ = create_wall_timer(std::chrono::milliseconds(2), std::bind(&VisionFrameBridgeNode::recv_once, this));
    watchdog_timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&VisionFrameBridgeNode::watchdog, this));
    stats_timer_ = create_wall_timer(std::chrono::seconds(10), std::bind(&VisionFrameBridgeNode::log_stats, this));
    stats_reset_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(),
                "vision bridge ready udp=%s:%d accept_legacy=%s transforms=%s selector=%s fatal_no_udp=%.1fs rules=%zu",
                bind_addr_.c_str(), udp_port_, accept_legacy_ ? "true" : "false",
                enable_transforms_ ? "true" : "false", enable_request_selector_ ? "true" : "false",
                fatal_no_udp_timeout_sec_, class_rules_.size());
  }

  ~VisionFrameBridgeNode() override {
    if (sock_ >= 0) close(sock_);
  }

 private:
  void declare_parameters() {
    declare_parameter("udp_bind_addr", "0.0.0.0");
    declare_parameter("udp_port", 12345);
    declare_parameter("frame_topic_name", "/techx/vision/frame");
    declare_parameter("object_topic_name", "/techx/vision/objects");
    declare_parameter("detail_topic_name", "/techx/vision/kfs_targets");
    declare_parameter("topic_name", "/techx/vision/targets");
    declare_parameter("request_topic_name", "/techx/vision/request");
    declare_parameter("selected_topic_name", "/techx/vision/selected");
    declare_parameter("publish_frame_topic", true);
    declare_parameter("publish_object_topic", true);
    declare_parameter("publish_detail_topic", false);
    declare_parameter("publish_legacy_topic", false);
    declare_parameter("accept_legacy", false);
    declare_parameter("enable_request_selector", true);
    declare_parameter("reliable_qos", true);
    declare_parameter("qos_depth", 5);
    declare_parameter("image_width", 640.0);
    declare_parameter("image_height", 480.0);
    declare_parameter<std::vector<std::string>>("class_rules", {"0-5:2:2:4:0.0", "100-102:1:1:3:0.0", "200:3:3:2:0.0"});
    declare_parameter("enable_transforms", true);
    declare_parameter<std::vector<double>>("T_robot_camera_xyz_rpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("T_arm1_robot_xyz_rpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("T_arm2_robot_xyz_rpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    declare_parameter("watchdog_timeout_sec", 0.3);
    declare_parameter("fatal_no_udp_timeout_sec", 600.0);
    declare_parameter("request_timeout_sec", 0.0);
    declare_parameter("default_max_frame_age_sec", 0.20);
    declare_parameter("publish_period_ms", 50);
  }

  void load_parameters() {
    bind_addr_ = get_parameter("udp_bind_addr").as_string();
    udp_port_ = get_parameter("udp_port").as_int();
    publish_frame_ = get_parameter("publish_frame_topic").as_bool();
    publish_object_ = get_parameter("publish_object_topic").as_bool();
    publish_detail_ = get_parameter("publish_detail_topic").as_bool();
    publish_legacy_ = get_parameter("publish_legacy_topic").as_bool();
    accept_legacy_ = get_parameter("accept_legacy").as_bool();
    enable_request_selector_ = get_parameter("enable_request_selector").as_bool();
    reliable_qos_ = get_parameter("reliable_qos").as_bool();
    qos_depth_ = std::max(1, static_cast<int>(get_parameter("qos_depth").as_int()));
    image_width_ = get_parameter("image_width").as_double();
    image_height_ = get_parameter("image_height").as_double();
    class_rules_ = parse_rules(get_parameter("class_rules").as_string_array());
    enable_transforms_ = get_parameter("enable_transforms").as_bool();
    tf_robot_camera_ = make_transform_from_xyz_rpy(get_parameter("T_robot_camera_xyz_rpy").as_double_array());
    tf_arm1_robot_ = make_transform_from_xyz_rpy(get_parameter("T_arm1_robot_xyz_rpy").as_double_array());
    tf_arm2_robot_ = make_transform_from_xyz_rpy(get_parameter("T_arm2_robot_xyz_rpy").as_double_array());
    watchdog_timeout_sec_ = get_parameter("watchdog_timeout_sec").as_double();
    fatal_no_udp_timeout_sec_ = get_parameter("fatal_no_udp_timeout_sec").as_double();
    request_timeout_sec_ = get_parameter("request_timeout_sec").as_double();
    default_max_frame_age_sec_ = get_parameter("default_max_frame_age_sec").as_double();
  }

  void create_publishers_and_subscribers() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_));
    reliable_qos_ ? qos.reliable() : qos.best_effort();
    qos.durability_volatile();
    frame_pub_ = create_publisher<techx_vision_bridge::msg::VisionFrame>(get_parameter("frame_topic_name").as_string(), qos);
    object_pub_ = create_publisher<techx_vision_bridge::msg::VisionObject>(get_parameter("object_topic_name").as_string(), qos);
    detail_pub_ = create_publisher<techx_vision_bridge::msg::VisionTarget>(get_parameter("detail_topic_name").as_string(), qos);
    legacy_pub_ = create_publisher<techx_vision_bridge::msg::Target3D>(get_parameter("topic_name").as_string(), qos);
    if (enable_request_selector_) {
      selected_pub_ = create_publisher<techx_vision_bridge::msg::VisionSelection>(get_parameter("selected_topic_name").as_string(), qos);
      request_sub_ = create_subscription<techx_vision_bridge::msg::VisionRequest>(
          get_parameter("request_topic_name").as_string(), qos,
          std::bind(&VisionFrameBridgeNode::on_request, this, std::placeholders::_1));
      const int period_ms = std::max(10, static_cast<int>(get_parameter("publish_period_ms").as_int()));
      selector_timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&VisionFrameBridgeNode::publish_selection, this));
    }
  }

  void open_socket() {
    if (sock_ >= 0) {
      close(sock_);
      sock_ = -1;
    }
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      RCLCPP_ERROR(get_logger(), "socket failed: %s", std::strerror(errno));
      return;
    }
    int opt = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int flags = fcntl(sock_, F_GETFL, 0);
    if (flags >= 0) fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(udp_port_));
    if (inet_pton(AF_INET, bind_addr_.c_str(), &addr.sin_addr) != 1) {
      RCLCPP_ERROR(get_logger(), "bad bind address: %s", bind_addr_.c_str());
      close(sock_);
      sock_ = -1;
      return;
    }
    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind failed: %s", std::strerror(errno));
      close(sock_);
      sock_ = -1;
    }
  }

  void recv_once() {
    if (sock_ < 0) {
      open_socket();
      return;
    }
    uint8_t buf[4096];
    DecodedFrame best;
    bool has_best = false;
    while (true) {
      ssize_t n = recvfrom(sock_, buf, sizeof(buf), 0, nullptr, nullptr);
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "recvfrom failed: %s", std::strerror(errno));
        }
        break;
      }
      DecodedFrame frame;
      if (decode(buf, static_cast<size_t>(n), frame) && prefer(frame, best, has_best)) {
        best = std::move(frame);
        has_best = true;
      } else {
        ++decode_failures_;
      }
    }
    if (has_best) {
      if (!first_frame_received_) {
        first_frame_received_ = true;
        RCLCPP_INFO(get_logger(), "first UDP frame received: seq=%u targets=%zu protocol=V%d",
                    best.seq, best.targets.size(), best.protocol_version);
      } else if (udp_lost_marker_) {
        udp_lost_marker_ = false;
        RCLCPP_INFO(get_logger(), "UDP resumed: seq=%u targets=%zu",
                    best.seq, best.targets.size());
      }
      ++frames_received_;
      targets_received_ += best.targets.size();
      last_valid_ = std::chrono::steady_clock::now();
      publish(best);
    }
  }

  bool prefer(const DecodedFrame &candidate, const DecodedFrame &current, bool has_current) const {
    if (!has_current) return true;
    if (candidate.protocol_version == 2 && current.protocol_version != 2) return true;
    if (candidate.protocol_version != 2 && current.protocol_version == 2) return false;
    return static_cast<int32_t>(candidate.seq - current.seq) > 0;
  }

  bool accept_seq(uint32_t seq, uint8_t version) {
    if (version == 2) {
      if (!v2_seq_init_ || static_cast<int32_t>(seq - last_v2_seq_) > 0) {
        last_v2_seq_ = seq;
        v2_seq_init_ = true;
        v2_seen_ = true;
        return true;
      }
      return false;
    }
    if (!accept_legacy_ || v2_seen_) return false;
    if (!legacy_seq_init_ || static_cast<int32_t>(seq - last_legacy_seq_) > 0) {
      last_legacy_seq_ = seq;
      legacy_seq_init_ = true;
      return true;
    }
    return false;
  }

  bool decode(const uint8_t *data, size_t len, DecodedFrame &out) {
    if (len < sizeof(uint16_t)) return false;
    uint16_t magic = 0;
    std::memcpy(&magic, data, sizeof(magic));
    if (magic == MAGIC_V2) return decode_v2(data, len, out);
    if (magic == MAGIC_LEGACY) return decode_legacy(data, len, out);
    return false;
  }

  bool decode_legacy(const uint8_t *data, size_t len, DecodedFrame &out) {
    if (!accept_legacy_ || len != sizeof(LegacyPacket)) return false;
    LegacyPacket pkt{};
    std::memcpy(&pkt, data, sizeof(pkt));
    if (crc16_ccitt(data, 27) != pkt.crc16) return false;
    if (!accept_seq(pkt.seq, 1)) return false;
    out = DecodedFrame{};
    out.protocol_version = 1;
    out.seq = pkt.seq;
    out.timestamp = pkt.timestamp;
    DecodedTarget t{};
    t.track_id = pkt.track_id;
    t.confidence = 1.0f;
    t.x = pkt.x;
    t.y = pkt.y;
    t.z = pkt.z;
    t.valid_xyz = pkt.z > 0.0f && finite3(pkt.x, pkt.y, pkt.z);
    enrich(t);
    out.targets.push_back(t);
    return true;
  }

  bool decode_v2(const uint8_t *data, size_t len, DecodedFrame &out) {
    if (len < sizeof(V2Header) + sizeof(uint16_t)) return false;
    V2Header h{};
    std::memcpy(&h, data, sizeof(h));
    if (h.version != VERSION_V2 || h.count > MAX_TARGETS) return false;
    const size_t expected = sizeof(V2Header) + static_cast<size_t>(h.count) * sizeof(V2Target) + sizeof(uint16_t);
    if (len != expected) return false;
    uint16_t rx_crc = 0;
    std::memcpy(&rx_crc, data + len - sizeof(uint16_t), sizeof(rx_crc));
    if (crc16_ccitt(data, len - sizeof(uint16_t)) != rx_crc) return false;
    if (!accept_seq(h.seq, 2)) return false;
    out = DecodedFrame{};
    out.protocol_version = 2;
    out.seq = h.seq;
    out.timestamp = h.timestamp;
    out.targets.reserve(h.count);
    size_t off = sizeof(V2Header);
    for (uint8_t i = 0; i < h.count; ++i) {
      V2Target raw{};
      std::memcpy(&raw, data + off, sizeof(raw));
      off += sizeof(raw);
      DecodedTarget t{};
      t.track_id = raw.track_id;
      t.class_id = raw.class_id;
      t.color = raw.color;
      t.confidence = raw.confidence;
      t.u = raw.u;
      t.v = raw.v;
      t.x = raw.x;
      t.y = raw.y;
      t.z = raw.z;
      t.valid_xyz = raw.z > 0.0f && finite3(raw.x, raw.y, raw.z);
      enrich(t);
      out.targets.push_back(t);
    }
    return true;
  }

  void apply_class_rule(DecodedTarget &t) const {
    const int cid = static_cast<int>(t.class_id);
    for (const auto &rule : class_rules_) {
      if (cid >= rule.min_id && cid <= rule.max_id) {
        t.zone_id = rule.zone_id;
        t.target_type = rule.target_type;
        t.control_frame = rule.control_frame;
        t.priority_bias = rule.priority_bias;
        return;
      }
    }
    t.zone_id = ZONE_UNKNOWN;
    t.target_type = TYPE_UNKNOWN;
    t.control_frame = FRAME_CAMERA_LINK;
  }

  void enrich(DecodedTarget &t) {
    apply_class_rule(t);
    if (image_width_ > 1.0 && image_height_ > 1.0 && std::isfinite(t.u) && std::isfinite(t.v)) {
      const double cx = image_width_ * 0.5;
      const double cy = image_height_ * 0.5;
      t.align_err_x = static_cast<float>((static_cast<double>(t.u) - cx) / cx);
      t.align_err_y = static_cast<float>((static_cast<double>(t.v) - cy) / cy);
    }
    if (enable_transforms_ && t.valid_xyz) {
      const Vec3 camera{t.x, t.y, t.z};
      const Vec3 robot = apply_transform(tf_robot_camera_, camera);
      t.valid_robot_xyz = true;
      t.robot_x = static_cast<float>(robot.x);
      t.robot_y = static_cast<float>(robot.y);
      t.robot_z = static_cast<float>(robot.z);
      const Vec3 arm1 = apply_transform(tf_arm1_robot_, robot);
      t.valid_arm1_xyz = true;
      t.arm1_x = static_cast<float>(arm1.x);
      t.arm1_y = static_cast<float>(arm1.y);
      t.arm1_z = static_cast<float>(arm1.z);
      const Vec3 arm2 = apply_transform(tf_arm2_robot_, robot);
      t.valid_arm2_xyz = true;
      t.arm2_x = static_cast<float>(arm2.x);
      t.arm2_y = static_cast<float>(arm2.y);
      t.arm2_z = static_cast<float>(arm2.z);
    }
    copy_control_coordinates(t);
    t.priority = t.confidence + t.priority_bias - 0.2f * (std::abs(t.align_err_x) + std::abs(t.align_err_y));
  }

  void copy_control_coordinates(DecodedTarget &t) {
    switch (t.control_frame) {
      case FRAME_ROBOT_BASE:
        t.valid_control_xyz = t.valid_robot_xyz;
        t.control_x = t.robot_x;
        t.control_y = t.robot_y;
        t.control_z = t.robot_z;
        break;
      case FRAME_ARM1_BASE:
        t.valid_control_xyz = t.valid_arm1_xyz;
        t.control_x = t.arm1_x;
        t.control_y = t.arm1_y;
        t.control_z = t.arm1_z;
        break;
      case FRAME_ARM2_BASE:
        t.valid_control_xyz = t.valid_arm2_xyz;
        t.control_x = t.arm2_x;
        t.control_y = t.arm2_y;
        t.control_z = t.arm2_z;
        break;
      case FRAME_CAMERA_LINK:
      default:
        t.valid_control_xyz = t.valid_xyz;
        t.control_x = t.x;
        t.control_y = t.y;
        t.control_z = t.z;
        break;
    }
  }

  rclcpp::Time stamp(const DecodedFrame &frame) const {
    if (frame.timestamp > 0.0) {
      const auto sec = static_cast<int64_t>(frame.timestamp);
      const auto nsec = static_cast<uint32_t>((frame.timestamp - static_cast<double>(sec)) * 1e9);
      return rclcpp::Time(sec, nsec, RCL_ROS_TIME);
    }
    return now();
  }

  techx_vision_bridge::msg::VisionObject to_object(const DecodedFrame &frame, const DecodedTarget &t, uint8_t index) {
    techx_vision_bridge::msg::VisionObject msg;
    msg.header.stamp = stamp(frame);
    msg.header.frame_id = "camera_link";
    msg.seq = frame.seq;
    msg.target_index = index;
    msg.target_count = static_cast<uint8_t>(frame.targets.size());
    msg.zone_id = t.zone_id;
    msg.target_type = t.target_type;
    msg.class_id = t.class_id;
    msg.color = t.color;
    msg.confidence = t.confidence;
    msg.u = t.u;
    msg.v = t.v;
    msg.valid_xyz = t.valid_xyz;
    msg.x = t.valid_xyz ? t.x : 0.0f;
    msg.y = t.valid_xyz ? t.y : 0.0f;
    msg.z = t.valid_xyz ? t.z : 0.0f;
    msg.valid_robot_xyz = t.valid_robot_xyz;
    msg.robot_x = t.valid_robot_xyz ? t.robot_x : 0.0f;
    msg.robot_y = t.valid_robot_xyz ? t.robot_y : 0.0f;
    msg.robot_z = t.valid_robot_xyz ? t.robot_z : 0.0f;
    msg.valid_arm1_xyz = t.valid_arm1_xyz;
    msg.arm1_x = t.valid_arm1_xyz ? t.arm1_x : 0.0f;
    msg.arm1_y = t.valid_arm1_xyz ? t.arm1_y : 0.0f;
    msg.arm1_z = t.valid_arm1_xyz ? t.arm1_z : 0.0f;
    msg.valid_arm2_xyz = t.valid_arm2_xyz;
    msg.arm2_x = t.valid_arm2_xyz ? t.arm2_x : 0.0f;
    msg.arm2_y = t.valid_arm2_xyz ? t.arm2_y : 0.0f;
    msg.arm2_z = t.valid_arm2_xyz ? t.arm2_z : 0.0f;
    msg.control_frame = t.control_frame;
    msg.valid_control_xyz = t.valid_control_xyz;
    msg.control_x = t.valid_control_xyz ? t.control_x : 0.0f;
    msg.control_y = t.valid_control_xyz ? t.control_y : 0.0f;
    msg.control_z = t.valid_control_xyz ? t.control_z : 0.0f;
    msg.align_err_x = t.align_err_x;
    msg.align_err_y = t.align_err_y;
    msg.priority = t.priority;
    return msg;
  }

  void publish(const DecodedFrame &frame) {
    techx_vision_bridge::msg::VisionFrame frame_msg;
    frame_msg.header.stamp = stamp(frame);
    frame_msg.header.frame_id = "camera_link";
    frame_msg.seq = frame.seq;
    frame_msg.protocol_version = frame.protocol_version;
    frame_msg.upstream_timestamp = frame.timestamp;
    frame_msg.target_count = static_cast<uint8_t>(frame.targets.size());
    frame_msg.has_target = !frame.targets.empty();
    frame_msg.targets.reserve(frame.targets.size());
    for (size_t i = 0; i < frame.targets.size(); ++i) {
      auto obj = to_object(frame, frame.targets[i], static_cast<uint8_t>(i));
      frame_msg.targets.push_back(obj);
      if (publish_object_) object_pub_->publish(obj);
      if (publish_detail_) publish_detail(frame, &frame.targets[i]);
      if (publish_legacy_ && frame.targets[i].valid_xyz) publish_xyz(frame, frame.targets[i]);
    }
    if (frame.targets.empty() && publish_detail_) publish_detail(frame, nullptr);
    latest_frame_ = frame_msg;
    latest_frame_time_ = std::chrono::steady_clock::now();
    has_frame_ = true;
    if (publish_frame_) frame_pub_->publish(frame_msg);
    publish_selection();
  }

  void publish_detail(const DecodedFrame &frame, const DecodedTarget *t) {
    techx_vision_bridge::msg::VisionTarget msg;
    msg.header.stamp = stamp(frame);
    msg.header.frame_id = "camera_link";
    msg.seq = frame.seq;
    msg.protocol_version = frame.protocol_version;
    msg.target_count = static_cast<uint8_t>(frame.targets.size());
    msg.has_target = t != nullptr;
    msg.valid_xyz = t && t->valid_xyz;
    if (t) {
      msg.track_id = t->track_id;
      msg.class_id = t->class_id;
      msg.color = t->color;
      msg.confidence = t->confidence;
      msg.u = t->u;
      msg.v = t->v;
      msg.x = t->valid_xyz ? t->x : 0.0f;
      msg.y = t->valid_xyz ? t->y : 0.0f;
      msg.z = t->valid_xyz ? t->z : 0.0f;
    }
    detail_pub_->publish(msg);
  }

  void publish_xyz(const DecodedFrame &frame, const DecodedTarget &t) {
    techx_vision_bridge::msg::Target3D msg;
    msg.header.stamp = stamp(frame);
    msg.header.frame_id = "camera_link_" + std::to_string(t.track_id);
    msg.track_id = t.track_id;
    msg.x = t.x;
    msg.y = t.y;
    msg.z = t.z;
    legacy_pub_->publish(msg);
  }

  void on_request(const techx_vision_bridge::msg::VisionRequest::SharedPtr msg) {
    latest_request_ = *msg;
    request_recv_time_ = std::chrono::steady_clock::now();
    has_request_ = true;
    RCLCPP_INFO(get_logger(), "request received: seq=%d class_id=%d target_type=%d zone_id=%d require_xyz=%s",
                msg->request_seq, msg->class_id, msg->target_type, msg->zone_id,
                msg->require_control_xyz ? "true" : "false");
    publish_selection();
  }

  bool request_expired() const {
    if (!has_request_ || request_timeout_sec_ <= 0.0) return false;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - request_recv_time_).count() > request_timeout_sec_;
  }

  float frame_age_sec() const {
    if (!has_frame_) return 0.0f;
    return static_cast<float>(std::chrono::duration<double>(std::chrono::steady_clock::now() - latest_frame_time_).count());
  }

  bool matches_request(const techx_vision_bridge::msg::VisionObject &obj) const {
    if (!has_request_) return false;
    if (latest_request_.target_type != techx_vision_bridge::msg::VisionRequest::TYPE_ANY && obj.target_type != latest_request_.target_type) return false;
    if (latest_request_.zone_id != techx_vision_bridge::msg::VisionRequest::ZONE_ANY && obj.zone_id != latest_request_.zone_id) return false;
    if (latest_request_.use_class_id && obj.class_id != latest_request_.class_id) return false;
    if (latest_request_.use_color && obj.color != latest_request_.color) return false;
    if (latest_request_.require_control_xyz && !obj.valid_control_xyz) return false;
    if (latest_request_.min_confidence > 0.0f && obj.confidence < latest_request_.min_confidence) return false;
    return true;
  }

  bool select_best(size_t &best_index, float &best_score) const {
    best_index = 0;
    best_score = -std::numeric_limits<float>::infinity();
    bool found = false;
    for (size_t i = 0; i < latest_frame_.targets.size(); ++i) {
      const auto &obj = latest_frame_.targets[i];
      if (!matches_request(obj)) continue;
      float score = finite_or_zero(obj.priority);
      if (obj.valid_control_xyz) score += 0.02f;
      if (score > best_score) {
        best_index = i;
        best_score = score;
        found = true;
      }
    }
    return found;
  }

  techx_vision_bridge::msg::VisionSelection base_selection(uint8_t status) {
    techx_vision_bridge::msg::VisionSelection out;
    out.header.stamp = now();
    out.header.frame_id = "camera_link";
    out.frame_seq = has_frame_ ? latest_frame_.seq : 0;
    out.request_seq = has_request_ ? latest_request_.request_seq : 0;
    out.has_request = has_request_;
    out.has_match = false;
    out.status = status;
    out.selected_index = INVALID_INDEX;
    out.frame_age_sec = frame_age_sec();
    out.score = 0.0f;
    return out;
  }

  void publish_selection() {
    if (!enable_request_selector_ || !selected_pub_) return;
    if (!has_request_) return;
    uint8_t status = techx_vision_bridge::msg::VisionSelection::STATUS_NO_REQUEST;
    if (request_expired()) {
      status = techx_vision_bridge::msg::VisionSelection::STATUS_REQUEST_STALE;
      selected_pub_->publish(base_selection(status));
      has_request_ = false;
    } else if (!has_frame_) {
      status = techx_vision_bridge::msg::VisionSelection::STATUS_NO_FRAME;
      selected_pub_->publish(base_selection(status));
    } else {
      const float max_age = latest_request_.max_frame_age_sec > 0.0f ? latest_request_.max_frame_age_sec : static_cast<float>(default_max_frame_age_sec_);
      if (max_age > 0.0f && frame_age_sec() > max_age) {
        status = techx_vision_bridge::msg::VisionSelection::STATUS_FRAME_STALE;
        selected_pub_->publish(base_selection(status));
      } else {
        size_t best_index = 0;
        float best_score = 0.0f;
        if (!select_best(best_index, best_score)) {
          status = techx_vision_bridge::msg::VisionSelection::STATUS_NO_MATCH;
          selected_pub_->publish(base_selection(status));
        } else {
          status = techx_vision_bridge::msg::VisionSelection::STATUS_OK;
          auto out = base_selection(status);
          out.has_match = true;
          out.selected_index = static_cast<uint8_t>(std::min<size_t>(best_index, INVALID_INDEX - 1));
          out.score = best_score;
          out.target = latest_frame_.targets[best_index];
          selected_pub_->publish(out);
        }
      }
    }
    if (status != last_selection_status_) {
      RCLCPP_INFO(get_logger(), "selector status %d -> %d (frame_seq=%d frame_age=%.3fs)",
                  last_selection_status_, status,
                  has_frame_ ? static_cast<int>(latest_frame_.seq) : 0,
                  frame_age_sec());
      last_selection_status_ = status;
    }
  }

  void watchdog() {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_valid_).count();
    if (elapsed > watchdog_timeout_sec_) {
      udp_lost_marker_ = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "vision UDP signal lost %.2fs", elapsed);
    }
    if (fatal_no_udp_timeout_sec_ > 0.0 && elapsed > fatal_no_udp_timeout_sec_) {
      RCLCPP_FATAL(get_logger(), "no valid Jetson UDP data for %.1fs; shutting down vision_bridge_node", elapsed);
      if (sock_ >= 0) {
        close(sock_);
        sock_ = -1;
      }
      rclcpp::shutdown();
    }
  }

  void log_stats() {
    const double interval = std::chrono::duration<double>(std::chrono::steady_clock::now() - stats_reset_time_).count();
    if (interval <= 0.0) return;
    const double fps = static_cast<double>(frames_received_) / interval;
    const double tps = static_cast<double>(targets_received_) / interval;
    const double dfps = static_cast<double>(decode_failures_) / interval;
    RCLCPP_INFO(get_logger(), "udp stats: %.1f fps %.1f tps %.1f decode_fail/s (over %.1fs)",
                fps, tps, dfps, interval);
    frames_received_ = 0;
    targets_received_ = 0;
    decode_failures_ = 0;
    stats_reset_time_ = std::chrono::steady_clock::now();
  }

  int sock_{-1};
  std::string bind_addr_;
  int udp_port_{12345};
  bool publish_frame_{true};
  bool publish_object_{true};
  bool publish_detail_{false};
  bool publish_legacy_{false};
  bool accept_legacy_{false};
  bool enable_request_selector_{true};
  bool reliable_qos_{true};
  int qos_depth_{5};
  double image_width_{640.0};
  double image_height_{480.0};
  std::vector<ClassRule> class_rules_{default_rules()};
  bool enable_transforms_{true};
  Transform tf_robot_camera_{};
  Transform tf_arm1_robot_{};
  Transform tf_arm2_robot_{};
  double watchdog_timeout_sec_{0.3};
  double fatal_no_udp_timeout_sec_{600.0};
  double request_timeout_sec_{0.0};
  double default_max_frame_age_sec_{0.20};

  bool v2_seq_init_{false};
  bool legacy_seq_init_{false};
  bool v2_seen_{false};
  uint32_t last_v2_seq_{0};
  uint32_t last_legacy_seq_{0};
  std::chrono::steady_clock::time_point last_valid_;
  std::chrono::steady_clock::time_point latest_frame_time_;
  std::chrono::steady_clock::time_point request_recv_time_;

  bool has_frame_{false};
  bool has_request_{false};
  techx_vision_bridge::msg::VisionFrame latest_frame_;
  techx_vision_bridge::msg::VisionRequest latest_request_;

  // stats for syslog
  bool first_frame_received_{false};
  uint64_t frames_received_{0};
  uint64_t decode_failures_{0};
  uint64_t targets_received_{0};
  uint8_t last_selection_status_{255};  // 255 = never published
  bool udp_lost_marker_{false};
  std::chrono::steady_clock::time_point stats_reset_time_;

  rclcpp::Publisher<techx_vision_bridge::msg::VisionFrame>::SharedPtr frame_pub_;
  rclcpp::Publisher<techx_vision_bridge::msg::VisionObject>::SharedPtr object_pub_;
  rclcpp::Publisher<techx_vision_bridge::msg::VisionTarget>::SharedPtr detail_pub_;
  rclcpp::Publisher<techx_vision_bridge::msg::Target3D>::SharedPtr legacy_pub_;
  rclcpp::Publisher<techx_vision_bridge::msg::VisionSelection>::SharedPtr selected_pub_;
  rclcpp::Subscription<techx_vision_bridge::msg::VisionRequest>::SharedPtr request_sub_;
  rclcpp::TimerBase::SharedPtr recv_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr selector_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

// ── syslog dual-output handler ──────────────────────────────────────────
static rcutils_logging_output_handler_t g_original_handler = nullptr;

static void dual_output_handler(
    const rcutils_log_location_t *location,
    int severity, const char *name,
    rcutils_time_point_value_t timestamp,
    const char *format, va_list *args) {
  // Forward to original ROS2 handler (console + rosout + file)
  if (g_original_handler) {
    g_original_handler(location, severity, name, timestamp, format, args);
  }
  // Also write to syslog
  int prio;
  switch (severity) {
    case RCUTILS_LOG_SEVERITY_DEBUG: prio = LOG_DEBUG;   break;
    case RCUTILS_LOG_SEVERITY_INFO:  prio = LOG_INFO;    break;
    case RCUTILS_LOG_SEVERITY_WARN:  prio = LOG_WARNING; break;
    case RCUTILS_LOG_SEVERITY_ERROR: prio = LOG_ERR;     break;
    case RCUTILS_LOG_SEVERITY_FATAL: prio = LOG_CRIT;    break;
    default:                         prio = LOG_INFO;
  }
  va_list args_copy;
  va_copy(args_copy, *args);
  vsyslog(prio, format, args_copy);
  va_end(args_copy);
}
// ────────────────────────────────────────────────────────────────────────

int main(int argc, char **argv) {
  openlog("vision_bridge", LOG_PID | LOG_CONS, LOG_USER);
  rclcpp::init(argc, argv);
  g_original_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(dual_output_handler);
  rclcpp::spin(std::make_shared<VisionFrameBridgeNode>());
  rclcpp::shutdown();
  rcutils_logging_set_output_handler(g_original_handler);
  closelog();
  return 0;
}
