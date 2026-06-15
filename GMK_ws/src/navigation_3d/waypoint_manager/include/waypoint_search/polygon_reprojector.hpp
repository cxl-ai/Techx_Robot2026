#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <optional>
#include <cmath>
#include <limits>

// 多边形障碍物检测与航点外推
// Linus 风格：数据结构简单直接，逻辑用静态函数实现
// 参考 far_planner 的凹凸性分析 + 墙体过滤

namespace polygon_reprojector {

// ============================================================================
// 数据结构
// ============================================================================

// 顶点凹凸性（用于判断是否适合外推）
enum class VertexConvexity {
  CONVEX,    // 凸点：可用于外推
  CONCAVE,   // 凹点：不适合外推
  UNKNOWN    // 未知/柱状
};

// 多边形顶点（带凹凸性信息）
struct PolyVertex {
  cv::Point2f position;
  VertexConvexity convexity{VertexConvexity::UNKNOWN};
  cv::Point2f free_dir{0.0f, 0.0f};  // 外推方向（仅凸点有效，指向多边形内部）
};

// 多边形结构（可以是凹多边形）
struct ConvexPolygon {
  std::vector<PolyVertex> vertices;
  bool is_robot_inside{false};  // 机器人是否在此多边形内
};

// 多边形构建参数
struct PolygonBuildParams {
  double grid_range{5.0};              // 局部网格半径 (m)
  double grid_resolution{0.05};        // 栅格分辨率 (m)
  int inflate_cells{1};                // 栅格膨胀步数
  double min_area{0.15};               // 最小多边形面积 (m²)
  double intensity_thred{0.6};         // 强度阈值（高于此值参与障碍物多边形生成）
  double height_lower{0.5};            // Z轴下方过滤范围 (m)
  double height_upper{0.5};            // Z轴上方过滤范围 (m)
  double wall_angle_thred{8.0};        // 墙体角度阈值（度，推荐5-10）
};

// 外推参数
struct ReprojectionParams {
  double free_radius{0.1};  // 外推距离 (m)
};

// 外推缓存（用于迭代外推）
struct ReprojectionCache {
  std::optional<size_t> waypoint_index;
  std::optional<cv::Point2f> position;
  
  void reset() {
    waypoint_index.reset();
    position.reset();
  }
  
  bool shouldReset(size_t current_index) const {
    return !waypoint_index || *waypoint_index != current_index;
  }
};

// ============================================================================
// 几何工具函数
// ============================================================================

// 2D 向量归一化
static inline cv::Point2f normalize2D(const cv::Point2f &v) {
  const float norm = std::hypotf(v.x, v.y);
  if (norm < 1e-6f) {
    return cv::Point2f(0.0f, 0.0f);
  }
  return cv::Point2f(v.x / norm, v.y / norm);
}

// 判断点是否在多边形内（射线法）
static inline bool pointInsidePoly(const std::vector<cv::Point2f> &poly, const cv::Point2f &p) {
  const int npol = static_cast<int>(poly.size());
  if (npol < 3) {
    return false;
  }
  int c = 0;
  for (int i = 0, j = npol - 1; i < npol; j = i++) {
    const cv::Point2f v1 = poly[i];
    const cv::Point2f v2 = poly[j];
    if ((((v1.y <= p.y) && (p.y < v2.y)) ||
         ((v2.y <= p.y) && (p.y < v1.y))) &&
        (p.x < (v2.x - v1.x) * (p.y - v1.y) / (v2.y - v1.y) + v1.x)) {
      c = !c;
    }
  }
  return c;
}

// 判断两线段是否相交并返回交点
static inline bool segmentIntersectsSegment(const cv::Point2f &a, const cv::Point2f &b,
                                            const cv::Point2f &c, const cv::Point2f &d,
                                            cv::Point2f &intersection) {
  const cv::Point2f ab(b.x - a.x, b.y - a.y);
  const cv::Point2f cd(d.x - c.x, d.y - c.y);
  const cv::Point2f ac(c.x - a.x, c.y - a.y);
  
  const float denom = ab.x * cd.y - ab.y * cd.x;
  if (std::abs(denom) < 1e-9f) {
    return false;  // 平行或重合
  }
  
  const float t = (ac.x * cd.y - ac.y * cd.x) / denom;
  const float u = (ac.x * ab.y - ac.y * ab.x) / denom;
  
  if (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f) {
    intersection.x = a.x + t * ab.x;
    intersection.y = a.y + t * ab.y;
    return true;
  }
  return false;
}

// 找线段与多边形边界的最近交点（用于外推）
static inline bool findSegmentPolygonIntersection(const cv::Point2f &seg_start,
                                                  const cv::Point2f &seg_end,
                                                  const std::vector<cv::Point2f> &poly,
                                                  cv::Point2f &nearest_intersection) {
  const int n = static_cast<int>(poly.size());
  if (n < 3) {
    return false;
  }
  
  bool found = false;
  float min_dist = std::numeric_limits<float>::infinity();
  
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    cv::Point2f intersection;
    if (segmentIntersectsSegment(seg_start, seg_end, poly[i], poly[j], intersection)) {
      const float dist = std::hypotf(intersection.x - seg_start.x, 
                                    intersection.y - seg_start.y);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_intersection = intersection;
        found = true;
      }
    }
  }
  return found;
}

// 判断三点是否接近共线（墙体检测）
static inline bool isPrevWallVertex(const cv::Point2f &first_p,
                                   const cv::Point2f &mid_p,
                                   const cv::Point2f &add_p,
                                   const double align_angle_cos) {
  cv::Point2f diff_p1 = first_p - mid_p;
  cv::Point2f diff_p2 = add_p - mid_p;
  diff_p1 = normalize2D(diff_p1);
  diff_p2 = normalize2D(diff_p2);
  
  // 如果两个向量的点积绝对值 > 阈值，说明接近共线（墙体）
  if (std::abs(diff_p1.dot(diff_p2)) > align_angle_cos) {
    return true;
  }
  return false;
}

// 递归移除墙体连接（移除接近直线的中间点）
static inline void removeWallConnection(std::vector<cv::Point2f> &contour,
                                       const cv::Point2f &add_p,
                                       std::size_t &refined_idx,
                                       const double align_angle_cos) {
  if (refined_idx < 2) return;
  
  if (!isPrevWallVertex(contour[refined_idx - 2], contour[refined_idx - 1], 
                        add_p, align_angle_cos)) {
    return;
  } else {
    // 移除中间点（墙体连接）
    --refined_idx;
    removeWallConnection(contour, add_p, refined_idx, align_angle_cos);
  }
}

// 对轮廓应用墙体过滤
static inline void applyWallFilter(std::vector<cv::Point2f> &contour_world,
                                  const double align_angle_cos,
                                  const double dist_limit) {
  if (contour_world.size() < 3) return;
  
  const std::size_t c_size = contour_world.size();
  std::vector<cv::Point2f> filtered;
  filtered.reserve(c_size);
  std::size_t refined_idx = 0;
  
  for (std::size_t j = 0; j < c_size; ++j) {
    cv::Point2f p = contour_world[j];
    
    // 如果与前一个点距离 > 阈值，保留
    if (refined_idx < 1 || 
        std::hypotf(filtered[refined_idx - 1].x - p.x, 
                   filtered[refined_idx - 1].y - p.y) > dist_limit) {
      // 移除墙体连接
      removeWallConnection(filtered, p, refined_idx, align_angle_cos);
      if (refined_idx < filtered.size()) {
        filtered[refined_idx] = p;
      } else {
        filtered.push_back(p);
      }
      ++refined_idx;
    }
  }
  
  // 最后对首尾点再检查一次墙体连接
  if (refined_idx > 0) {
    removeWallConnection(filtered, filtered[0], refined_idx, align_angle_cos);
  }
  
  filtered.resize(refined_idx);
  
  // 如果首尾点距离过近，移除尾点
  if (filtered.size() > 1 && 
      std::hypotf(filtered.front().x - filtered.back().x,
                 filtered.front().y - filtered.back().y) < dist_limit) {
    filtered.pop_back();
  }
  
  contour_world = std::move(filtered);
}

// 检查点云是否有指定字段
static inline bool hasField(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name) {
  for (const auto &field : cloud.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

// 从 ConvexPolygon 提取顶点位置列表
static inline std::vector<cv::Point2f> extractPolyPositions(const ConvexPolygon &cpoly) {
  std::vector<cv::Point2f> positions;
  positions.reserve(cpoly.vertices.size());
  for (const auto &v : cpoly.vertices) {
    positions.push_back(v.position);
  }
  return positions;
}

// ============================================================================
// 核心功能函数
// ============================================================================

// 构建局部障碍物多边形（参考 far_planner 思路）
// 特性：保持原始形状（凹/凸）+ 顶点凹凸性分析 + 墙体过滤
static inline std::vector<ConvexPolygon> buildConvexPolygonsFromTerrain(
    const geometry_msgs::msg::Point &center,
    const sensor_msgs::msg::PointCloud2 &cloud,
    const PolygonBuildParams &params,
    bool &warned_no_intensity) {
  
  std::vector<ConvexPolygon> polys_result;
  
  if (params.grid_range <= 0.0 || params.grid_resolution <= 0.0) {
    return polys_result;
  }

  const int grid_size = static_cast<int>(std::ceil(params.grid_range * 2.0 / params.grid_resolution));
  const int mat_size = (grid_size % 2 == 0) ? grid_size + 1 : grid_size;
  const int center_idx = mat_size / 2;

  cv::Mat img = cv::Mat::zeros(mat_size, mat_size, CV_8UC1);

  const bool has_intensity = hasField(cloud, "intensity");
  if (!has_intensity && !warned_no_intensity) {
    warned_no_intensity = true;
    // 注意：这里无法直接输出日志，调用者需要处理警告
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  std::optional<sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity;
  if (has_intensity) {
    iter_intensity.emplace(cloud, "intensity");
  }

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      if (has_intensity) {
        ++(*iter_intensity);
      }
      continue;
    }

    if (has_intensity) {
      const float intensity = *(*iter_intensity);
      ++(*iter_intensity);
      // 障碍物多边形：仅强度高于阈值的点参与（高强度=障碍物）
      if (intensity < params.intensity_thred) {
        continue;
      }
    }

    // Z 轴高度过滤（上下分别设置范围）
    const double dz = static_cast<double>(*iter_z) - center.z;
    if (dz < -params.height_lower || dz > params.height_upper) {
      continue;  // 过滤低于下限或高于上限的点
    }

    const double dx = static_cast<double>(*iter_x) - center.x;
    const double dy = static_cast<double>(*iter_y) - center.y;
    if (std::abs(dx) > params.grid_range || std::abs(dy) > params.grid_range) {
      continue;
    }

    const int row = center_idx + static_cast<int>(std::lround(dx / params.grid_resolution));
    const int col = center_idx + static_cast<int>(std::lround(dy / params.grid_resolution));
    if (row < 0 || row >= mat_size || col < 0 || col >= mat_size) {
      continue;
    }
    img.at<unsigned char>(row, col) = 255;
  }

  if (params.inflate_cells > 0) {
    const int k = params.inflate_cells * 2 + 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
    cv::dilate(img, img, kernel);
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 获取机器人位置用于判断 is_robot_inside
  const cv::Point2f robot2(static_cast<float>(center.x), static_cast<float>(center.y));

  polys_result.reserve(contours.size());

  for (const auto &contour : contours) {
    if (contour.size() < 3) {
      continue;
    }
    // 多边形近似（Ramer-Douglas-Peucker 算法）
    std::vector<cv::Point> approx;
    const double epsilon = params.grid_resolution * 2.0;  // 近似精度
    cv::approxPolyDP(contour, approx, epsilon, true);
    if (approx.size() < 3) {
      continue;
    }
    
    // 不使用凸包，保持原始形状（可以是凹多边形）
    // 这样可以避免室内相邻障碍物合并成一个大凸多边形
    const double area_pix = std::abs(cv::contourArea(approx));
    const double area_m2 = area_pix * params.grid_resolution * params.grid_resolution;
    if (area_m2 < params.min_area) {
      continue;
    }

    // 转换到世界坐标
    std::vector<cv::Point2f> poly_world;
    poly_world.reserve(approx.size());
    for (const auto &p : approx) {
      const double wx = (static_cast<double>(p.y) - center_idx) * params.grid_resolution + center.x;
      const double wy = (static_cast<double>(p.x) - center_idx) * params.grid_resolution + center.y;
      poly_world.emplace_back(static_cast<float>(wx), static_cast<float>(wy));
    }

    // 墙体过滤（移除接近直线的轮廓段）
    const double align_angle_cos = std::cos(params.wall_angle_thred * M_PI / 180.0 / 2.0);
    const double dist_limit = params.grid_resolution * 2.0;  // 距离阈值
    applyWallFilter(poly_world, align_angle_cos, dist_limit);
    
    // 墙体过滤后可能点数减少，需要重新检查
    if (poly_world.size() < 3) {
      continue;
    }

    // 构建带凹凸性分析的多边形（可以是凹多边形）
    ConvexPolygon cpoly;  // 注：名称保留为 ConvexPolygon 但实际可以是凹多边形
    cpoly.is_robot_inside = pointInsidePoly(poly_world, robot2);
    cpoly.vertices.reserve(poly_world.size());

    const int n = static_cast<int>(poly_world.size());
    for (int i = 0; i < n; ++i) {
      PolyVertex pv;
      pv.position = poly_world[i];
      
      // 凹凸性分析（参考 far_planner）
      const cv::Point2f &v = poly_world[i];
      const cv::Point2f &prev = poly_world[(i - 1 + n) % n];
      const cv::Point2f &next = poly_world[(i + 1) % n];

      const cv::Point2f dir1 = normalize2D(prev - v);
      const cv::Point2f dir2 = normalize2D(next - v);
      const cv::Point2f dir = normalize2D(dir1 + dir2);
      
      if (std::hypotf(dir.x, dir.y) < 1e-6f) {
        pv.convexity = VertexConvexity::UNKNOWN;
      } else {
        // 测试点：沿 dir 方向移动一小段距离
        const cv::Point2f test_p = v + dir * static_cast<float>(params.grid_resolution);
        
        // 如果 test_p 在多边形内，说明 dir 指向内部，v 是凸点
        if (pointInsidePoly(poly_world, test_p)) {
          pv.convexity = VertexConvexity::CONVEX;
          pv.free_dir = dir;  // 外推时需要反向（减去 dir）
        } else {
          pv.convexity = VertexConvexity::CONCAVE;
        }
      }
      
      cpoly.vertices.push_back(pv);
    }

    polys_result.push_back(std::move(cpoly));
  }

  return polys_result;
}

// 航点外推：当航点在多边形内时，沿"航点→机器人"连线在边界外推
// 返回：是否进行了外推
static inline bool reprojectPointOutsidePolygons(
    geometry_msgs::msg::Point &point,
    const std::optional<geometry_msgs::msg::Point> &robot_pos_opt,
    const std::vector<ConvexPolygon> &polys,
    const ReprojectionParams &params) {
  
  // 需要机器人位置信息
  if (!robot_pos_opt) {
    return false;
  }
  
  if (polys.empty()) {
    return false;
  }

  const cv::Point2f robot_pos(robot_pos_opt->x, robot_pos_opt->y);
  const cv::Point2f waypoint_pos(static_cast<float>(point.x), static_cast<float>(point.y));

  for (const auto &cpoly : polys) {
    // 提取顶点位置用于点在多边形内判断
    const auto poly_positions = extractPolyPositions(cpoly);
    
    // 如果航点不在此多边形内，跳过
    if (!pointInsidePoly(poly_positions, waypoint_pos)) {
      continue;
    }
    
    // 如果机器人也在此多边形内，跳过（避免把机器人自己困住）
    if (cpoly.is_robot_inside) {
      continue;
    }

    // 找"航点→机器人"线段与多边形边界的交点
    cv::Point2f intersection;
    if (findSegmentPolygonIntersection(waypoint_pos, robot_pos, poly_positions, intersection)) {
      // 沿"航点→机器人"方向外推（从交点指向机器人）
      const cv::Point2f direction = normalize2D(robot_pos - waypoint_pos);
      const cv::Point2f new_p = intersection + direction * static_cast<float>(params.free_radius);
      point.x = new_p.x;
      point.y = new_p.y;
      return true;
    }
  }

  return false;
}

}  // namespace polygon_reprojector
