#pragma once

#include "waypoint_search_types.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <optional>

namespace fs = std::filesystem;

// CSV 文件加载器：简单的函数，不搞复杂的类
// Linus: "数据就是数据，函数就是函数"

namespace csv_loader {

// 在目录里找最新的 CSV 文件（按前缀匹配）
// 返回：找到的文件路径，或空
static inline std::optional<fs::path> findLatestCsvByPrefix(
    const fs::path &dir,
    const std::string &prefix) {
  
  if (!fs::exists(dir) || !fs::is_directory(dir)) {
    return std::nullopt;
  }

  bool found = false;
  fs::path best;
  fs::file_time_type best_time{};
  
  for (const auto &entry : fs::directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    
    const auto p = entry.path();
    if (p.extension() != ".csv") {
      continue;
    }
    
    const auto name = p.filename().string();
    if (name.rfind(prefix, 0) != 0) {  // 不是以 prefix 开头
      continue;
    }
    
    const auto t = entry.last_write_time();
    if (!found || t > best_time) {
      found = true;
      best_time = t;
      best = p;
    }
  }
  
  return found ? std::make_optional(best) : std::nullopt;
}

// 从 CSV 文件读取所有点
// 格式：floor, index, x, y, z, yaw
// 兼容有/无表头的情况
static inline std::vector<CsvPoint> readCsvPoints(
    const std::string &csv_path,
    rclcpp::Logger logger) {
  
  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(logger, "无法打开CSV文件: %s", csv_path.c_str());
    return {};
  }

  std::vector<CsvPoint> out;
  std::string line;

  // Lambda: 尝试解析一行 CSV
  auto try_parse = [&](const std::string &ln, CsvPoint &p) -> bool {
    std::stringstream ss(ln);
    std::string cell;
    std::vector<std::string> cols;
    
    while (std::getline(ss, cell, ',')) {
      cols.push_back(waypoint_helpers::trimCopy(cell));
    }
    
    if (cols.size() < 6) {
      return false;  // 列数不够
    }
    
    try {
      p.floor = std::stoi(cols[0]);
      p.index = std::stoi(cols[1]);
      p.position.x = std::stod(cols[2]);
      p.position.y = std::stod(cols[3]);
      p.position.z = std::stod(cols[4]);
      p.yaw = std::stod(cols[5]);
      return true;
    } catch (...) {
      return false;  // 解析失败
    }
  };

  // 第一行：尝试解析，失败就当表头跳过
  if (!std::getline(ifs, line)) {
    return {};
  }
  
  CsvPoint p;
  if (try_parse(line, p)) {
    out.push_back(p);
  }

  // 剩余行：逐行解析
  while (std::getline(ifs, line)) {
    CsvPoint q;
    if (!try_parse(line, q)) {
      RCLCPP_WARN(logger, "跳过无效行: %s", line.c_str());
      continue;
    }
    out.push_back(q);
  }
  
  return out;
}

// 从点列表构建楼层索引（方便按楼层查找）
// 输出：waypoints_by_floor[floor] = sorted vector of CsvPoint
static inline std::unordered_map<int, std::vector<CsvPoint>> buildFloorIndex(
    const std::vector<CsvPoint> &points) {
  
  std::unordered_map<int, std::vector<CsvPoint>> result;
  
  for (const auto &p : points) {
    result[p.floor].push_back(p);
  }
  
  // 每个楼层内按 index 排序
  for (auto &[floor, vec] : result) {
    std::sort(vec.begin(), vec.end(), [](const CsvPoint &a, const CsvPoint &b) {
      return a.index < b.index;
    });
  }
  
  return result;
}

// 从点列表构建 floor-index 双重索引（快速查找指定点）
// 输出：index[floor][index] = CsvPoint
static inline std::unordered_map<int, std::unordered_map<int, CsvPoint>> 
buildFloorIndexMap(const std::vector<CsvPoint> &points) {
  
  std::unordered_map<int, std::unordered_map<int, CsvPoint>> result;
  
  for (const auto &p : points) {
    result[p.floor][p.index] = p;
  }
  
  return result;
}

}  // namespace csv_loader
