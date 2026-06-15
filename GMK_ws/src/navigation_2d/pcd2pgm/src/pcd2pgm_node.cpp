#include <rclcpp/rclcpp.hpp>

// PCL：点云加载与预处理滤波
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

// octomap：三维体素化与二维投影
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <filesystem>

/**
 * @brief PCD → PGM/YAML 地图转换节点（octomap 版）
 *
 * 原始工作流（双节点）：
 *   pcd2map → 发布 /map_cloud → octomap_server 订阅 → 构建 OcTree → 投影 2D → map_saver 保存
 *
 * 本节点将上述流程合并到单一可执行文件：
 *   PCD 加载 → PCL 滤波 → octomap::OcTree 体素化 → 2D 投影 → PGM/YAML 写出
 *
 * 使用 octomap 而非纯 PCL 的原因：
 *   1. OcTree 自动合并同一体素的多次命中（天然去重），分辨率与地图像素严格对应
 *   2. 与 octomap_server 产生相同的体素语义，输出可相互参照
 *   3. getMetricMin/Max() 直接提供地图边界，无需额外遍历点云
 *
 * 输出文件：
 *   <map_name>.pgm  — 灰度图（0=障碍/黑色，254=自由/白色，205=未知/灰色）
 *   <map_name>.yaml — nav2_map_server 元数据
 */
class Pcd2PgmNode : public rclcpp::Node
{
public:
    Pcd2PgmNode() : Node("pcd2pgm")
    {
        // ── 声明参数 ──────────────────────────────────────────────────────────
        declare_parameter<std::string>("pcd_file",   "");
        declare_parameter<std::string>("output_dir", ".");
        declare_parameter<std::string>("map_name",   "map");

        // 体素/栅格分辨率（米），同时决定 octomap 的分辨率和 PGM 的像素大小
        declare_parameter<double>("resolution", 0.05);

        // 高度切片范围：只将 z ∈ [minh, maxh] 的点插入八叉树
        declare_parameter<double>("minh",    -0.3);
        declare_parameter<double>("maxh",     1.5);

        // 地图边界向外扩展的留白（米）
        declare_parameter<double>("padding",  1.0);

        // 可选：体素降采样（在插入 octomap 前减少点数，加速插入）
        declare_parameter<bool>  ("enable_voxel_filter",  false);
        declare_parameter<double>("voxel_leaf_size",      0.05);

        // 可选：半径离群点去除（消除噪声点，避免引入错误占用体素）
        declare_parameter<bool>  ("enable_radius_filter", false);
        declare_parameter<double>("radius_filter",        0.3);
        declare_parameter<int>   ("min_neighbors",        5);

        // 非占用栅格默认值："free"=254（白色，推荐室内），"unknown"=205（灰色，保守）
        declare_parameter<std::string>("default_cell",    "free");

        // 写入 YAML 的阈值参数（与 nav2_map_server / octomap_server 保持一致）
        declare_parameter<double>("occupied_thresh", 0.65);
        declare_parameter<double>("free_thresh",     0.196);

        // ── 读取参数 ─────────────────────────────────────────────────────────
        pcd_file_       = get_parameter("pcd_file").as_string();
        output_dir_     = get_parameter("output_dir").as_string();
        map_name_       = get_parameter("map_name").as_string();
        resolution_     = get_parameter("resolution").as_double();
        minh_           = get_parameter("minh").as_double();
        maxh_           = get_parameter("maxh").as_double();
        padding_        = get_parameter("padding").as_double();
        enable_voxel_   = get_parameter("enable_voxel_filter").as_bool();
        voxel_leaf_     = get_parameter("voxel_leaf_size").as_double();
        enable_radius_  = get_parameter("enable_radius_filter").as_bool();
        radius_         = get_parameter("radius_filter").as_double();
        min_neighbors_  = get_parameter("min_neighbors").as_int();
        default_cell_   = get_parameter("default_cell").as_string();
        occ_thresh_     = get_parameter("occupied_thresh").as_double();
        free_thresh_    = get_parameter("free_thresh").as_double();

        // ── 参数校验 ─────────────────────────────────────────────────────────
        if (pcd_file_.empty()) {
            RCLCPP_FATAL(get_logger(), "参数 'pcd_file' 未设置");
            throw std::runtime_error("pcd_file 参数为空");
        }
        if (maxh_ <= minh_) {
            RCLCPP_FATAL(get_logger(), "高度范围无效：maxh(%.2f) <= minh(%.2f)", maxh_, minh_);
            throw std::runtime_error("高度范围参数无效");
        }
        if (resolution_ <= 0.0) {
            RCLCPP_FATAL(get_logger(), "resolution 必须大于 0，当前值：%.4f", resolution_);
            throw std::runtime_error("resolution 参数无效");
        }

        // ── 执行转换 ─────────────────────────────────────────────────────────
        convert();
    }

private:
    // ── 主转换流程 ─────────────────────────────────────────────────────────────
    void convert()
    {
        // ── 步骤 1：加载 PCD ──────────────────────────────────────────────────
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud) == -1) {
            RCLCPP_FATAL(get_logger(), "加载 PCD 失败：%s", pcd_file_.c_str());
            throw std::runtime_error("PCD 加载失败");
        }
        RCLCPP_INFO(get_logger(), "加载 PCD：%s（%zu 点）", pcd_file_.c_str(), cloud->size());

        // ── 步骤 2：高度滤波（PassThrough Z 轴）─────────────────────────────
        {
            auto tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(static_cast<float>(minh_), static_cast<float>(maxh_));
            pass.filter(*tmp);
            RCLCPP_INFO(get_logger(), "高度滤波 z∈[%.2f, %.2f]：%zu → %zu 点",
                minh_, maxh_, cloud->size(), tmp->size());
            cloud = tmp;
        }
        if (cloud->empty()) {
            RCLCPP_FATAL(get_logger(), "高度滤波后点云为空，请检查 minh/maxh 参数");
            throw std::runtime_error("滤波后点云为空");
        }

        // ── 步骤 3：体素降采样（可选，加速 octomap 插入）────────────────────
        if (enable_voxel_) {
            auto tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(
                static_cast<float>(voxel_leaf_),
                static_cast<float>(voxel_leaf_),
                static_cast<float>(voxel_leaf_));
            voxel.filter(*tmp);
            RCLCPP_INFO(get_logger(), "体素降采样（%.3f m）：%zu → %zu 点",
                voxel_leaf_, cloud->size(), tmp->size());
            cloud = tmp;
        }

        // ── 步骤 4：半径离群点去除（可选，消除噪声点）───────────────────────
        if (enable_radius_) {
            auto tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
            ror.setInputCloud(cloud);
            ror.setRadiusSearch(radius_);
            ror.setMinNeighborsInRadius(min_neighbors_);
            ror.filter(*tmp);
            RCLCPP_INFO(get_logger(), "半径离群点去除（r=%.2f，邻居≥%d）：%zu → %zu 点",
                radius_, min_neighbors_, cloud->size(), tmp->size());
            cloud = tmp;
        }

        // ── 步骤 5：将点云插入 octomap::OcTree ──────────────────────────────
        //
        // 说明：PCD 文件不含传感器原点，无法做 ray casting 标记自由空间。
        // 此处仅将点标记为 occupied（free space 由 default_cell 参数决定）。
        RCLCPP_INFO(get_logger(), "开始插入 octomap（分辨率 %.4f m，共 %zu 点）...",
            resolution_, cloud->size());

        octomap::OcTree tree(resolution_);
        for (const auto & pt : cloud->points) {
            tree.updateNode(
                octomap::point3d(
                    static_cast<float>(pt.x),
                    static_cast<float>(pt.y),
                    static_cast<float>(pt.z)),
                true);   // true = occupied
        }
        // 更新内部节点的占用概率（多分辨率一致性）
        tree.updateInnerOccupancy();
        RCLCPP_INFO(get_logger(), "octomap 插入完成，叶节点数：%zu", tree.getNumLeafNodes());

        // ── 步骤 6：获取地图边界（含留白 padding）───────────────────────────
        double xmin_tree, ymin_tree, zmin_tree;
        double xmax_tree, ymax_tree, zmax_tree;
        tree.getMetricMin(xmin_tree, ymin_tree, zmin_tree);
        tree.getMetricMax(xmax_tree, ymax_tree, zmax_tree);

        // 加留白，防止边缘点被裁切
        const double xmin = xmin_tree - padding_;
        const double xmax = xmax_tree + padding_;
        const double ymin = ymin_tree - padding_;
        const double ymax = ymax_tree + padding_;

        // 计算栅格尺寸（向上取整保证全覆盖）
        const int map_width  = static_cast<int>(std::ceil((xmax - xmin) / resolution_));
        const int map_height = static_cast<int>(std::ceil((ymax - ymin) / resolution_));

        RCLCPP_INFO(get_logger(), "地图边界：X[%.2f, %.2f] Y[%.2f, %.2f]",
            xmin, xmax, ymin, ymax);
        RCLCPP_INFO(get_logger(), "栅格尺寸：%d × %d 像素（%.4f m/px）",
            map_width, map_height, resolution_);

        // ── 步骤 7：初始化 2D 栅格 ─────────────────────────────────────────
        // "free"   → 254（白色，适合室内建图，导航可规划）
        // "unknown"→ 205（灰色，保守策略）
        const uint8_t init_val = (default_cell_ == "unknown") ? 205u : 254u;
        std::vector<uint8_t> grid(
            static_cast<size_t>(map_width * map_height), init_val);

        // ── 步骤 8：遍历 octomap 叶节点，投影到 2D ─────────────────────────
        //
        // PGM 坐标系：原点在左上角，行从上到下递增
        // ROS 地图坐标系：原点在左下角，y 轴朝上
        // → 对 row 做镜像翻转：pgm_row = (map_height - 1) - row
        int occupied_count = 0;
        for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
            // 只处理被标记为 occupied 的节点
            if (!tree.isNodeOccupied(*it)) {
                continue;
            }
            // 过滤超出高度范围的节点（防止插入时已过滤但 inner 节点仍存在的情况）
            const double z = it.getZ();
            if (z < minh_ || z > maxh_) {
                continue;
            }

            // 计算对应的 2D 栅格坐标
            const int col     = static_cast<int>((it.getX() - xmin) / resolution_);
            const int row     = static_cast<int>((it.getY() - ymin) / resolution_);
            const int pgm_row = map_height - 1 - row;  // 上下翻转

            if (col >= 0 && col < map_width && pgm_row >= 0 && pgm_row < map_height) {
                grid[static_cast<size_t>(pgm_row * map_width + col)] = 0u;  // 0 = 障碍（黑色）
                ++occupied_count;
            }
        }
        RCLCPP_INFO(get_logger(), "2D 投影完成，占用格数：%d / %d",
            occupied_count, map_width * map_height);

        // ── 步骤 9：写出文件 ──────────────────────────────────────────────────
        std::filesystem::create_directories(output_dir_);
        const std::string pgm_path  = output_dir_ + "/" + map_name_ + ".pgm";
        const std::string yaml_path = output_dir_ + "/" + map_name_ + ".yaml";

        savePgm(pgm_path, grid, map_width, map_height);
        saveYaml(yaml_path, map_name_ + ".pgm", xmin, ymin);

        RCLCPP_INFO(get_logger(), "转换完成：");
        RCLCPP_INFO(get_logger(), "  → %s", pgm_path.c_str());
        RCLCPP_INFO(get_logger(), "  → %s", yaml_path.c_str());
    }

    // ── 写出 PGM（P5 二进制格式，与 map_saver 输出格式一致）────────────────
    void savePgm(const std::string & path,
                 const std::vector<uint8_t> & data,
                 int width, int height)
    {
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("无法写入 PGM 文件：" + path);
        }
        // P5 头部：格式魔数、宽高、最大灰度值
        file << "P5\n"
             << "# generated by pcd2pgm (octomap)\n"
             << width << " " << height << "\n"
             << "255\n";
        file.write(reinterpret_cast<const char *>(data.data()),
                   static_cast<std::streamsize>(data.size()));
    }

    // ── 写出 YAML（nav2_map_server / map_server 标准格式）───────────────────
    void saveYaml(const std::string & path,
                  const std::string & image_filename,
                  double origin_x, double origin_y)
    {
        std::ofstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("无法写入 YAML 文件：" + path);
        }
        // image 只写文件名（不含路径），方便地图文件整体移动
        file << std::fixed << std::setprecision(6)
             << "image: "          << image_filename         << "\n"
             << "resolution: "     << resolution_             << "\n"
             << "origin: ["        << origin_x << ", "
                                   << origin_y << ", 0.0]\n"
             << "negate: 0\n"
             << "occupied_thresh: "<< occ_thresh_             << "\n"
             << "free_thresh: "    << free_thresh_            << "\n"
             << "mode: trinary\n";  // nav2 兼容模式
    }

    // ── 参数成员变量 ──────────────────────────────────────────────────────────
    std::string pcd_file_;
    std::string output_dir_;
    std::string map_name_;
    double      resolution_;
    double      minh_;
    double      maxh_;
    double      padding_;
    bool        enable_voxel_;
    double      voxel_leaf_;
    bool        enable_radius_;
    double      radius_;
    int         min_neighbors_;
    std::string default_cell_;
    double      occ_thresh_;
    double      free_thresh_;
};

// ── 入口：一次性转换，完成后退出 ──────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<Pcd2PgmNode>();
        // 构造函数完成全部工作，无需 spin
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("pcd2pgm"), "转换失败：%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
