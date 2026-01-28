#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <fstream>
#include <cmath>
#include <filesystem>  // 新增：用于路径处理

namespace fs = std::filesystem;  // 新增：文件系统命名空间

class Pcd2PgmNode : public rclcpp::Node {
public:
    Pcd2PgmNode() : Node("pcl_filters") {
        // 声明参数（新增输出目录参数）
        this->declare_parameter("file_directory", "./save/");
        this->declare_parameter("file_name", "map");
        this->declare_parameter("thre_z_min", 0.2);
        this->declare_parameter("thre_z_max", 2.0);
        this->declare_parameter("flag_pass_through", 0);
        this->declare_parameter("thre_radius", 0.5);
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("thres_point_count", 10);
        this->declare_parameter("map_topic_name", "map");
        this->declare_parameter("output_directory", "./save/");  // 新增：声明输出目录参数

        // 获取参数
        std::string file_directory = this->get_parameter("file_directory").as_string();
        std::string file_name = this->get_parameter("file_name").as_string();
        pcd_file = file_directory + file_name + ".pcd";
        
        // 新增：获取输出目录并确保目录存在
        std::string output_directory = this->get_parameter("output_directory").as_string();
        if (!fs::exists(output_directory)) {
            fs::create_directories(output_directory);
        }
        // 使用输出目录构建地图保存路径
        map_save_path = output_directory + "/" + file_name;
        
        thre_z_min = this->get_parameter("thre_z_min").as_double();
        thre_z_max = this->get_parameter("thre_z_max").as_double();
        flag_pass_through = this->get_parameter("flag_pass_through").as_int();
        thre_radius = this->get_parameter("thre_radius").as_double();
        map_resolution = this->get_parameter("map_resolution").as_double();
        thres_point_count = this->get_parameter("thres_point_count").as_int();
        std::string map_topic_name = this->get_parameter("map_topic_name").as_string();

        // 创建发布者
        map_topic_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            map_topic_name, 10);

        // 加载PCD文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file: %s", pcd_file.c_str());
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "初始点云数据点数：%zu", pcd_cloud->points.size());
        
        // 执行滤波
        PassThroughFilter(thre_z_min, thre_z_max, flag_pass_through, output_directory);  // 修改：传递输出目录
        RadiusOutlierFilter(cloud_after_PassThrough, thre_radius, thres_point_count, output_directory);  // 修改：传递输出目录
        
        // 转换为栅格地图
        SetMapTopicMsg(cloud_after_Radius, map_topic_msg);

        // 保存栅格地图
        saveOccupancyGrid(map_topic_msg, map_save_path);

        // 创建定时器，1Hz发布地图
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Pcd2PgmNode::publishMap, this));
    }

private:
    void publishMap() {
        map_topic_pub->publish(map_topic_msg);
    }

    // 保存栅格地图为PGM和YAML文件
    void saveOccupancyGrid(const nav_msgs::msg::OccupancyGrid& map, const std::string& base_path) {
        // 保存PGM图像
        std::string pgm_path = base_path + ".pgm";
        std::ofstream pgm_file(pgm_path, std::ios::binary);
        if (!pgm_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件用于保存PGM: %s", pgm_path.c_str());
            return;
        }

        // PGM文件头
        pgm_file << "P5\n";
        pgm_file << map.info.width << " " << map.info.height << "\n";
        pgm_file << "255\n";

        // 写入像素数据
        for (int y = map.info.height - 1; y >= 0; --y) {
            for (int x = 0; x < map.info.width; ++x) {
                int index = x + y * map.info.width;
                int value = map.data[index];
                
                uint8_t gray;
                if (value == -1) {
                    gray = 205;
                } else if (value == 0) {
                    gray = 255;
                } else {
                    gray = 0;
                }
                pgm_file.put(gray);
            }
        }
        pgm_file.close();
        RCLCPP_INFO(this->get_logger(), "栅格地图PGM已保存至: %s", pgm_path.c_str());

        // 保存YAML元数据
        std::string yaml_path = base_path + ".yaml";
        std::ofstream yaml_file(yaml_path);
        if (!yaml_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件用于保存YAML: %s", yaml_path.c_str());
            return;
        }

        yaml_file << "image: " << pgm_path << "\n";
        yaml_file << "resolution: " << map.info.resolution << "\n";
        yaml_file << "origin: [" << map.info.origin.position.x << ", " 
                  << map.info.origin.position.y << ", " 
                  << map.info.origin.position.z << "]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: 0.65\n";
        yaml_file << "free_thresh: 0.196\n";
        yaml_file.close();
        RCLCPP_INFO(this->get_logger(), "地图元数据YAML已保存至: %s", yaml_path.c_str());
    }

    // 直通滤波（修改：添加输出目录参数）
    void PassThroughFilter(const double &thre_low, const double &thre_high, const int &flag_in, const std::string& output_dir) {
        pcl::PassThrough<pcl::PointXYZ> passthrough;
        passthrough.setInputCloud(pcd_cloud);
        passthrough.setFilterFieldName("z");
        passthrough.setFilterLimits(thre_low, thre_high);
        passthrough.setFilterLimitsNegative(flag_in);
        passthrough.filter(*cloud_after_PassThrough);

        // 保存滤波结果到输出目录
        pcl::io::savePCDFile<pcl::PointXYZ>(output_dir + "/map_filter.pcd", *cloud_after_PassThrough);
        RCLCPP_INFO(this->get_logger(), "直通滤波后点云数据点数：%zu", cloud_after_PassThrough->points.size());
    }

    // 半径滤波（修改：添加输出目录参数）
    void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                            const double &radius, const int &thre_count, const std::string& output_dir) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
        radiusoutlier.setInputCloud(pcd_cloud0);
        radiusoutlier.setRadiusSearch(radius);
        radiusoutlier.setMinNeighborsInRadius(thre_count);
        radiusoutlier.filter(*cloud_after_Radius);

        // 保存滤波结果到输出目录
        pcl::io::savePCDFile<pcl::PointXYZ>(output_dir + "/map_radius_filter.pcd", *cloud_after_Radius);
        RCLCPP_INFO(this->get_logger(), "半径滤波后点云数据点数：%zu", cloud_after_Radius->points.size());
    }

    // 转换为栅格地图数据（保持不变）
    void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       nav_msgs::msg::OccupancyGrid &msg) {
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.info.map_load_time = this->now();
        msg.info.resolution = map_resolution;

        double x_min, x_max, y_min, y_max;
        double z_max_grey_rate = 0.05;
        double z_min_grey_rate = 0.95;
        double k_line = (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
        double b_line = (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) / 
                       (thre_z_max - thre_z_min);

        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "pcd is empty!");
            return;
        }

        // 计算点云边界
        x_min = x_max = cloud->points[0].x;
        y_min = y_max = cloud->points[0].y;
        for (size_t i = 1; i < cloud->points.size(); ++i) {
            double x = cloud->points[i].x;
            double y = cloud->points[i].y;

            x_min = std::min(x, x_min);
            x_max = std::max(x, x_max);
            y_min = std::min(y, y_min);
            y_max = std::max(y, y_max);
        }

        // 设置地图原点和尺寸
        msg.info.origin.position.x = x_min;
        msg.info.origin.position.y = y_min;
        msg.info.origin.position.z = 0.0;
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        msg.info.width = static_cast<int>((x_max - x_min) / map_resolution);
        msg.info.height = static_cast<int>((y_max - y_min) / map_resolution);
        msg.data.resize(msg.info.width * msg.info.height, 0);

        RCLCPP_INFO(this->get_logger(), "data size = %zu", msg.data.size());

        // 填充栅格数据
        for (const auto &point : cloud->points) {
            int i = static_cast<int>((point.x - x_min) / map_resolution);
            if (i < 0 || i >= msg.info.width) continue;

            int j = static_cast<int>((point.y - y_min) / map_resolution);
            if (j < 0 || j >= msg.info.height) continue;

            msg.data[i + j * msg.info.width] = 100;
        }
    }

    // 成员变量
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_topic_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_topic_msg;
    
    std::string pcd_file;
    std::string map_save_path;
    double thre_z_min, thre_z_max;
    int flag_pass_through;
    double map_resolution;
    double thre_radius;
    int thres_point_count;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough = 
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius = 
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud = 
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Pcd2PgmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
