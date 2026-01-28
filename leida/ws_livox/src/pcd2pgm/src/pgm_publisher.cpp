#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>

class PgmPublisherNode : public rclcpp::Node {
public:
    PgmPublisherNode() : Node("pgm_publisher") {
        // 声明参数
        this->declare_parameter("yaml_path", "./save/scans.yaml");
        this->declare_parameter("map_topic", "scans_map");
        
        // 获取参数
        std::string yaml_path = this->get_parameter("yaml_path").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        
        // 创建发布者
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 10);
        
        // 加载并发布地图
        if (loadMapFromYaml(yaml_path)) {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&PgmPublisherNode::publishMap, this)
            );
            RCLCPP_INFO(this->get_logger(), "成功加载地图，开始发布到话题: %s", map_topic.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "地图加载失败，节点将退出");
            rclcpp::shutdown();
        }
    }

private:
    bool loadMapFromYaml(const std::string& yaml_path) {
        try {
            // 解析YAML文件
            YAML::Node config = YAML::LoadFile(yaml_path);
            
            // 读取YAML参数
            std::string pgm_path = config["image"].as<std::string>();
            double resolution = config["resolution"].as<double>();
            auto origin = config["origin"].as<std::vector<double>>();
            bool negate = config["negate"].as<int>() != 0;
            double occupied_thresh = config["occupied_thresh"].as<double>();
            double free_thresh = config["free_thresh"].as<double>();
            
            // 读取PGM文件
            if (!loadPgmFile(pgm_path, negate, occupied_thresh, free_thresh)) {
                return false;
            }
            
            // 设置地图元数据
            map_msg_.info.resolution = resolution;
            map_msg_.info.origin.position.x = origin[0];
            map_msg_.info.origin.position.y = origin[1];
            map_msg_.info.origin.position.z = origin[2];
            map_msg_.info.origin.orientation.w = 1.0;  // 无旋转
            map_msg_.header.frame_id = "map";
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "解析YAML失败: %s", e.what());
            return false;
        }
    }

    bool loadPgmFile(const std::string& pgm_path, bool negate, 
                    double occupied_thresh, double free_thresh) {
        std::ifstream file(pgm_path, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开PGM文件: %s", pgm_path.c_str());
            return false;
        }
        
        // 解析PGM头部
        std::string magic;
        int width, height, max_val;
        file >> magic;
        if (magic != "P5") {
            RCLCPP_ERROR(this->get_logger(), "不支持的PGM格式，只支持P5");
            return false;
        }
        
        file >> width >> height >> max_val;
        file.ignore();  // 忽略换行符
        
        // 读取像素数据
        map_msg_.info.width = width;
        map_msg_.info.height = height;
        map_msg_.data.resize(width * height);
        
        std::vector<uint8_t> pgm_data(width * height);
        file.read(reinterpret_cast<char*>(pgm_data.data()), width * height);
        
        // 转换为OccupancyGrid数据 (-1:未知, 0:空闲, 100:占用)
        for (int i = 0; i < width * height; ++i) {
            double value = static_cast<double>(pgm_data[i]) / max_val;
            
            if (!negate) {
                value = 1.0 - value;
            }
            
            if (value > occupied_thresh) {
                map_msg_.data[i] = 100;  // 占用
            } else if (value < free_thresh) {
                map_msg_.data[i] = 0;    // 空闲
            } else {
                map_msg_.data[i] = -1;   // 未知
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "成功加载PGM文件: %s (%dx%d)", 
                   pgm_path.c_str(), width, height);
        return true;
    }

    void publishMap() {
        map_msg_.header.stamp = this->now();
        map_pub_->publish(map_msg_);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_msg_;
};

int main(int argc, char**argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PgmPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
