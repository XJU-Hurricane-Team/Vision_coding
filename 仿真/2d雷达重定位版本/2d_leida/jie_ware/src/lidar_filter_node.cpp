#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <limits> 
#include <cmath> 

class CLidarFilter : public rclcpp::Node
{
public:
    CLidarFilter() : Node("lidar_filter_node")
    {
        // 参数声明
        this->declare_parameter<std::string>("source_topic", "/scan");
        this->declare_parameter<std::string>("pub_topic", "/scan_filtered");
        this->declare_parameter<double>("outlier_threshold", 0.1);

        // 获取参数
        this->get_parameter("source_topic", source_topic_name_);
        this->get_parameter("pub_topic", pub_topic_name_);
        this->get_parameter("outlier_threshold", outlier_threshold_);

        // 创建发布者和订阅者
        // QoS 设置为 10，相当于 KeepLast(10)
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic_name_, 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            source_topic_name_, 10, std::bind(&CLidarFilter::lidarCallback, this, std::placeholders::_1));
    }

private:
     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
     std::string source_topic_name_;
     std::string pub_topic_name_;
     double outlier_threshold_;

     void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
     {
        int nRanges = scan->ranges.size();

        if (nRanges < 3) 
        {
            scan_pub_->publish(*scan);
            return;
        }

        auto new_scan = *scan; // 深度拷贝

        for (int i = 1; i < nRanges - 1; ++i)
        {
            float prev_range = new_scan.ranges[i-1];
            float current_range = new_scan.ranges[i];
            float next_range = new_scan.ranges[i+1];

            bool current_valid = std::isfinite(current_range) && 
                                 current_range >= new_scan.range_min && 
                                 current_range <= new_scan.range_max;

            if (!current_valid) continue;

            if (std::abs(current_range - prev_range) > outlier_threshold_ &&
                std::abs(current_range - next_range) > outlier_threshold_)
            {
                new_scan.ranges[i] = std::numeric_limits<float>::infinity();
                if (!new_scan.intensities.empty() && i < (int)new_scan.intensities.size()) 
                {
                    new_scan.intensities[i] = 0.0f; 
                }
            }
        }

        scan_pub_->publish(new_scan);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CLidarFilter>());
    rclcpp::shutdown();
    return 0; 
}