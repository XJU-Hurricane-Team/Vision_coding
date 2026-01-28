#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometryListener : public rclcpp::Node
{
public:
    OdometryListener() : Node("odometry_listener")
    {
        this->declare_parameter<std::string>("odom_topic", "/Odometry");
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            10,
            std::bind(&OdometryListener::odometry_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Odometry listener initialized. Subscribing to: %s", odom_topic.c_str());
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        // 提取位置信息
        auto position = msg->pose.pose.position;
        
        // 提取姿态四元数
        auto orientation = msg->pose.pose.orientation;
        
        // 打印位置和四元数信息
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "Received odometry data:\n"
            "  Position: x=" << position.x << ", y=" << position.y << ", z=" << position.z << "\n"
            "  Quaternion: x=" << orientation.x << ", y=" << orientation.y 
                          << ", z=" << orientation.z << ", w=" << orientation.w
        );
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryListener>());
    rclcpp::shutdown();
    return 0;
}
