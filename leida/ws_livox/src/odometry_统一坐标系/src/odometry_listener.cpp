#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>

class OdometryListener : public rclcpp::Node
{
public:
    OdometryListener() : Node("odometry_listener")
    {
        this->declare_parameter<std::string>("odom_topic", "/Odometry");
        
        // 1. 声明四个误差补偿参数 (默认都是 0.0)
        this->declare_parameter<double>("offset_x", 0.0);
        this->declare_parameter<double>("offset_y", 0.0);
        this->declare_parameter<double>("offset_z", 0.0);
        this->declare_parameter<double>("offset_w", 0.0);
        
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            10,
            std::bind(&OdometryListener::odometry_callback, this, std::placeholders::_1)
        );
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        auto position = msg->pose.pose.position;
        auto orientation = msg->pose.pose.orientation;
        
        // 2. 实时获取传入的补偿值
        double ox = this->get_parameter("offset_x").as_double();
        double oy = this->get_parameter("offset_y").as_double();
        double oz = this->get_parameter("offset_z").as_double();
        double ow = this->get_parameter("offset_w").as_double();
        
        // 3. 打印时减去补偿值，实现完美清零
        std::cout << "X " << position.x + ox << " "
                  << "Y " << position.y + oy << " " 
                  << "qZ " << orientation.z + oz << " "
                  << "qW " << orientation.w + ow << std::endl;
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