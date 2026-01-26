#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OdometryTransform : public rclcpp::Node
{
public:
    OdometryTransform() : Node("odometry_transform")
    {
        // 声明参数（与原始监听器保持参数风格一致）
        this->declare_parameter<std::string>("odom_topic", "/Odometry");
        
        // 机械臂基座标偏移参数（相对于里程计）
        this->declare_parameter<double>("base_x", 0.0);
        this->declare_parameter<double>("base_y", 0.0);
        this->declare_parameter<double>("base_z", 0.0);
        this->declare_parameter<double>("base_qx", 0.0);
        this->declare_parameter<double>("base_qy", 0.0);
        this->declare_parameter<double>("base_qz", 0.0);
        this->declare_parameter<double>("base_qw", 1.0);
        
        // 机械臂TCP偏移参数（相对于基座标）
        this->declare_parameter<double>("tcp_x", 0.0);
        this->declare_parameter<double>("tcp_y", 0.0);
        this->declare_parameter<double>("tcp_z", 0.0);
        this->declare_parameter<double>("tcp_qx", 0.0);
        this->declare_parameter<double>("tcp_qy", 0.0);
        this->declare_parameter<double>("tcp_qz", 0.0);
        this->declare_parameter<double>("tcp_qw", 1.0);

        // 获取参数值
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        
        // 基座标偏移变换
        base_offset_.setOrigin(tf2::Vector3(
            this->get_parameter("base_x").as_double(),
            this->get_parameter("base_y").as_double(),
            this->get_parameter("base_z").as_double()
        ));
        tf2::Quaternion base_quat(
            this->get_parameter("base_qx").as_double(),
            this->get_parameter("base_qy").as_double(),
            this->get_parameter("base_qz").as_double(),
            this->get_parameter("base_qw").as_double()
        );
        base_offset_.setRotation(base_quat.normalize());
        
        // TCP偏移变换（相对于基座标）
        tcp_offset_.setOrigin(tf2::Vector3(
            this->get_parameter("tcp_x").as_double(),
            this->get_parameter("tcp_y").as_double(),
            this->get_parameter("tcp_z").as_double()
        ));
        tf2::Quaternion tcp_quat(
            this->get_parameter("tcp_qx").as_double(),
            this->get_parameter("tcp_qy").as_double(),
            this->get_parameter("tcp_qz").as_double(),
            this->get_parameter("tcp_qw").as_double()
        );
        tcp_offset_.setRotation(tcp_quat.normalize());

        // 创建订阅者（与原始监听器保持订阅逻辑一致）
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            10,
            std::bind(&OdometryTransform::odometry_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Odometry transform initialized. Subscribing to: %s", odom_topic.c_str());
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        // 转换里程计位姿为tf2变换
        tf2::Transform odom_transform;
        tf2::fromMsg(msg->pose.pose, odom_transform);
        
        // 计算基座标位姿：里程计 × 基座标偏移
        tf2::Transform base_transform = odom_transform * base_offset_;
        
        // 计算TCP最终位姿：基座标 × TCP偏移
        tf2::Transform tcp_transform = base_transform * tcp_offset_;

        // 仅输出TCP坐标（保持与原始监听器一致的打印风格）
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "TCP坐标:\n"
            "  位置: x=" << tcp_transform.getOrigin().x() << ", y=" << tcp_transform.getOrigin().y() << ", z=" << tcp_transform.getOrigin().z() << "\n"
            "  四元数: x=" << tcp_transform.getRotation().x() << ", y=" << tcp_transform.getRotation().y()
                          << ", z=" << tcp_transform.getRotation().z() << ", w=" << tcp_transform.getRotation().w()
        );
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    tf2::Transform base_offset_;  // 基座标相对里程计的偏移
    tf2::Transform tcp_offset_;   // TCP相对基座标的偏移
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTransform>());
    rclcpp::shutdown();
    return 0;
}
