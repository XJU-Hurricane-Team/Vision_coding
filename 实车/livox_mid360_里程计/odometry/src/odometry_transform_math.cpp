#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath> // 引入标准数学库

class OdometryTransformMath : public rclcpp::Node
{
public:
    OdometryTransformMath() : Node("odometry_transform_math")
    {
        // 1. 获取【统一地图原点】
        this->declare_parameter("map_origin_x", -5.543);
        this->declare_parameter("map_origin_y", -5.998);
        this->declare_parameter("map_origin_z", 0.000); 
        this->declare_parameter("map_origin_qz", 0.000);
        this->declare_parameter("map_origin_qw", 1.000);

        // 2. 获取【雷达在地图上的真实起步点】
        this->declare_parameter("lidar_init_x", -5.293);
        this->declare_parameter("lidar_init_y", -4.090);
        this->declare_parameter("lidar_init_z", 0.000); 
        this->declare_parameter("lidar_init_qz", 0.000);
        this->declare_parameter("lidar_init_qw", 1.000);

        double mx = this->get_parameter("map_origin_x").as_double();
        double my = this->get_parameter("map_origin_y").as_double();
        double mz = this->get_parameter("map_origin_z").as_double();
        double mqz = this->get_parameter("map_origin_qz").as_double();
        double mqw = this->get_parameter("map_origin_qw").as_double();

        double lx = this->get_parameter("lidar_init_x").as_double();
        double ly = this->get_parameter("lidar_init_y").as_double();
        double lz = this->get_parameter("lidar_init_z").as_double();
        double lqz = this->get_parameter("lidar_init_qz").as_double();
        double lqw = this->get_parameter("lidar_init_qw").as_double();

        // 3. 【核心纯数学预计算：将四元数转换为偏航角 Yaw】
        double map_yaw = 2.0 * std::atan2(mqz, mqw);
        double lidar_yaw = 2.0 * std::atan2(lqz, lqw);

        // A. 计算直线差值
        double dx = lx - mx;
        double dy = ly - my;
        
        // B. 预计算偏移坐标 (投射到地图原点的角度系下)
        offset_x_ = dx * std::cos(map_yaw) - dy * std::sin(map_yaw);
        offset_y_ = dx * std::sin(map_yaw) + dy * std::cos(map_yaw);
        offset_z_ = lz - mz;

        // C. 预计算偏航角的固定差值
        offset_yaw_ = lidar_yaw - map_yaw;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_raw", 10, std::bind(&OdometryTransformMath::odom_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_map", 10);

        RCLCPP_INFO(this->get_logger(), "== 纯数学推导版节点 (odometry_transform_math) 已启动 ==");
        RCLCPP_INFO(this->get_logger(), "X: %.3f, Y: %.3f, Yaw: %.3f", offset_x_, offset_y_, offset_yaw_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double rx = msg->pose.pose.position.x;
        double ry = msg->pose.pose.position.y;
        double rz = msg->pose.pose.position.z;

        double raw_qz = msg->pose.pose.orientation.z;
        double raw_qw = msg->pose.pose.orientation.w;
        double raw_yaw = 2.0 * std::atan2(raw_qz, raw_qw);

        // 【如果发现Y轴误差非常大且总是相反，可以尝试把下面这行的 + ry 改为 - ry】
        double final_x = offset_x_ + (rx * std::cos(offset_yaw_) - ry * std::sin(offset_yaw_));
        double final_y = offset_y_ + (rx * std::sin(offset_yaw_) + ry * std::cos(offset_yaw_));
        double final_yaw = offset_yaw_ + raw_yaw;

        geometry_msgs::msg::PoseStamped pure_pose;
        pure_pose.header = msg->header;
        pure_pose.header.frame_id = "unified_map"; 

        pure_pose.pose.position.x = final_x;
        pure_pose.pose.position.y = final_y;
        pure_pose.pose.position.z = rz + 0.024;

        pure_pose.pose.orientation.w = std::cos(final_yaw / 2.0);
        pure_pose.pose.orientation.z = std::sin(final_yaw / 2.0);
        pure_pose.pose.orientation.x = 0.0;
        pure_pose.pose.orientation.y = 0.0;

        pose_pub_->publish(pure_pose);

        RCLCPP_INFO(this->get_logger(), 
            "Math X: %.3f, Y: %.3f, Z: %.3f, qz: %.3f, qw: %.3f", 
            pure_pose.pose.position.x, 
            pure_pose.pose.position.y,
            pure_pose.pose.position.z,
            pure_pose.pose.orientation.z,
            pure_pose.pose.orientation.w);
    }

    double offset_x_, offset_y_, offset_z_, offset_yaw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTransformMath>());
    rclcpp::shutdown();
    return 0;
}