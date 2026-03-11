#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // <--- 引入极简的位姿消息头文件
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OdometryTransform : public rclcpp::Node
{
public:
    OdometryTransform() : Node("odometry_transform")
    {
        // 1. 获取【统一地图原点】(最终给用户看的 0,0,0 点)
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

        // 3. 构建这两个点的全局位姿变换矩阵
        tf2::Transform T_Global_MapOrigin; 
        T_Global_MapOrigin.setOrigin(tf2::Vector3(mx, my, mz));
        T_Global_MapOrigin.setRotation(tf2::Quaternion(0, 0, mqz, mqw).normalized());

        tf2::Transform T_Global_LidarInit; 
        T_Global_LidarInit.setOrigin(tf2::Vector3(lx, ly, lz));
        T_Global_LidarInit.setRotation(tf2::Quaternion(0, 0, lqz, lqw).normalized());

        // 4. 【核心一步】预计算：雷达起步点相对于地图原点的固定偏移量
        map_to_lidar_init_ = T_Global_MapOrigin.inverse() * T_Global_LidarInit;

        // 订阅雷达原生的相对里程计
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_raw", 10, std::bind(&OdometryTransform::odom_callback, this, std::placeholders::_1));

        // 【关键改动】发布的话题类型改为 geometry_msgs::msg::PoseStamped，只包含坐标和姿态
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("odom_map", 10);

        RCLCPP_INFO(this->get_logger(), "== 双坐标对齐节点 (极简位姿输出版) 已启动 ==");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // A. 获取雷达内部传来的里程计
        tf2::Transform raw_transform;
        tf2::fromMsg(msg->pose.pose, raw_transform);

        // B. 计算最终的映射坐标 = 固定的起步偏移量 * 雷达当前的行驶距离
        tf2::Transform current_in_map = map_to_lidar_init_ * raw_transform;

        // C. 【关键改动】创建一个清爽的 PoseStamped 消息
        geometry_msgs::msg::PoseStamped pure_pose;
        pure_pose.header = msg->header;
        pure_pose.header.frame_id = "unified_map"; 

        // 将算好的矩阵转换为 ROS 消息格式 (此时 X, Y, Z, 姿态 都是映射后的)
        tf2::toMsg(current_in_map, pure_pose.pose);
        
        // 强制还原 Z 轴数据！让它等于底层原生发出的 Z
        pure_pose.pose.position.z = msg->pose.pose.position.z+0.024;

        // 直接发布出去，没有 Twist，也没有 Covariance！
        pose_pub_->publish(pure_pose);

        // D. 终端打印
        RCLCPP_INFO(this->get_logger(), 
            "X: %.3f, Y: %.3f, Z: %.3f, qz: %.3f, qw: %.3f", 
            pure_pose.pose.position.x, 
            pure_pose.pose.position.y,
            pure_pose.pose.position.z,
            pure_pose.pose.orientation.z,
            pure_pose.pose.orientation.w);
    }

    tf2::Transform map_to_lidar_init_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTransform>());
    rclcpp::shutdown();
    return 0;
}