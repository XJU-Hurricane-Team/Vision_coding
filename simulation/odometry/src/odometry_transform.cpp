#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath> // 用于 atan2 和 M_PI

class OdometryTransform : public rclcpp::Node
{
public:
    OdometryTransform() : Node("odometry_transform")
    {
        // 声明参数
        this->declare_parameter<std::string>("target_frame", "map");
        this->declare_parameter<std::string>("source_frame", "base_footprint");
        
        // 机械臂偏移参数声明 (保持不变)
        this->declare_parameter<double>("base_x", 0.0);
        this->declare_parameter<double>("base_y", 0.0);
        this->declare_parameter<double>("base_z", 0.0);
        this->declare_parameter<double>("base_qx", 0.0);
        this->declare_parameter<double>("base_qy", 0.0);
        this->declare_parameter<double>("base_qz", 0.0);
        this->declare_parameter<double>("base_qw", 1.0);
        
        this->declare_parameter<double>("tcp_x", 0.0);
        this->declare_parameter<double>("tcp_y", 0.0);
        this->declare_parameter<double>("tcp_z", 0.0);
        this->declare_parameter<double>("tcp_qx", 0.0);
        this->declare_parameter<double>("tcp_qy", 0.0);
        this->declare_parameter<double>("tcp_qz", 0.0);
        this->declare_parameter<double>("tcp_qw", 1.0);

        // 初始化偏移量
        init_offsets();

        // 初始化 TF 监听
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建定时器 (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdometryTransform::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Transform Node Initialized. Listening to TF: map -> base_footprint");
    }

private:
    void init_offsets()
    {
        // 设置基座标偏移
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
        
        // 设置 TCP 偏移
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
    }

    void timer_callback()
    {
        std::string target_frame = this->get_parameter("target_frame").as_string();
        std::string source_frame = this->get_parameter("source_frame").as_string();

        try {
            // 1. 查询 map 到 base_footprint 的变换
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

            tf2::Transform map_to_base;
            tf2::fromMsg(t.transform, map_to_base);

            // 2. 叠加偏移：Map -> Base -> TCP
            tf2::Transform tcp_transform = map_to_base * base_offset_ * tcp_offset_;

            // 3. 计算 Yaw (角度)
            tf2::Quaternion q = tcp_transform.getRotation();
            double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
            double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
            double yaw_deg = std::atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

            // 4. 输出
            RCLCPP_INFO_STREAM(this->get_logger(), 
                "TCP 全局坐标:\n"
                "  XYZ: " << tcp_transform.getOrigin().x() << ", " 
                          << tcp_transform.getOrigin().y() << ", " 
                          << tcp_transform.getOrigin().z() << "\n"
                "  Yaw: " << yaw_deg << "°"
            );

        } catch (const tf2::TransformException & ex) {
            // 启动初期可能查不到 TF，属于正常现象
            RCLCPP_DEBUG(this->get_logger(), "Waiting for transform: %s", ex.what());
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::Transform base_offset_;
    tf2::Transform tcp_offset_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTransform>());
    rclcpp::shutdown();
    return 0;
}