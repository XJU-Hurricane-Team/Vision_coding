#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath> // 用于 atan2 和 M_PI

class OdometryListener : public rclcpp::Node
{
public:
    OdometryListener() : Node("odometry_listener")
    {
        this->declare_parameter<std::string>("target_frame", "map");
        this->declare_parameter<std::string>("source_frame", "base_footprint");
        
        // 【新增】声明 XYZ 三轴偏移参数
        this->declare_parameter<double>("x_offset", 0.0);
        this->declare_parameter<double>("y_offset", 0.0);
        this->declare_parameter<double>("z_offset", 0.0); // 默认值在launch文件中指定

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 10Hz 定时查询
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdometryListener::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Listener Node Initialized. Watching: map -> base_footprint");
    }

private:
    void timer_callback()
    {
        std::string target_frame = this->get_parameter("target_frame").as_string();
        std::string source_frame = this->get_parameter("source_frame").as_string();
        
        // 【新增】获取偏移量参数
        double x_off = this->get_parameter("x_offset").as_double();
        double y_off = this->get_parameter("y_offset").as_double();
        double z_off = this->get_parameter("z_offset").as_double();

        try {
            // 查询 TF 变换
            geometry_msgs::msg::TransformStamped t;
            t = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

            // 提取位置并加上偏移
            double x = t.transform.translation.x + x_off;
            double y = t.transform.translation.y + y_off;
            double z = t.transform.translation.z + z_off;

            // 提取 Yaw (角度)
            auto q = t.transform.rotation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            double yaw_deg = std::atan2(siny_cosp, cosy_cosp) * (180.0 / M_PI);

            // 输出
            RCLCPP_INFO_STREAM(this->get_logger(), 
                "机器人当前位置 (Map Frame) [Offset XYZ: " << x_off << ", " << y_off << ", " << z_off << "]:\n"
                "  XYZ: " << x << ", " << y << ", " << z << "\n"
                "  Yaw: " << yaw_deg << "°"
            );

        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for transform: %s", ex.what());
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryListener>());
    rclcpp::shutdown();
    return 0;
}