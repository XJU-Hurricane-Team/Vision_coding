#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;
using Patrol = chapt4_interfaces::srv::Patrol;

class TurtleController : public rclcpp::Node {
    public:
        TurtleController() : Node("turtle_controller") {
            // 声明和获取参数初始值
            this->declare_parameter("k", 1.0);
            this->declare_parameter("max_speed",1.0);
            this->get_parameter("k",k_);
            this->get_parameter("max_speed",max_speed_);
            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleController::on_pose_received, this, std::placeholders::_1));
            // 3.创建服务
            patrol_server_ = this->create_service<Patrol>(
                "patrol",
                [&](const std::shared_ptr<Patrol::Request> request, const std::shared_ptr<Patrol::Response> response) -> void{
                    // 判断逻辑点是否在模拟器边界内
                    if((0 < request->target_x && request->target_x < 12.0f) && (0 < request->target_y && request->target_y < 12.0f)){
                        target_x_ = request->target_x;
                        target_y_ = request->target_y;
                        response->result = Patrol::Response::SUCCESS;
                    }
                    else{
                        response->result = Patrol::Response::FAIL;
                    }
                });
            // 添加参数设置回调
            parameters_callback_handle_ = this->add_on_set_parameters_callback(
                [&](const std::vector<rclcpp::Parameter> &params)->SetParametersResult {
                    // 遍历参数
                    for (auto param : params) {
                        RCLCPP_INFO(this->get_logger(), " Updating parameter %s to value %f", param.get_name().c_str(), param.as_double());
                        if (param.get_name() == "k") {
                            k_ = param.as_double();
                        } else if (param.get_name() == "max_speed") {
                            max_speed_ = param.as_double();
                        }
                    }
                    auto result = SetParametersResult();
                    result.successful = true;
                    return result;
                    }
            );
        }
    private:
        void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose) {
            auto message = geometry_msgs::msg::Twist();
            // 1. Record current position
            double current_x = pose->x;
            double current_y = pose->y;
            RCLCPP_INFO(this->get_logger(), " Current position: (x=%f,y=%f)", current_x, current_y);
            // 2. Calculate distance and angle difference to target, and angle difference to current heading
            double distance = std::sqrt((target_x_ - current_x) * (target_x_ - current_x) + (target_y_ - current_y) * (target_y_ - current_y));
            double angle = std::atan2(target_y_ - current_y, target_x_ - current_x) - pose->theta;
            // 3. Control strategy: continue moving if distance is greater than 0.1, turn if angle difference is greater than 0.2, otherwise go straight
            if (distance > 0.1) {
                if (fabs(angle) > 0.2) {
                    message.angular.z = std::fabs(angle);
                } else {
                    // Calculate output speed using proportional controller
                    message.linear.x = k_ * distance;
                }
            }
            // 4. Limit maximum value and publish message
            if (message.linear.x > max_speed_) {
                message.linear.x = max_speed_;
            }
            velocity_publisher_->publish(message);
        }
    private:
        // 2. 添加Patrol类型服务共享指针 patrol_server_为成员变量
        OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
        rclcpp::Service<Patrol>::SharedPtr patrol_server_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;   
        double target_x_{1.0}; // Target position x, set default value 1.0
        double target_y_{1.0}; // Target position y, set default value 1.0
        double k_{1.0}; // Proportional coefficient, control output = error * k_
        double max_speed_{3.0}; // Maximum linear speed, set default value 3.0
    };
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}