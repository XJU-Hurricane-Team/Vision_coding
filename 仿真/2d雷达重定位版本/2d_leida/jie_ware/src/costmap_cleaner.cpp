#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <chrono>

class CostmapClearer : public rclcpp::Node
{
public:
    CostmapClearer() : Node("costmap_cleaner")
    {
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1, std::bind(&CostmapClearer::initialPoseCallback, this, std::placeholders::_1));
            
        // ROS2 Nav2 清除代价地图服务名通常是 /<costmap_name>/clear_entirely_costmap
        // 这里创建两个客户端，用于清除 Nav2 的 local_costmap 和 global_costmap
        local_costmap_client_ = this->create_client<std_srvs::srv::Empty>("/local_costmap/clear_entirely_costmap");
        global_costmap_client_ = this->create_client<std_srvs::srv::Empty>("/global_costmap/clear_entirely_costmap");
        RCLCPP_INFO(this->get_logger(), "代价地图清除节点启动。将监听 /initialpose 并清除 Nav2 代价地图。");
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        (void)msg; // 避免未使用的参数警告
        
        clear_costmap(local_costmap_client_, "局部代价地图");
        clear_costmap(global_costmap_client_, "全局代价地图");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_costmap_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_costmap_client_;

    void clear_costmap(rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client, const std::string& map_name)
    {
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "%s 清除服务未上线，跳过清除。", map_name.c_str());
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        
        // 异步发送请求，并在回调中处理结果
        auto future_result = client->async_send_request(request, 
            [this, map_name](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
                try {
                    future.get(); // 检查是否有异常
                    RCLCPP_INFO(this->get_logger(), "成功清除 %s！", map_name.c_str());
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "%s 清除失败: %s", map_name.c_str(), e.what());
                }
            });
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapClearer>());
    rclcpp::shutdown();
    return 0;
}