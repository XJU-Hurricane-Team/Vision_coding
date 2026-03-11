#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>
#include <cmath>
#include <deque>

using std::placeholders::_1;

class LidarLoc : public rclcpp::Node
{
public:
    LidarLoc() : Node("lidar_loc")
    {
        // 声明并获取参数
        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("laser_frame", "laser");
        this->declare_parameter<std::string>("laser_topic", "scan");

        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("laser_frame", laser_frame_);
        this->get_parameter("laser_topic", laser_topic_);

        // 初始化 TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // 订阅和客户端
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 1, std::bind(&LidarLoc::mapCallback, this, _1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_, 1, std::bind(&LidarLoc::scanCallback, this, _1));
            
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 1, std::bind(&LidarLoc::initialPoseCallback, this, _1));

        // ROS2 Nav2 清除代价地图服务名通常是 /<costmap_name>/clear_entirely_costmap
        // 这里创建客户端来清除 Nav2 的 local_costmap (当收到 initialpose 后)
        clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>("/local_costmap/clear_entirely_costmap");

        // 定时器替代主循环
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&LidarLoc::poseTfTimer, this));
        
        lidar_x = 250; lidar_y = 250; lidar_yaw = 0;
        cur_sum = 0;
        clear_countdown = -1;
        scan_count = 0;
    }

private:
    std::string base_frame_, odom_frame_, laser_frame_, laser_topic_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid map_msg;
    cv::Mat map_cropped;
    cv::Mat map_temp;
    sensor_msgs::msg::RegionOfInterest map_roi_info;
    std::vector<cv::Point2f> scan_points;
    
    float lidar_x, lidar_y, lidar_yaw;
    const float deg_to_rad = M_PI / 180.0;
    int cur_sum;
    int clear_countdown;
    int scan_count;
    
    // 队列用于 check 函数
    std::deque<std::tuple<float, float, float>> data_queue;
    const size_t max_size = 10;

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        double map_x = msg->pose.pose.position.x;
        double map_y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (map_msg.info.resolution <= 0) {
            RCLCPP_ERROR(this->get_logger(), "地图信息无效或未接收");
            return;
        }

        lidar_x = (map_x - map_msg.info.origin.position.x) / map_msg.info.resolution - map_roi_info.x_offset;
        lidar_y = (map_y - map_msg.info.origin.position.y) / map_msg.info.resolution - map_roi_info.y_offset;
        lidar_yaw = -yaw;
        clear_countdown = 30;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_msg = *msg;
        crop_map();
        processMap();        
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_points.clear();
        double angle = msg->angle_min;
        geometry_msgs::msg::TransformStamped transformStamped;

        try {
            // ROS 2 tf2::TimePointZero for latest transform
            transformStamped = tf_buffer_->lookupTransform(base_frame_, laser_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        tf2::Quaternion q_lidar;
        tf2::fromMsg(transformStamped.transform.rotation, q_lidar);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);
        const double tolerance = 0.1;
        bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
        // 修复布尔乘法警告，改用逻辑与
        bool pitch_check = !(std::abs(std::abs(pitch) - M_PI) < tolerance);
        lidar_is_inverted = lidar_is_inverted && pitch_check;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
            {
                float x_laser = msg->ranges[i] * cos(angle);
                float y_laser = -msg->ranges[i] * sin(angle);

                geometry_msgs::msg::PointStamped point_laser;
                point_laser.point.x = x_laser;
                point_laser.point.y = y_laser;
                point_laser.point.z = 0.0;

                geometry_msgs::msg::PointStamped point_base;
                tf2::doTransform(point_laser, point_base, transformStamped);

                float x = point_base.point.x / map_msg.info.resolution;
                float y = point_base.point.y / map_msg.info.resolution;
                if (lidar_is_inverted) { x = -x; y = -y; }

                scan_points.push_back(cv::Point2f(x, y));
            }
            angle += msg->angle_increment;
        }
        if(scan_count == 0) scan_count++;

        // 核心定位算法
        run_matching();

        if(clear_countdown > -1) clear_countdown--;
        if(clear_countdown == 0)
        {
            if (!clear_costmaps_client_->wait_for_service(std::chrono::milliseconds(100))) {
                 RCLCPP_WARN(this->get_logger(), "清除代价地图服务未上线，跳过清除。");
            } else {
                auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                // 异步调用服务
                clear_costmaps_client_->async_send_request(request);
            }
        }
    }

    void run_matching()
    {
        if (map_cropped.empty()) return;
        
        // 计算三种情况下的雷达点坐标数组
        std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;
        int max_sum = 0;
        float best_dx = 0, best_dy = 0, best_dyaw = 0;

        for (const auto& point : scan_points)
        {
            // 情况一：原始角度
            float rotated_x = point.x * cos(lidar_yaw) - point.y * sin(lidar_yaw);
            float rotated_y = point.x * sin(lidar_yaw) + point.y * cos(lidar_yaw);
            transform_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));

            // 情况二：顺时针旋转1度
            float clockwise_yaw = lidar_yaw + deg_to_rad;
            rotated_x = point.x * cos(clockwise_yaw) - point.y * sin(clockwise_yaw);
            rotated_y = point.x * sin(clockwise_yaw) + point.y * cos(clockwise_yaw);
            clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));

            // 情况三：逆时针旋转1度
            float counter_yaw = lidar_yaw - deg_to_rad;
            rotated_x = point.x * cos(counter_yaw) - point.y * sin(counter_yaw);
            rotated_y = point.x * sin(counter_yaw) + point.y * cos(counter_yaw);
            counter_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));
        }

        // 计算15种变换方式的匹配值
        std::vector<cv::Point2f> offsets = {{0,0}, {1,0}, {-1,0}, {0,1}, {0,-1}};
        std::vector<std::vector<cv::Point2f>> point_sets = {transform_points, clockwise_points, counter_points};
        std::vector<float> yaw_offsets = {0, deg_to_rad, -deg_to_rad};

        for (size_t i = 0; i < offsets.size(); ++i)
        {
            for (size_t j = 0; j < point_sets.size(); ++j)
            {
                int sum = 0;
                for (const auto& point : point_sets[j])
                {
                    float px = point.x + offsets[i].x;
                    float py = point.y + offsets[i].y;
                    if (px >= 0 && px < map_temp.cols && py >= 0 && py < map_temp.rows)
                    {
                        sum += map_temp.at<uchar>(py, px);
                    }
                }
                if (sum > max_sum)
                {
                    max_sum = sum;
                    best_dx = offsets[i].x;
                    best_dy = offsets[i].y;
                    best_dyaw = yaw_offsets[j];
                }
            }
        }

        lidar_x += best_dx;
        lidar_y += best_dy;
        lidar_yaw += best_dyaw;
        
        check(lidar_x , lidar_y , lidar_yaw);
    }

    void crop_map()
    {
        nav_msgs::msg::MapMetaData info = map_msg.info;
        int xMax,xMin,yMax,yMin ;
        xMax=xMin= info.width/2;
        yMax=yMin= info.height/2;
        bool bFirstPoint = true;

        cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128));

        for(unsigned int y = 0; y < info.height; y++)
        {
            for(unsigned int x = 0; x < info.width; x++)
            {
                int index = y * info.width + x;
                int map_value = map_msg.data[index];
                if (map_value == -1) {
                    map_raw.at<uchar>(y, x) = 128; // Unknown/Unoccupied
                } else if (map_value == 100) {
                    map_raw.at<uchar>(y, x) = 100; // Occupied
                } else {
                    map_raw.at<uchar>(y, x) = 0;   // Free
                }

                if(map_msg.data[index] == 100)
                {
                    if(bFirstPoint) { xMax = xMin = x; yMax = yMin = y; bFirstPoint = false; continue; }
                    xMin = std::min(xMin, (int)x); xMax = std::max(xMax, (int)x);
                    yMin = std::min(yMin, (int)y); yMax = std::max(yMax, (int)y);
                }
            }
        }
        int cen_x = (xMin + xMax)/2;
        int cen_y = (yMin + yMax)/2;
        int new_half_width = abs(xMax - xMin)/2 + 50;
        int new_half_height = abs(yMax - yMin)/2 + 50;
        int new_origin_x = cen_x - new_half_width;
        int new_origin_y = cen_y - new_half_height;
        int new_width = new_half_width*2;
        int new_height = new_half_height*2;

        if(new_origin_x < 0) new_origin_x = 0;
        if((unsigned int)(new_origin_x + new_width) > info.width) new_width = info.width - new_origin_x;
        if(new_origin_y < 0) new_origin_y = 0;
        if((unsigned int)(new_origin_y + new_height) > info.height) new_height = info.height - new_origin_y;

        cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
        map_cropped = map_raw(roi).clone();

        map_roi_info.x_offset = new_origin_x;
        map_roi_info.y_offset = new_origin_y;
        map_roi_info.width = new_width;
        map_roi_info.height = new_height;
        
        auto init_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        init_pose->pose.pose.orientation.w = 1.0;
        initialPoseCallback(init_pose);
    }

    bool check(float x, float y, float yaw)
    {
        if(x == 0 && y == 0 && yaw == 0) { data_queue.clear(); return true; }
        data_queue.push_back(std::make_tuple(x, y, yaw));
        if (data_queue.size() > max_size) data_queue.pop_front();
        if (data_queue.size() == max_size) 
        {
            auto& first = data_queue.front();
            auto& last = data_queue.back();
            float dx = std::abs(std::get<0>(last) - std::get<0>(first));
            float dy = std::abs(std::get<1>(last) - std::get<1>(first));
            float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));
            if (dx < 5 && dy < 5 && dyaw < 5*deg_to_rad) { data_queue.clear(); return true; }
        }
        return false;
    }

    cv::Mat createGradientMask(int size)
    {
        cv::Mat mask(size, size, CV_8UC1);
        int center = size / 2;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                double distance = std::hypot(x - center, y - center);
                int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
                mask.at<uchar>(y, x) = value;
            }
        }
        return mask;
    }

    void processMap()
    {
        if (map_cropped.empty()) return;
        map_temp = cv::Mat::zeros(map_cropped.size(), CV_8UC1);
        cv::Mat gradient_mask = createGradientMask(101);

        for (int y = 0; y < map_cropped.rows; y++) {
            for (int x = 0; x < map_cropped.cols; x++) {
                if (map_cropped.at<uchar>(y, x) == 100) {
                    int left = std::max(0, x - 50);
                    int top = std::max(0, y - 50);
                    int right = std::min(map_cropped.cols - 1, x + 50);
                    int bottom = std::min(map_cropped.rows - 1, y + 50);
                    cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                    cv::Mat region = map_temp(roi);
                    int mask_left = 50 - (x - left);
                    int mask_top = 50 - (y - top);
                    cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                    cv::Mat mask = gradient_mask(mask_roi);
                    cv::max(region, mask, region);
                }
            }
        }
    }

    void poseTfTimer()
    {
        if (scan_count == 0) return;
        if (map_cropped.empty() || map_msg.data.empty() || map_msg.info.resolution <= 0) return;

        double full_map_pixel_x = lidar_x + map_roi_info.x_offset;
        double full_map_pixel_y = lidar_y + map_roi_info.y_offset;
        double x_in_map_frame = full_map_pixel_x * map_msg.info.resolution + map_msg.info.origin.position.x;
        double y_in_map_frame = full_map_pixel_y * map_msg.info.resolution + map_msg.info.origin.position.y;
        double yaw_in_map_frame = -lidar_yaw;

        tf2::Transform map_to_base;
        map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_in_map_frame);
        map_to_base.setRotation(q);

        geometry_msgs::msg::TransformStamped odom_to_base_msg;
        try {
            odom_to_base_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
            return;
        }

        tf2::Transform odom_to_base_tf2;
        tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf2);
        tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

        geometry_msgs::msg::TransformStamped map_to_odom_msg;
        map_to_odom_msg.header.stamp = this->now();
        map_to_odom_msg.header.frame_id = "map";
        map_to_odom_msg.child_frame_id = odom_frame_;
        map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

        tf_broadcaster_->sendTransform(map_to_odom_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarLoc>());
    rclcpp::shutdown();
    return 0;
}