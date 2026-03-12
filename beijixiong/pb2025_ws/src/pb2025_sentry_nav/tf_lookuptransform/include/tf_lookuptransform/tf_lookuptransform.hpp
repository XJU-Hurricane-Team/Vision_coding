// Auther: Ziyan Ye
// Time: 2025.4.25

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace TFListener{
    class CoordinateGetter{
        public:
            CoordinateGetter(
                rclcpp::Node::SharedPtr node,
                const std::string& target_frame,
                const std::string& source_frame);
            
            bool getCurrentPose(geometry_msgs::msg::PoseStamped& pose, double timeout=1.0); //判断是否成功获取
        
        private:
            rclcpp::Node::SharedPtr node_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            std::string target_frame_;
            std::string source_frame_;
    };
} // namespace TFListener
