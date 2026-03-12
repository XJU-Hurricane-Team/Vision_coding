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

#include "tf_lookuptransform/tf_lookuptransform.hpp"

namespace TFListener{
    CoordinateGetter::CoordinateGetter(
        rclcpp::Node::SharedPtr node,
        const std::string& target_frame,
        const std::string& source_frame)
        :node_(node), target_frame_(target_frame), source_frame_(source_frame)
    {
        // 初始化tf缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    bool CoordinateGetter::getCurrentPose(
        geometry_msgs::msg::PoseStamped& pose,
        double timeout)
    {
        try{
            auto transform = tf_buffer_->lookupTransform(
                target_frame_,
                source_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(timeout));
            pose.header.stamp = node_->now();
            pose.header.frame_id = target_frame_;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;
            return true;
        } catch(const tf2::TransformException& ex){
            RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to get transform from %s to %s : %s",
                source_frame_.c_str(), target_frame_.c_str(), ex.what());
            return false;
        }
    }
} // namespace TFListener