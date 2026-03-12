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

#include "rclcpp/rclcpp.hpp"
#include "tf_lookuptransform/tf_lookuptransform.hpp"

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_coordinate_publisher");
    auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);

    TFListener::CoordinateGetter getter(
        node, 
        "map",
        "base_footprint");
    
    rclcpp::Rate loop_rate(50); // 频率20Hz

    while(rclcpp::ok()){
        geometry_msgs::msg::PoseStamped pose;
        if(getter.getCurrentPose(pose)){
            RCLCPP_INFO(
                node->get_logger(),
                "current pose: x=%.2f, y=%.2f, z=%.2f",
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z);
            
            pose_publisher->publish(pose);  // 发布信息
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}