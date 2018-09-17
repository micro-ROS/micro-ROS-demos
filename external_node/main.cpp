// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class ExternalNode : public rclcpp::Node
{
public:
  ExternalNode() : Node("external_node")
  {
    power_publisher_ = this->create_publisher<std_msgs::msg::Float64>("engine_power");
    
    altitude_subscription_ = this->create_subscription<std_msgs::msg::String>("status_report", std::bind(&ExternalNode::topic_callback, this, _1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr power_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr altitude_subscription_;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (strcmp(msg->data.c_str(), "Failure!!") == 0)
        {
          //std_msgs__msg__Float64 message;
          auto message = std_msgs::msg::Float64();
          message.data = 10;
          this->power_publisher_->publish(message);
        }
        else if (strcmp(msg->data.c_str(), "Failure!!") == 0)
        {
          auto message = std_msgs::msg::Float64();
          message.data = 5;
          this->power_publisher_->publish(message);
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Unkwnow status: '%s'", msg->data.c_str())
        }  
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExternalNode>());
  rclcpp::shutdown();
  return 0;
}
