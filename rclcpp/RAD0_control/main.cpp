// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class ExternalNode : public rclcpp::Node
{
public:
  ExternalNode()
  : Node("rad0_control_cpp")
  {
    rclcpp::QoS qos(10);
    engine_power_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
      "std_msgs_msg_Int32", qos);
    warn_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "std_msgs_msg_String", qos);

    altitude_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "std_msgs_msg_Float64", qos, std::bind(&ExternalNode::topic_callback, this, _1));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr engine_power_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warn_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr altitude_subscription_;

  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Warning
    if (msg->data <= 500) {
      {
        auto message = std_msgs::msg::String();
        message.data = "Failure!!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->warn_publisher_->publish(message);
      }

      {
        auto message = std_msgs::msg::Int32();
        message.data = 10;
        this->engine_power_publisher_->publish(message);
      }
      // Failure
    } else if (msg->data <= 1000) {
      {
        auto message = std_msgs::msg::String();
        message.data = "Warning!!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->warn_publisher_->publish(message);
      }

      {
        auto message = std_msgs::msg::Int32();
        message.data = 5;
        this->engine_power_publisher_->publish(message);
      }
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
