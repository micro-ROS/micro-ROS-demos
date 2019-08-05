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
#include <std_msgs/msg/int32.hpp>

#include <memory>
#include <cstdio>

using std::placeholders::_1;

class int32_subscriber_node : public rclcpp::Node
{
public:
  int32_subscriber_node()
  : Node("int32_subscriber_cpp")
  {
    rclcpp::QoS qos(10);
    subscription_ = this->create_subscription<std_msgs::msg::Int32>("std_msgs_msg_Int32", qos,
        std::bind(&int32_subscriber_node::topic_callback, this, _1));
//    subscription_ = this->create_subscription<std_msgs::msg::Int32>("std_msgs_msg_Int32",
//        std::bind(&int32_subscriber_node::topic_callback, this, _1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::cout << "I heard: " << msg->data << std::endl;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<int32_subscriber_node>());
  rclcpp::shutdown();
  return 0;
}
