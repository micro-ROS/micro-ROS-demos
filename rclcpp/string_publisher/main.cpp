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

#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

class string_publisher_cpp_node : public rclcpp::Node
{
public:
  string_publisher_cpp_node()
  : Node("string_publisher_cpp")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("std_msgs_msg_String");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&string_publisher_cpp_node::timer_callback, this));
  }

private:
  void timer_callback()
  {
    msg.data = "Hello World " + std::to_string(num++);
    publisher_->publish(msg);
    std::cout << msg.data << std::endl;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::String msg;
  size_t num = 0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<string_publisher_cpp_node>());
  rclcpp::shutdown();
  return 0;
}
