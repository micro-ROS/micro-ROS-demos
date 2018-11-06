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

#include <chrono>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using namespace std::chrono_literals;

class int32_publisher_node : public rclcpp::Node
{
public:
  int32_publisher_node() : Node("int32_publisher_cpp")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("std_msgs_msg_Int32");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&int32_publisher_node::timer_callback, this));
  }

private:
  void timer_callback()
  {
    count_.data++;
    publisher_->publish(count_);
    std::cout << count_.data << std::endl;
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::Int32 count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<int32_publisher_node>());
  rclcpp::shutdown();
  return 0;
}
