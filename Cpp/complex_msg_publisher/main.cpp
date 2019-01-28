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
#include <complex_msgs/msg/nested_msg_test.hpp>

#include <memory>
#include <chrono>
#include <cstdio>

using namespace std::chrono_literals;

class complex_msg_publisher_cpp_node : public rclcpp::Node
{
public:
  complex_msg_publisher_cpp_node()
  : Node("complex_msg_publisher_cpp")
  {
    publisher_ = this->create_publisher<complex_msgs::msg::NestedMsgTest>(
      "complex_msgs_msg_NestedMsgTest");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&complex_msg_publisher_cpp_node::timer_callback, this));
  }

private:
  void timer_callback()
  {
    msg.data1 = static_cast<bool>(num & 0x01);
    msg.data2 = static_cast<uint8_t>(num);
    msg.data3 = static_cast<signed char>(num);
    msg.data4 = static_cast<float>(num);
    msg.data5 = static_cast<double>(num);
    msg.data6 = static_cast<int8_t>(num);
    msg.data7 = static_cast<uint8_t>(num);
    msg.data8 = static_cast<int16_t>(num);
    msg.data9 = static_cast<uint16_t>(num);
    msg.data10 = static_cast<int32_t>(num);
    msg.data11 = static_cast<uint32_t>(num);
    msg.data12 = static_cast<int64_t>(num);
    msg.data13 = static_cast<uint64_t>(num);
    msg.data14.data1 = "Msg A - " + std::to_string(num);
    msg.data14.data2 = "Msg B - " + std::to_string(num);
    msg.data14.data3 = "Msg C - " + std::to_string(num);
    msg.data14.data4 = "Msg D - " + std::to_string(num);
    num++;

    publisher_->publish(msg);

    std::cout << "I send:" << std::endl;
    std::cout << "\tBool: " << std::to_string(msg.data1) << std::endl;
    std::cout << "\tuint8_t: " << std::to_string(msg.data2) << std::endl;
    std::cout << "\tsigned char: " << std::to_string(msg.data3) << std::endl;
    std::cout << "\tfloat: " << std::to_string(msg.data4) << std::endl;
    std::cout << "\tdouble: " << std::to_string(msg.data5) << std::endl;
    std::cout << "\tint8_t: " << std::to_string(msg.data6) << std::endl;
    std::cout << "\tuint8_t: " << std::to_string(msg.data7) << std::endl;
    std::cout << "\tint16_t: " << std::to_string(msg.data8) << std::endl;
    std::cout << "\tuint16_t: " << std::to_string(msg.data9) << std::endl;
    std::cout << "\tint32_t: " << std::to_string(msg.data10) << std::endl;
    std::cout << "\tuint32_t: " << std::to_string(msg.data11) << std::endl;
    std::cout << "\tint64_t: " << std::to_string(msg.data12) << std::endl;
    std::cout << "\tuint64_t: " << std::to_string(msg.data13) << std::endl;

    std::cout << "\tstring 1: " << msg.data14.data1 << std::endl;
    std::cout << "\tstring 2: " << msg.data14.data2 << std::endl;
    std::cout << "\tstring 3: " << msg.data14.data3 << std::endl;
    std::cout << "\tstring 4: " << msg.data14.data4 << std::endl;
    std::cout << "\n" << std::endl;
  }

  rclcpp::Publisher<complex_msgs::msg::NestedMsgTest>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  complex_msgs::msg::NestedMsgTest msg;
  size_t num = 0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<complex_msg_publisher_cpp_node>());
  rclcpp::shutdown();
  return 0;
}
