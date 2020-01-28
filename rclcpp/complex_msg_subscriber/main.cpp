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

#include <complex_msgs/msg/nested_msg_test.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <cstdio>

using std::placeholders::_1;

class complex_msg_subscriber_cpp_node : public rclcpp::Node
{
public:
  complex_msg_subscriber_cpp_node()
  : Node("complex_msg_subscriber_cpp")
  {
    subscription_ = this->create_subscription<complex_msgs::msg::NestedMsgTest>(
      "complex_msgs_msg_NestedMsgTest", rclcpp::SystemDefaultsQoS(),
      std::bind(&complex_msg_subscriber_cpp_node::topic_callback, this, _1));
  }

private:
  rclcpp::Subscription<complex_msgs::msg::NestedMsgTest>::SharedPtr subscription_;

  void topic_callback(const complex_msgs::msg::NestedMsgTest::SharedPtr msg)
  {
    std::cout << "I heard: " << std::endl;
    std::cout << "\tBool: " << std::to_string(msg->data1) << std::endl;
    std::cout << "\tuint8_t: " << std::to_string(msg->data2) << std::endl;
    std::cout << "\tsigned char: " << std::to_string(msg->data3) << std::endl;
    std::cout << "\tfloat: " << std::to_string(msg->data4) << std::endl;
    std::cout << "\tdouble: " << std::to_string(msg->data5) << std::endl;
    std::cout << "\tint8_t: " << std::to_string(msg->data6) << std::endl;
    std::cout << "\tuint8_t: " << std::to_string(msg->data7) << std::endl;
    std::cout << "\tint16_t: " << std::to_string(msg->data8) << std::endl;
    std::cout << "\tuint16_t: " << std::to_string(msg->data9) << std::endl;
    std::cout << "\tint32_t: " << std::to_string(msg->data10) << std::endl;
    std::cout << "\tuint32_t: " << std::to_string(msg->data11) << std::endl;
    std::cout << "\tint64_t: " << std::to_string(msg->data12) << std::endl;
    std::cout << "\tuint64_t: " << std::to_string(msg->data13) << std::endl;

    std::cout << "\tstring 1: " << msg->data14.data1 << std::endl;
    std::cout << "\tstring 2: " << msg->data14.data2 << std::endl;
    std::cout << "\tstring 3: " << msg->data14.data3 << std::endl;
    std::cout << "\tstring 4: " << msg->data14.data4 << std::endl;
    std::cout << "\n" << std::endl;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<complex_msg_subscriber_cpp_node>());
  rclcpp::shutdown();
  return 0;
}
