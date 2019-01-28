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

#include <rclc/rclc.h>
#include <complex_msgs/msg/nested_msg_test.h>

#include <stdio.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) {return -1;}

void on_message(const void * msgin)
{
  const complex_msgs__msg__NestedMsgTest * msg = (const complex_msgs__msg__NestedMsgTest *)msgin;
  printf("I heard:\n");
  printf("\tBool: %u\n", msg->data1);
  printf("\tuint8_t: %u\n", msg->data2);
  printf("\tsigned char: %u\n", msg->data3);
  printf("\tfloat: %f\n", msg->data4);
  printf("\tdouble: %lf\n", msg->data5);
  printf("\tint8_t: %i\n", msg->data6);
  printf("\tuint8_t: %u\n", msg->data7);
  printf("\tint16_t: %i\n", msg->data8);
  printf("\tuint16_t: %u\n", msg->data9);
  printf("\tint32_t: %i\n", msg->data10);
  printf("\tuint32_t: %u\n", msg->data11);
  printf("\tint64_t: %li\n", msg->data12);
  printf("\tuint64_t: %lu\n", msg->data13);

  printf("\tstring 1: %s\n", msg->data14.data1.data);
  printf("\tstring 2: %s\n", msg->data14.data2.data);
  printf("\tstring 3: %s\n", msg->data14.data3.data);
  printf("\tstring 4: %s\n", msg->data14.data4.data);
  printf("\n\n");
}

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  rclc_ret_t ret;

  ret = rclc_init(0, NULL);
  if (ret != RCL_RET_OK) {
    return -1;
  }
  rclc_node_t * node = rclc_create_node("complex_msg_subscriber_c", "");
  CUSTOM_ASSERT(node);
  rclc_subscription_t * sub =
    rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(complex_msgs, msg,
      NestedMsgTest), "complex_msgs_msg_NestedMsgTest", on_message, 1,
      false);
  CUSTOM_ASSERT(sub);

  rclc_spin_node(node);

  ret = rclc_destroy_subscription(sub);
  ret = rclc_destroy_node(node);
  return 0;
}
