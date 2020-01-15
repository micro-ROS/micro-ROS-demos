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


#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <geometry_msgs/msg/vector3.h>
#include <rmw_uros/options.h>

int main(int argc, char * argv[])
{
  rcl_ret_t ret;

  //Init RCL context
  rcl_context_t context;
  rcl_init_options_t init_options;

  init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());

  printf("Connecting to agent %s:%d\n",argv[1],atoi(argv[2]));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_udp_address(argv[1], argv[2], rmw_options);
  rmw_uros_options_set_client_key(0xCAFEBABE, rmw_options);

  context = rcl_get_zero_initialized_context();

  ret = rcl_init(0, NULL, &init_options, &context);

  //Init Node
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();

  ret = rcl_node_init(&node, "vector3_subscriber", "", &context, &node_ops);

  //Init Subscriber
  const char* topic_name = "sample_vector3";
  rcl_subscription_t subscriber_vector3 = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subs_ops = rcl_subscription_get_default_options();
  
  const rosidl_message_type_support_t * sub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
  
  ret = rcl_subscription_init(&subscriber_vector3, &node, sub_type_support, topic_name, &subs_ops);

  geometry_msgs__msg__Vector3 topic_data;
  geometry_msgs__msg__Vector3__init(&topic_data);

  // Spin
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());

  uint32_t timeout_ms = 500;

  while (rcl_context_is_valid(&context))
  {     
    // get empty wait set
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());

    // set rmw fields to NULL
    ret = rcl_wait_set_clear(&wait_set);

    size_t index = 0;
    ret = rcl_wait_set_add_subscription(&wait_set, &subscriber_vector3, &index);

    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(timeout_ms));				

    if (ret == RCL_RET_OK && wait_set.subscriptions[0]) {
        rmw_message_info_t messageInfo;
        ret = rcl_take(wait_set.subscriptions[0], &topic_data, &messageInfo, NULL);
        printf("Received: %f, %f, %f\n",topic_data.x,topic_data.y,topic_data.z);
    }

    rcl_wait_set_fini(&wait_set);
  }
  
  ret = rcl_node_fini(&node);

}
