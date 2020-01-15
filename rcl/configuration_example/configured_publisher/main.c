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

int main(int argc, char * const argv[])
{
  //Init RCL context
  rcl_context_t context;
  rcl_init_options_t init_options;

  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, rcl_get_default_allocator()))
  
  printf("Connecting to agent %s:%d\n",argv[1],atoi(argv[2]));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  RCCHECK(rmw_uros_options_set_udp_address(argv[1], argv[2], rmw_options))
  RCCHECK(rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options))

  context = rcl_get_zero_initialized_context();

  RCCHECK(rcl_init(0, NULL, &init_options, &context))

  //Init Node
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();

  RCCHECK(rcl_node_init(&node, "vector3_publisher", "", &context, &node_ops))

  //Init Publisher
  const char* topic_name = "sample_vector3";

  rcl_publisher_t publisher_vector3 = rcl_get_zero_initialized_publisher();
  const rosidl_message_type_support_t * pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
  rcl_publisher_options_t pub_opt = rcl_publisher_get_default_options();

  RCCHECK(rcl_publisher_init(&publisher_vector3, &node, pub_type_support, topic_name, &pub_opt))

  geometry_msgs__msg__Vector3 topic_data;
  geometry_msgs__msg__Vector3__init(&topic_data);

  // Spin
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

  float values[3];
  values[0] = 0.0;
  values[1] = 1.0;
  values[2] = 2.0;

  topic_data.x = values[0];
  topic_data.y = values[1];
  topic_data.z = values[2];

  while (rcl_context_is_valid(&context)) {     
    RCSOFTCHECK(rcl_publish( &publisher_vector3, (const void *) &topic_data, NULL))
    
    for(uint8_t i = 0; i < 3; i++) {
      values[i]++;
    }

    topic_data.x = values[0];
    topic_data.y = values[1];
    topic_data.z = values[2];

    printf("Publish: %f, %f, %f\n",topic_data.x,topic_data.y,topic_data.z);
    sleep(1);
  }
  
  RCCHECK(rcl_node_fini(&node))
}
