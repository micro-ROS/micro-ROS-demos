#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}

int main(int argc, const char * const * argv)
{
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context))

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  // Creating first node
  rcl_node_t node1 = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node1, "int32_node_1", "", &context, &node_ops))

  rcl_publisher_t publisher1 = rcl_get_zero_initialized_publisher();
  RCCHECK(rcl_publisher_init(&publisher1, &node1, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "node_1_to_2", &publisher_ops))

  rcl_subscription_t subscription1 = rcl_get_zero_initialized_subscription();
  RCCHECK(rcl_subscription_init(&subscription1, &node1, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "node_2_to_1", &subscription_ops))

  // Creating second node
  rcl_node_t node2 = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node2, "int32_node_2", "", &context, &node_ops))

  rcl_publisher_t publisher2 = rcl_get_zero_initialized_publisher();
  RCCHECK(rcl_publisher_init(&publisher2, &node2, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "node_2_to_1", &publisher_ops))

  rcl_subscription_t subscription2 = rcl_get_zero_initialized_subscription();
  RCCHECK(rcl_subscription_init(&subscription2, &node2, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "node_1_to_2", &subscription_ops))

  // Shared wait set
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 2, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

  std_msgs__msg__Int32 msg1, msg2;

  msg1.data = -100;
  msg2.data = 100;

  sleep(2); // Sleep a while to ensure DDS matching before sending request

  uint32_t count = 0;

  do {
    if (count % 1000 == 0){
      // Publish data on each node
      msg1.data++;
      RCSOFTCHECK(rcl_publish(&publisher1, (const void*)&msg1, NULL))

      msg2.data--;
      RCSOFTCHECK(rcl_publish(&publisher2, (const void*)&msg2, NULL))
    }
    
    // Add both subscritions to same wait set
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))

    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription1, NULL))

    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription2, NULL))

    RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(1)))

    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i]) { 
        std_msgs__msg__Int32 received_msg;

        rcl_ret_t rc = rcl_take(wait_set.subscriptions[i], &received_msg, NULL, NULL);
        
        if (RCL_RET_OK == rc) {
          printf("Node %ld has received: [%d]\n", i, received_msg.data);
        }
      }
    }

    count++;
    usleep(1000);
  } while (true);

  RCCHECK(rcl_publisher_fini(&publisher1, &node1))
  RCCHECK(rcl_publisher_fini(&publisher2, &node2))
  RCCHECK(rcl_subscription_fini(&subscription1, &node1))
  RCCHECK(rcl_subscription_fini(&subscription2, &node2))
  RCCHECK(rcl_node_fini(&node1))
  RCCHECK(rcl_node_fini(&node2))

  return 0;
}
