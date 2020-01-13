#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>

int main(int argc, const char * const * argv)
{
  rcl_ret_t rv = RCL_RET_ERROR;

  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rv = rcl_init_options_init(&options, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("rcl init options error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_context_t context = rcl_get_zero_initialized_context();
  rv = rcl_init(argc, argv, &options, &context);
  if (RCL_RET_OK != rv) {
    printf("rcl initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rv = rcl_node_init(&node, "int32_subscriber_rcl", "", &context, &node_ops);
  if (RCL_RET_OK != rv)
  {
    fprintf(stderr, "[main] error in rcl : %s\n", rcutils_get_error_string().str);
    rcl_reset_error();
    return 1;
  }

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  rv = rcl_subscription_init(
    &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &subscription_ops);
  if (RCL_RET_OK != rv) {
    printf("Subscription initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rv = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("Wait set initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  void* msg = rcl_get_default_allocator().zero_allocate(sizeof(std_msgs__msg__Int32), 1, rcl_get_default_allocator().state);
  do {
    rv = rcl_wait_set_clear(&wait_set);
    if (RCL_RET_OK != rv) {
      printf("Wait set clear error: %s\n", rcl_get_error_string().str);
      break;
    }
    
    size_t index;
    rv = rcl_wait_set_add_subscription(&wait_set, &subscription, &index);
    if (RCL_RET_OK != rv) {
      printf("Wait set add subscription error: %s\n", rcl_get_error_string().str);
      break;
    }    
    
    rv = rcl_wait(&wait_set, 1000000);
    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      if (wait_set.subscriptions[i])
      {
        rv = rcl_take(wait_set.subscriptions[i], msg, NULL, NULL);
        if (RCL_RET_OK == rv)
        {
          printf("I received: [%i]\n", ((const std_msgs__msg__Int32*)msg)->data);
        }
      }
    }
  } while ( true );

  rv = rcl_subscription_fini(&subscription, &node);
  rv = rcl_node_fini(&node);

  return 0;
}
