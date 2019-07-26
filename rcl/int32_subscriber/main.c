#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>

int main(int argc, const char * const * argv)
{
  rcl_ret_t rv;

  rv = rcl_init(argc, argv, NULL, NULL);
  if (RCL_RET_OK != rv) {
    printf("rcl initialization error\n");
    return 1;
  }

  rcl_node_t node;
  rv = rcl_node_fini(&node);
  if (RCL_RET_OK != rv) {
    printf("Node initialization error\n");
    return 1;
  }

  rcl_subscription_t subscription;
  rv = rcl_subscription_init(
    &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", NULL);
  if (RCL_RET_OK != rv) {
    printf("Subscription initialization error\n");
    return 1;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rv = rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 0, 0, NULL, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("Wait set initialization error\n");
    return 1;
  }

  rv = rcl_wait_set_clear(&wait_set);
  if (RCL_RET_OK != rv) {
    printf("Wait set clear error\n");
    return 1;
  }

  rv = rcl_wait_set_add_subscription(&wait_set, &subscription, NULL);
  if (RCL_RET_OK != rv) {
    printf("Wait set add subscription error\n");
    return 1;
  }

  void* msg = rcl_get_default_allocator().zero_allocate(sizeof(std_msgs__msg__Int32), 1, rcl_get_default_allocator().state);
  do {
    rv &= rcl_wait(&wait_set, -1);
    for (size_t i = 0; i < wait_set.size_of_subscriptions; ++i) {
      rv = rcl_take(wait_set.subscriptions[i], msg, NULL, NULL);
      if (RCL_RET_OK == rv)
      {
        printf("I received: [%i]\n", ((const std_msgs__msg__Int32*)msg)->data);
      }
    }
  } while (rv);

  rv = rcl_subscription_fini(&subscription, &node);
  rv = rcl_node_fini(&node);

  return 0;
}
