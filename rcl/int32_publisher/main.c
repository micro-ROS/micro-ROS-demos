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

  rcl_publisher_t publisher;
  rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", NULL);
  if (RCL_RET_OK != rv) {
    printf("Publisher initialization error\n");
    return 1;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rv = rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 0, 0, NULL, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("Wait set initialization error\n");
    return 1;
  }

  std_msgs__msg__Int32 msg;
  msg.data = -1;
  do {
    printf("Sending: '%i'\n", msg.data++);
    rv &= rcl_publish(&publisher, (const void*)&msg, NULL);
    rv &= rcl_wait(&wait_set, 500);
  } while (rv);

  rv = rcl_publisher_fini(&publisher, &node);
  rv = rcl_node_fini(&node);

  return 0;
}
