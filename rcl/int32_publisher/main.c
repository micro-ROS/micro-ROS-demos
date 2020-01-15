#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <unistd.h>

int main(int argc, const char * const * argv)
{
  rcl_ret_t rv;

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
  rv = rcl_node_init(&node, "int32_publisher_rcl", "", &context, &node_ops);
  if (RCL_RET_OK != rv) {
    printf("Node initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  rv = rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", &publisher_ops);
  if (RCL_RET_OK != rv) {
    printf("Publisher initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  std_msgs__msg__Int32 msg;
  const int num_msg = 1000;
  msg.data = 0;
  
  
  sleep(2); // Sleep a while to ensure DDS matching before sending request

  do {
    rv = rcl_publish(&publisher, (const void*)&msg, NULL);
    if (RCL_RET_OK == rv )
    {
        printf("Sent: '%i'\n", msg.data++);
    }
  } while (RCL_RET_OK == rv && msg.data < num_msg );
  printf("TOTAL sent: %i\n", num_msg);

  rv = rcl_publisher_fini(&publisher, &node);
  rv = rcl_node_fini(&node);

  return 0;
}
