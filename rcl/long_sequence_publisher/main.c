#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/string.h>

#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>

#define ARRAY_CAP 800
#define ARRAY_LEN 600

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

int main(int argc, const char * const * argv)
{
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context))

  rcl_node_options_t node_ops = rcl_node_get_default_options();

  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "char_long_sequence_publisher_rcl", "", &context, &node_ops))

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
  rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
  RCCHECK(rcl_publisher_init(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/char_long_sequence", &publisher_ops))
  
  std_msgs__msg__String msg;
  msg.data.data = (char * ) malloc(ARRAY_CAP * sizeof(char));
  msg.data.size = ARRAY_LEN;
  msg.data.capacity = ARRAY_CAP;

  // Fill the array with a known sequence
  for (size_t i = 0; i < ARRAY_LEN-1; i++){
    msg.data.data[i] = (char) 'z';
  }
  msg.data.data[ARRAY_LEN-1] = '\0';

  for (size_t i = 0; i < ARRAY_LEN; i++){
    printf("%c ", msg.data.data[i]);
  }
  printf("\n");
  
  sleep(2); // Sleep a while to ensure DDS matching before sending request

  rcl_ret_t rc;
  do {
    rc = rcl_publish(&publisher, (const void*)&msg, NULL);
    if (RCL_RET_OK == rc ) {
        printf("Sent %ld array\n", msg.data.size);
    }else{
        printf("Publishing error %d\n", rc);
    }
    sleep(1);
  } while (true);
  
  RCCHECK(rcl_publisher_fini(&publisher, &node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}
