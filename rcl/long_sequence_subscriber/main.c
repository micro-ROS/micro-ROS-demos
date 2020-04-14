#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/string.h>

#include <rmw_uros/options.h>

#include <stdio.h>

#define ARRAY_LEN 4096

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
  RCCHECK(rcl_node_init(&node, "char_long_sequence_subscriber_rcl", "", &context, &node_ops))

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  RCCHECK(rcl_subscription_init(&subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/char_long_sequence", &subscription_ops))

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator()))

  std_msgs__msg__String msg;

  msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = ARRAY_LEN;
  
  char test_array[ARRAY_LEN];
  
  do {
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
    size_t index;
    RCSOFTCHECK(rcl_wait_set_add_subscription(&wait_set, &subscription, &index))
    
    RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(1)))

    if (wait_set.subscriptions[index]) {


      rcl_ret_t rc = rcl_take(wait_set.subscriptions[index], &msg, NULL, NULL);
      if (RCL_RET_OK == rc) {
        
        memset(test_array,'z',msg.data.size);
        test_array[msg.data.size] = '\0';
        
        // Check if sequence items matches the know pattern
        bool pass_test = srtcmp(test_array, msg.data.data) == 0;

        printf("I received an %ld array. Test: [%s]\n", msg.data.size, (pass_test) ? "OK" : "FAIL");
      }
    }
  } while ( true );

  RCCHECK(rcl_subscription_fini(&subscription, &node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}
