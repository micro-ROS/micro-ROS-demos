#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <example_interfaces/action/fibonacci.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define RCCHECK(fn) rc = fn; if((rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); return 1;}
#define RCSOFTCHECK(fn) rc = fn; if((rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}

int main(int argc, const char * const * argv)
{
  rcl_ret_t rc;

  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context))

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "fibonacci_action_client_rcl", "", &context, &node_ops))

  const char * action_name = "fibonacci";
  const rosidl_action_type_support_t * action_type_support = ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci);
  rcl_action_client_t action_client = rcl_action_get_zero_initialized_client();
  rcl_action_client_options_t action_client_ops = rcl_action_client_get_default_options();

  RCCHECK(rcl_action_client_init(&action_client, &node, action_type_support, action_name, &action_client_ops))

  size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;

  RCCHECK(rcl_action_client_wait_set_get_num_entities(&action_client, &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services))
  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &context, rcl_get_default_allocator()))

  int64_t goal_sequence_number;
  int64_t result_sequence_number;

  bool done = false;
  bool goal_accepted = false;
  int order = 10;

  sleep(2); // Sleep a while to ensure DDS matching before sending request

  printf("Sending goal request\n");
  example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;
  ros_goal_request.goal.order = order;
  RCCHECK(rcl_action_send_goal_request(&action_client, &ros_goal_request, &goal_sequence_number))
  

  do {    
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
    size_t client_index, subscription_index;
    RCSOFTCHECK(rcl_action_wait_set_add_action_client(&wait_set, &action_client, &client_index, &subscription_index))

    RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(50)))

    bool is_feedback_ready = false;
    bool is_status_ready = false;
    bool is_goal_response_ready = false;
    bool is_cancel_response_ready = false;
    bool is_result_response_ready = false;

    RCSOFTCHECK(rcl_action_client_wait_set_get_entities_ready(&wait_set, &action_client, &is_feedback_ready, &is_status_ready, &is_goal_response_ready, &is_cancel_response_ready, &is_result_response_ready))

    if (is_goal_response_ready) {
      printf("Goal response ready\n");
      example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
      rmw_request_id_t response_header;

      RCSOFTCHECK(rcl_action_take_goal_response(&action_client, &response_header, &ros_goal_response))

      if (ros_goal_response.accepted) {
          printf("Goal request accepted\n");
          printf("Requesting response...\n");

          goal_accepted = true;

          example_interfaces__action__Fibonacci_GetResult_Request ros_result_request;
          RCSOFTCHECK(rcl_action_send_result_request(&action_client, &ros_result_request, &result_sequence_number))
      }
    }
    
    if (is_feedback_ready) {
      example_interfaces__action__Fibonacci_FeedbackMessage ros_feedback;

      ros_feedback.feedback.sequence.data = (int32_t*) malloc(order * sizeof(int32_t));
      ros_feedback.feedback.sequence.capacity = order;

      RCSOFTCHECK(rcl_action_take_feedback(&action_client, &ros_feedback))

      printf("Feedback: [");
      for (size_t i = 0; i < ros_feedback.feedback.sequence.size ; i++) {
        printf("%d, ", ros_feedback.feedback.sequence.data[i]);
      }
      printf("]\n");

      free(ros_feedback.feedback.sequence.data);
    } 
    
    if (is_result_response_ready) {
      example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;
      rmw_request_id_t response_header;

      ros_result_response.result.sequence.data = (int32_t*) malloc(order * sizeof(int32_t));
      ros_result_response.result.sequence.capacity = order;

      RCSOFTCHECK(rcl_action_take_result_response(&action_client, &response_header, &ros_result_response))

      printf("Response: [");
      for (size_t i = 0; i < ros_result_response.result.sequence.size ; i++) {
        printf("%d, ", ros_result_response.result.sequence.data[i]);
      }
      printf("]\n");

      done = true;
      free(ros_result_response.result.sequence.data);
    }
  } while ( !done );

  RCCHECK(rcl_action_server_fini(&action_client, &node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}

