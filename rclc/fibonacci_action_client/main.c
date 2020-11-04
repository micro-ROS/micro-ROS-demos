#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <example_interfaces/action/fibonacci.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_action_client_t action_client;
example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;
example_interfaces__action__Fibonacci_SendGoal_Response goal_response;
example_interfaces__action__Fibonacci_FeedbackMessage feedback;
example_interfaces__action__Fibonacci_GetResult_Response result_response;
int64_t goal_sequence_number;
int64_t result_sequence_number;

void goal_callback(const void * msg, rmw_request_id_t * request){
  printf("Goal callback\n");
  example_interfaces__action__Fibonacci_SendGoal_Response  * ros_msg = (example_interfaces__action__Fibonacci_SendGoal_Response * ) msg; 

  if (ros_msg->accepted) {
    printf("Goal request accepted\n");
    printf("Requesting response...\n");

    example_interfaces__action__Fibonacci_GetResult_Request ros_result_request;
    RCSOFTCHECK(rcl_action_send_result_request(&action_client, &ros_result_request, &result_sequence_number))
  }
}

void feedback_callback(const void * msg){

  example_interfaces__action__Fibonacci_FeedbackMessage  * ros_msg = (example_interfaces__action__Fibonacci_FeedbackMessage * ) msg; 

  printf("Feedback: [");
  for (size_t i = 0; i < ros_msg->feedback.sequence.size ; i++) {
    printf("%d, ", ros_msg->feedback.sequence.data[i]);
  }
  printf("]\n");
}

void result_callback(const void * msg, rmw_request_id_t * request){

  example_interfaces__action__Fibonacci_GetResult_Response  * ros_msg = (example_interfaces__action__Fibonacci_GetResult_Response * ) msg; 

  printf("Response: [");
  for (size_t i = 0; i < ros_msg->result.sequence.size ; i++) {
    printf("%d, ", ros_msg->result.sequence.data[i]);
  }
  printf("]\n");
}

int main()
{	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "fibonacci_action_client_rclc", "", &support));

	// create action client
	RCCHECK(rclc_action_client_init_default(
		&action_client,
		&node,
		ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
		"fibonacci"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

  RCCHECK(rclc_executor_add_action_client(
    &executor,
    &action_client,
    &goal_response,
    &feedback,
    &result_response,
    goal_callback,
    feedback_callback,
    result_callback));

  // Init messages
  ros_goal_request.goal.order = 10;

  example_interfaces__action__Fibonacci_Feedback__init(&feedback.feedback);
  rosidl_runtime_c__int32__Sequence__init(&feedback.feedback.sequence, ros_goal_request.goal.order);
  
  example_interfaces__action__Fibonacci_GetResult_Response__init(&result_response);
  rosidl_runtime_c__int32__Sequence__init(&result_response.result.sequence, ros_goal_request.goal.order);

  // Goal request
  RCCHECK(rcl_action_send_goal_request(&action_client, &ros_goal_request, &goal_sequence_number))

  rclc_executor_spin(&executor);

  RCCHECK(rcl_action_client_fini(&action_client, &node))
  RCCHECK(rcl_node_fini(&node));
}