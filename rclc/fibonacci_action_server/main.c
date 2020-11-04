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

bool processing_goal = false;
bool result_requested = false;
pthread_t processing_thread;
pthread_mutex_t lock;

rcl_action_server_t action_server;
example_interfaces__action__Fibonacci_SendGoal_Request goal_request;
example_interfaces__action__Fibonacci_GetResult_Request result_request;

rcl_action_goal_info_t goal_info;
rcl_action_goal_handle_t * goal_handle;
rmw_request_id_t request_header;
example_interfaces__action__Fibonacci_FeedbackMessage ros_goal_feedback; 

void * fibonacci(void *args){
  int32_t order = *((int32_t *)args);

  example_interfaces__action__Fibonacci_Feedback__init(&ros_goal_feedback.feedback);
  rosidl_runtime_c__int32__Sequence__init(&ros_goal_feedback.feedback.sequence, order);

  ros_goal_feedback.goal_id = goal_info.goal_id;
  ros_goal_feedback.feedback.sequence.data[0] = 0;
  ros_goal_feedback.feedback.sequence.data[1] = 1;
  ros_goal_feedback.feedback.sequence.size = 2;

  for (uint32_t i = 2; i < order; i++) {
    ros_goal_feedback.feedback.sequence.size++;
    ros_goal_feedback.feedback.sequence.data[i] = 
        ros_goal_feedback.feedback.sequence.data[i-1] + 
        ros_goal_feedback.feedback.sequence.data[i-2];


    if (result_requested)
    {
      pthread_mutex_lock(&lock);
      printf("Sending feedback\n");
      RCSOFTCHECK(rcl_action_publish_feedback(&action_server, &ros_goal_feedback));
      pthread_mutex_unlock(&lock);
    }
    usleep(200000);
  }

  RCSOFTCHECK(rcl_action_update_goal_state(goal_handle, GOAL_EVENT_SUCCEED));
  RCSOFTCHECK(rcl_action_notify_goal_done(&action_server));

  while(!result_requested){
    usleep(1000);
  }
  result_requested = false;

  example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;
  example_interfaces__action__Fibonacci_Result result;
  result.sequence.capacity = order;
  result.sequence.size = order;
  result.sequence.data = ros_goal_feedback.feedback.sequence.data;

  ros_result_response.status = action_msgs__msg__GoalStatus__STATUS_SUCCEEDED;
  ros_result_response.result = result;

  printf("Sending result\n");
  pthread_mutex_lock(&lock);
  RCSOFTCHECK(rcl_action_send_result_response(&action_server, &request_header, &ros_result_response))
  pthread_mutex_unlock(&lock);

  processing_goal = false;
}

void goal_callback(const void * msg, rmw_request_id_t * request){
  example_interfaces__action__Fibonacci_SendGoal_Request * ros_msg = (example_interfaces__action__Fibonacci_SendGoal_Request * ) msg;

  example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
  ros_goal_response.accepted = !processing_goal;

  RCSOFTCHECK(rcl_action_send_goal_response(&action_server, request, &ros_goal_response))

  if (ros_goal_response.accepted) {
    printf("Goal request accepted\n");

    goal_info.goal_id = ros_msg->goal_id;
    goal_handle = rcl_action_accept_new_goal(&action_server, &goal_info);

    RCSOFTCHECK(rcl_action_update_goal_state(goal_handle, GOAL_EVENT_EXECUTE))

    rcl_action_goal_status_array_t c_status_array = rcl_action_get_zero_initialized_goal_status_array();
    RCSOFTCHECK(rcl_action_get_goal_status_array(&action_server, &c_status_array))
    RCSOFTCHECK(rcl_action_publish_status(&action_server, &c_status_array.msg))

    // ---- Calling thread callback
    processing_goal = true;
    pthread_create(&processing_thread, NULL, fibonacci, (void *) &ros_msg->goal.order);

  } else {
    printf("Goal request rejected\n");
  }
}

void result_callback(const void * msg, rmw_request_id_t * request){
  printf("Goal result requested\n");
  result_requested = true;
  request_header = *request;
}

int main()
{	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "fibonacci_action_server_rclc", "", &support));

	// create action server
	RCCHECK(rclc_action_server_init_default(
		&action_server,
		&node,
    &support.clock,
		ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci),
		"fibonacci"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

  RCCHECK(rclc_executor_add_action_server(
    &executor,
    &action_server,
    &goal_request,
    &result_request,
    goal_callback,
    result_callback));

  pthread_mutex_init(&lock, NULL);

  while (1){
    pthread_mutex_lock(&lock);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    pthread_mutex_unlock(&lock);
    usleep(1000);
  }
  
  pthread_mutex_destroy(&lock);

  RCCHECK(rcl_action_server_fini(&action_server, &node))
  RCCHECK(rcl_node_fini(&node));
}