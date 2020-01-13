#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <example_interfaces/action/fibonacci.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

typedef struct fibonacci_args {
  uint32_t order;
  bool * goal_done;
  int32_t * feedback;
  int32_t * feedback_lenght;
} fibonacci_args;

void * fibonacci(void *args){
  fibonacci_args * fib_args = (fibonacci_args *)args;

  *fib_args->goal_done = false;

  fib_args->feedback[0] = 0;
  fib_args->feedback[1] = 1;
  *fib_args->feedback_lenght = 2;

  for (uint32_t i = 2; i <= fib_args->order; ++i) {

    *fib_args->feedback_lenght = i;
    fib_args->feedback[i] = fib_args->feedback[i-1] + fib_args->feedback[i-2];
    
    usleep(500000);
  }

  *fib_args->goal_done = true;
}

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
  rv = rcl_node_init(&node, "fibonacci_action_server_rcl", "", &context, &node_ops);
  if (RCL_RET_OK != rv) {
    printf("Node initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  const char * action_name = "fibonacci";
  const rosidl_action_type_support_t * action_type_support = ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci);
  rcl_action_server_t action_server = rcl_action_get_zero_initialized_server();
  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rv = rcl_ros_clock_init(&clock, &allocator);
  if (RCL_RET_OK != rv) {
    printf("ros clock init error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rv = rcl_action_server_init(
      &action_server, 
      &node, 
      &clock,
      action_type_support, 
      action_name, 
      &action_server_ops);

  if (RCL_RET_OK != rv) {
    printf("Action server initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;

  rcl_action_server_wait_set_get_num_entities(&action_server, &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services);

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rv = rcl_wait_set_init(&wait_set, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &context, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("Wait set initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_action_goal_handle_t * goal_handle;
  rcl_action_goal_info_t goal_info;
  pthread_t goal_thread;

  bool processing_goal = false;
  bool goal_done = false;
  int32_t * feedback;
  int32_t feedback_lenght;
  int32_t goal_order;

  do {
    rv = rcl_wait_set_clear(&wait_set);
    if (RCL_RET_OK != rv) {
      printf("Wait set clear error: %s\n", rcl_get_error_string().str);
      break;
    }
    
    size_t index;
    rv = rcl_action_wait_set_add_action_server(&wait_set, &action_server, &index);
    if (RCL_RET_OK != rv) {
      printf("wait set add action error: %s\n", rcl_get_error_string().str);
      break;
    }

    rv = rcl_wait(&wait_set, RCL_MS_TO_NS(200));
    if (RCL_RET_OK != rv) {
      printf("rcl wait error: %s\n", rcl_get_error_string().str);
      break;
    }

    bool is_goal_request_ready = false;
    bool is_cancel_request_ready = false;
    bool is_result_request_ready = false;
    bool is_goal_expired = false;

    rv = rcl_action_server_wait_set_get_entities_ready(
          &wait_set, 
          &action_server, 
          &is_goal_request_ready,
          &is_cancel_request_ready,
          &is_result_request_ready,
          &is_goal_expired);
    
    if (RCL_RET_OK != rv) {
      printf("rcl action server get entities error: %s\n", rcl_get_error_string().str);
      break;
    }

    if (is_goal_request_ready)
    { 
      printf("Goal request received\n");

      goal_info = rcl_action_get_zero_initialized_goal_info();
      rmw_request_id_t request_header;
      example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;

      rv = rcl_action_take_goal_request(&action_server, &request_header, &ros_goal_request);

      if (RCL_RET_OK != rv) {
        printf("rcl action take goal error: %s\n", rcl_get_error_string().str);
        break;
      }

      example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
      ros_goal_response.accepted = !processing_goal;
      // ros_goal_response.stamp =
      rv = rcl_action_send_goal_response(&action_server, &request_header, &ros_goal_response);
      
      if (RCL_RET_OK != rv) {
        printf("rcl action send goal response error: %s\n", rcl_get_error_string().str);
        break;
      }

      if (ros_goal_response.accepted)
      {
        printf("Goal request accepted\n");

        // ---- Accept goal
        goal_info.goal_id = ros_goal_request.goal_id;
        // goal_info.stamp = 
        goal_handle = rcl_action_accept_new_goal(&action_server, &goal_info);

        // ---- Update state
        rv = rcl_action_update_goal_state(goal_handle, GOAL_EVENT_EXECUTE);

        if (RCL_RET_OK != rv) {
          printf("rcl action update goal state error: %s\n", rcl_get_error_string().str);
          break;
        }

        // ---- Publish statuses

        rcl_action_goal_status_array_t c_status_array = rcl_action_get_zero_initialized_goal_status_array();
        rcl_action_get_goal_status_array(&action_server, &c_status_array);
        rcl_action_publish_status(&action_server, &c_status_array.msg);

        // ---- Calling thread callback
        processing_goal = true;
        feedback = (int32_t*) malloc(ros_goal_request.goal.order * sizeof(int32_t));
        goal_order = ros_goal_request.goal.order;

        fibonacci_args args = {
          .order = ros_goal_request.goal.order,
          .goal_done = &goal_done,
          .feedback = feedback,
          .feedback_lenght = &feedback_lenght,
        };

        pthread_create(&goal_thread, NULL, fibonacci, &args);
      }else{
        printf("Goal request rejected\n");
      }
    } else if(!goal_done && processing_goal)
    {
      // ---- Publish feedback
      printf("Publishing feedback\n");

      example_interfaces__action__Fibonacci_FeedbackMessage ros_goal_feedback; 

      ros_goal_feedback.goal_id = goal_info.goal_id;
      ros_goal_feedback.feedback.sequence.data = feedback;
      ros_goal_feedback.feedback.sequence.size = feedback_lenght;
      ros_goal_feedback.feedback.sequence.capacity = goal_order;

      rcl_action_publish_feedback(&action_server, &ros_goal_feedback);

    } else if (goal_done && processing_goal)
    {
      // ---- Sending result ready
      printf("Sending result ready state\n");

      processing_goal = false;

      rv = rcl_action_update_goal_state(goal_handle, GOAL_EVENT_SUCCEED);
      
      if (RCL_RET_OK != rv) {
        printf("rcl action update goal state error: %s\n", rcl_get_error_string().str);
        break;
      }

      rv = rcl_action_notify_goal_done(&action_server);

      if (RCL_RET_OK != rv) {
        printf("rcl action notify goal error: %s\n", rcl_get_error_string().str);
        break;
      }

    } else if(is_result_request_ready && goal_done && !processing_goal)
    {
      printf("Sending result array\n");

      goal_done = false;

      example_interfaces__action__Fibonacci_GetResult_Request ros_result_request;
      rmw_request_id_t request_header;
      rv = rcl_action_take_result_request(&action_server, &request_header, &ros_result_request);

      if (RCL_RET_OK != rv) {
        printf("rcl action take result request error: %s\n", rcl_get_error_string().str);
        break;
      }

      example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;
      example_interfaces__action__Fibonacci_Result result;
      result.sequence.capacity = goal_order;
      result.sequence.size = feedback_lenght;
      result.sequence.data = feedback;

      ros_result_response.status = action_msgs__msg__GoalStatus__STATUS_SUCCEEDED;
      ros_result_response.result = result;

      rv = rcl_action_send_result_response(&action_server, &request_header, &ros_result_response);

      if (RCL_RET_OK != rv) {
        printf("rcl action send result response error: %s\n", rcl_get_error_string().str);
        break;
      }

      free(feedback);
    }
    
  } while ( true );

  rv = rcl_action_server_fini(&action_server, &node);
  rv = rcl_node_fini(&node);

  return 0;
}

