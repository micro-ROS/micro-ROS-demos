#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <example_interfaces/action/fibonacci.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context))

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "fibonacci_action_server_rcl", "", &context, &node_ops))

  const char * action_name = "fibonacci";
  const rosidl_action_type_support_t * action_type_support = ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci);
  rcl_action_server_t action_server = rcl_action_get_zero_initialized_server();
  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  RCCHECK(rcl_ros_clock_init(&clock, &allocator))

  RCCHECK(rcl_action_server_init(&action_server, &node, &clock, action_type_support, action_name, &action_server_ops))

  size_t num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services;

  RCCHECK(rcl_action_server_wait_set_get_num_entities(&action_server, &num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services))

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &context, rcl_get_default_allocator()))


  rcl_action_goal_handle_t * goal_handle;
  rcl_action_goal_info_t goal_info;
  pthread_t goal_thread;

  bool processing_goal = false;
  bool goal_done = false;
  int32_t * feedback;
  int32_t feedback_lenght;
  int32_t goal_order;

  do {
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
    size_t index;
    RCSOFTCHECK(rcl_action_wait_set_add_action_server(&wait_set, &action_server, &index))

    rcl_wait(&wait_set, RCL_MS_TO_NS(200));

    bool is_goal_request_ready = false;
    bool is_cancel_request_ready = false;
    bool is_result_request_ready = false;
    bool is_goal_expired = false;

    RCSOFTCHECK(rcl_action_server_wait_set_get_entities_ready(&wait_set, &action_server, &is_goal_request_ready, &is_cancel_request_ready, &is_result_request_ready, &is_goal_expired))

    if (is_goal_request_ready) { 
      printf("Goal request received\n");

      goal_info = rcl_action_get_zero_initialized_goal_info();
      rmw_request_id_t request_header;
      example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;

      RCSOFTCHECK(rcl_action_take_goal_request(&action_server, &request_header, &ros_goal_request))

      example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
      ros_goal_response.accepted = !processing_goal;
      // ros_goal_response.stamp =
      RCSOFTCHECK(rcl_action_send_goal_response(&action_server, &request_header, &ros_goal_response))

      if (ros_goal_response.accepted) {
        printf("Goal request accepted\n");

        // ---- Accept goal
        goal_info.goal_id = ros_goal_request.goal_id;
        // goal_info.stamp = 
        goal_handle = rcl_action_accept_new_goal(&action_server, &goal_info);

        // ---- Update state
        RCSOFTCHECK(rcl_action_update_goal_state(goal_handle, GOAL_EVENT_EXECUTE))

        // ---- Publish statuses

        rcl_action_goal_status_array_t c_status_array = rcl_action_get_zero_initialized_goal_status_array();
        RCSOFTCHECK(rcl_action_get_goal_status_array(&action_server, &c_status_array))
        RCSOFTCHECK(rcl_action_publish_status(&action_server, &c_status_array.msg))

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
      } else {
        printf("Goal request rejected\n");
      }
    } else if (!goal_done && processing_goal) {
      // ---- Publish feedback
      printf("Publishing feedback\n");

      example_interfaces__action__Fibonacci_FeedbackMessage ros_goal_feedback; 

      ros_goal_feedback.goal_id = goal_info.goal_id;
      ros_goal_feedback.feedback.sequence.data = feedback;
      ros_goal_feedback.feedback.sequence.size = feedback_lenght;
      ros_goal_feedback.feedback.sequence.capacity = goal_order;

      RCSOFTCHECK(rcl_action_publish_feedback(&action_server, &ros_goal_feedback))

    } else if (goal_done && processing_goal) {
      // ---- Sending result ready
      printf("Sending result ready state\n");

      processing_goal = false;

      RCSOFTCHECK(rcl_action_update_goal_state(goal_handle, GOAL_EVENT_SUCCEED))

      RCSOFTCHECK(rcl_action_notify_goal_done(&action_server))

    } else if (is_result_request_ready && goal_done && !processing_goal) {
      printf("Sending result array\n");

      goal_done = false;

      example_interfaces__action__Fibonacci_GetResult_Request ros_result_request;
      rmw_request_id_t request_header;
      RCSOFTCHECK(rcl_action_take_result_request(&action_server, &request_header, &ros_result_request))

      example_interfaces__action__Fibonacci_GetResult_Response ros_result_response;
      example_interfaces__action__Fibonacci_Result result;
      result.sequence.capacity = goal_order;
      result.sequence.size = feedback_lenght;
      result.sequence.data = feedback;

      ros_result_response.status = action_msgs__msg__GoalStatus__STATUS_SUCCEEDED;
      ros_result_response.result = result;

      RCSOFTCHECK(rcl_action_send_result_response(&action_server, &request_header, &ros_result_response))

      free(feedback);
    }
  } while ( true );

  RCCHECK(rcl_action_server_fini(&action_server, &node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}

