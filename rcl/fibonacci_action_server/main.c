#include <rcl/rcl.h>
#include <rcl_action/rcl_action.h>
#include <rcl/error_handling.h>
#include <example_interfaces/action/fibonacci.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

typedef struct fibonacci_args {
  rcl_action_server_t * action_server;
  unique_identifier_msgs__msg__UUID uuid;
  uint32_t order;
} fibonacci_args;

void * fibonacci(void *args){
  fibonacci_args * fib_args = (fibonacci_args *)args;

  int32_t * feedback = (int32_t*) malloc(fib_args->order * sizeof(int32_t));

  int t1 = 0, t2 = 1, nextTerm;

  for (uint32_t i = 1; i <= fib_args->order; ++i) {
    nextTerm = t1 + t2;
    t1 = t2;
    t2 = nextTerm;

    feedback[i] = nextTerm;
    example_interfaces__action__Fibonacci_FeedbackMessage ros_goal_feedback;    

    ros_goal_feedback.goal_id = fib_args->uuid;
    ros_goal_feedback.feedback.sequence.data = feedback;
    ros_goal_feedback.feedback.sequence.size = i;
    ros_goal_feedback.feedback.sequence.capacity = fib_args->order;

    rcl_action_publish_feedback(fib_args->action_server, &ros_goal_feedback);
    usleep(500000);
  }
  
  rcl_action_notify_goal_done(fib_args->action_server);
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
  // #include <rosidl_generator_c/action_type_support_struct.h>
  // #include <example_interfaces/action/fibonacci.h>
  const rosidl_action_type_support_t * action_type_support = ROSIDL_GET_ACTION_TYPE_SUPPORT(example_interfaces, Fibonacci);
  rcl_action_server_t action_server = rcl_action_get_zero_initialized_server();
  rcl_action_server_options_t action_server_ops = rcl_action_server_get_default_options();

  rcl_clock_t clock;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rv = rcl_ros_clock_init(&clock, &allocator);

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

  rcl_action_server_wait_set_get_num_entities(&action_server,&num_subscriptions, &num_guard_conditions, &num_timers, &num_clients, &num_services);

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rv = rcl_wait_set_init(&wait_set, num_subscriptions, num_guard_conditions, num_timers, num_clients, num_services, 0, &context, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("Wait set initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_action_goal_handle_t * goal_handle;
  bool processing_goal = false;
  pthread_t goal_thread;

  do {
    rv = rcl_wait_set_clear(&wait_set);
    if (RCL_RET_OK != rv) {
      printf("Wait set clear error: %s\n", rcl_get_error_string().str);
      return 1;
    }
    
    size_t index;
    rv = rcl_action_wait_set_add_action_server(&wait_set, &action_server, &index);
    
    rv = rcl_wait(&wait_set, 1000000);

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
    
    // rcl_action_get_goal_status_array 
    // rcl_action_publish_status
    // rcl_action_take_result_request 
    // rcl_action_send_result_response 
    // rcl_action_expire_goals
    // rcl_action_notify_goal_done 
    // rcl_action_take_cancel_request
    // rcl_action_process_cancel_request
    // rcl_action_send_cancel_response

    if (is_goal_request_ready)
    {   
      rcl_action_goal_info_t goal_info = rcl_action_get_zero_initialized_goal_info();
      rmw_request_id_t request_header;
      example_interfaces__action__Fibonacci_SendGoal_Request ros_goal_request;

      rv = rcl_action_take_goal_request(&action_server, &request_header, &ros_goal_request);

      example_interfaces__action__Fibonacci_SendGoal_Response ros_goal_response;
      ros_goal_response.accepted = !processing_goal;
      // ros_goal_response.stamp =
      rv = rcl_action_send_goal_response(&action_server, &request_header, &ros_goal_response);

      if (ros_goal_response.accepted)
      {
        // ---- Accept goal
        goal_info.goal_id = ros_goal_request.goal_id;
        // goal_info.stamp = 
        goal_handle = rcl_action_accept_new_goal(&action_server, &goal_info);



        // ---- Update state
        rv = rcl_action_update_goal_state(goal_handle, GOAL_EVENT_EXECUTE);



        // ---- Publish statuses

        rcl_action_goal_handle_t ** goal_handles = NULL;
        size_t num_goals = 0;

        rcl_action_server_get_goal_handles(&action_server, &goal_handles, &num_goals);

        rcl_action_goal_status_array_t c_status_array = rcl_action_get_zero_initialized_goal_status_array();
        rcl_action_get_goal_status_array(&action_server, &c_status_array);

        action_msgs__msg__GoalStatusArray status_array;
        // action_msgs__msg__GoalStatus goal_status;

        // goal_status.goal_info

        // status_array.status_list.capacity = 1;
        // status_array.status_list.size = 1;
        // status_array.status_list.data = &goal_status;


        rcl_action_publish_status(&action_server, &c_status_array.msg);
        // ---- Calling thread callback
        processing_goal = true;

        fibonacci_args args = {
          .action_server = &action_server,
          .uuid = ros_goal_request.goal_id,
          .order = ros_goal_request.goal.order,
        };

        pthread_create(&goal_thread, NULL, fibonacci, &args);


      }
      
      
    }
    
  } while ( true );

  rv = rcl_action_server_fini(&action_server, &node);
  rv = rcl_node_fini(&node);

  return 0;
}

