#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/string.h>
//#include <rmw_uros/options.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define STRING_BUFFER_LEN 100
//std_msgs__msg__Header msg;
std_msgs__msg__String msg;
rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
int device_id;
int seq_no;
int pong_count = 0;

// Publishing ping with a timer
void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
  UNUSED(timer);
  UNUSED(last_call_time);
  seq_no = rand();
  printf("new ping \n");
  sprintf(msg.data.data, "%d_%d", seq_no, device_id);
  msg.data.size = strlen(msg.data.data);
  
  // Fill the message timestamp
  // struct timespec ts;
  // clock_gettime(CLOCK_REALTIME, &ts);
  // msg.stamp.sec = ts.tv_sec;
  // msg.stamp.nanosec = ts.tv_nsec;

  // Reset the pong count and publish the ping message
  pong_count = 0;
  rcl_publish(&ping_publisher, &msg, NULL);
  printf("Ping send seq %s\n", msg.data.data);  
}

void ping_callback(const void * msgin)
{

  const std_msgs__msg__String * rcv_msg = (const std_msgs__msg__String *)msgin;
  printf("Ping received with seq %s. Answering.\n", rcv_msg->data.data);
  // Dont pong my own pings
  if(strcmp(msg.data.data,rcv_msg->data.data) != 0){
    printf("Ping received with seq %s. Answering.\n", rcv_msg->data.data);
    rcl_publish(&pong_publisher, rcv_msg, NULL);
  }

}
void pong_callback(const void * msgin)
{
  const std_msgs__msg__String * rcv_msg = (const std_msgs__msg__String *)msgin;
   printf("Pong for seq %s (%d)\n", rcv_msg->data.data, pong_count);
  if(strcmp(msg.data.data,rcv_msg->data.data) == 0) {
    pong_count++;
    printf("Pong for seq %s (%d)\n", rcv_msg->data.data, pong_count);
  }
}

int main(int argc, const char * const * argv)
{
  UNUSED(argc);
  UNUSED(argv);
  //Init RCL options
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&options, rcl_get_default_allocator());
  
  // Init RCL context
  rcl_context_t context = rcl_get_zero_initialized_context();
  rcl_init(0, NULL, &options, &context);

  // Create a node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_init(&node, "pingpong_node", "", &context, &node_ops);

  // Create a reliable ping publisher
  rcl_publisher_options_t ping_publisher_ops = rcl_publisher_get_default_options();
  ping_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&ping_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/ping", &ping_publisher_ops);

  // Create a best effort pong publisher
  rcl_publisher_options_t pong_publisher_ops = rcl_publisher_get_default_options();
  //pong_publisher_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  pong_publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_init(&pong_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/pong", &pong_publisher_ops);

  // Create a best effort  pong subscriber
  rcl_subscription_options_t pong_subscription_ops = rcl_subscription_get_default_options();
  //pong_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rcl_subscription_t pong_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&pong_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/pong", &pong_subscription_ops);

  // Create a best effort ping subscriber
  rcl_subscription_options_t ping_subscription_ops = rcl_subscription_get_default_options();
  //ping_subscription_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rcl_subscription_t ping_subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_init(&ping_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/microROS/ping", &ping_subscription_ops);

  // create a timer, which will call the publish ping
  rcl_clock_t clock;
  rcl_ret_t rc;
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rc = rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_clock_init.\n");
    return -1;
  }
  rcl_timer_t my_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 5000; // 5 seconds
  rc = rcl_timer_init(&my_timer, &clock, &context, RCL_MS_TO_NS(timer_timeout), my_timer_callback, allocator);
  if (rc != RCL_RET_OK) {
    printf("Error in rcl_timer_init.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_timeout);
  }

  // initialize and create the executor
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  unsigned int num_handles = 3;
  rclc_executor_init(&executor, &context, num_handles, &allocator);
  
  // add ping subscription to executor
  std_msgs__msg__String ping_rcv_msg;
  std_msgs__msg__String__init(&ping_rcv_msg);
  rc = rclc_executor_add_subscription(&executor, &ping_subscription, &ping_rcv_msg, &ping_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_subscription. \n");
  }

  // add pong subscription to executor
  std_msgs__msg__String pong_rcv_msg;
  std_msgs__msg__String__init(&pong_rcv_msg);
  rc = rclc_executor_add_subscription(&executor, &pong_subscription, &pong_rcv_msg, &pong_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_subscription. \n");
  }
  
  // add timer to executor (previous guard condition)
  rclc_executor_add_timer(&executor, &my_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer.\n");
  }

  // Create and allocate the pingpong publication message
  char msg_buffer[STRING_BUFFER_LEN];
  std_msgs__msg__String__init(&msg);
  msg.data.data = malloc(STRING_BUFFER_LEN);
  msg.data.capacity = STRING_BUFFER_LEN;

  // Set device id
  device_id = 1; // rand();
  pong_count = 0;
  do {
    // timeout for rcl_wait 100ms  
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    usleep(10000);
  } while (true);

}
