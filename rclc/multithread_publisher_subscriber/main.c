#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#define STRING_BUFFER_LEN 100
#define PUBLISHER_NUMBER 3

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

struct arg_struct {
    int index;
    rcl_publisher_t *publisher;
};

rcl_node_t node;
pthread_t pub_thr[PUBLISHER_NUMBER];
rcl_publisher_t publisher[PUBLISHER_NUMBER];
rcl_subscription_t subscriber;
std_msgs__msg__Header recv_msg;

volatile bool exit_flag = false;

// Publish thread
void* publish(
        void* args)
{
	struct arg_struct *arguments = args;
	rcl_publisher_t *publisher = arguments->publisher;
	uint8_t id = arguments->index;

	// Create and allocate the publisher message
	std_msgs__msg__Header send_msg;
	char send_msg_buffer[STRING_BUFFER_LEN];
	send_msg.frame_id.data = send_msg_buffer;
	send_msg.frame_id.capacity = STRING_BUFFER_LEN;

	sprintf(send_msg.frame_id.data, "Thread %d", id);
	send_msg.frame_id.size = strlen(send_msg.frame_id.data);

	uint32_t period_us =  1e6 + ((rand()) % 10) * 1e5;
	printf("Thread %d start, publish period: %.1f seconds\n", id, period_us/1000000.0);

    while (!exit_flag)
    {
		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		send_msg.stamp.sec = ts.tv_sec;
		send_msg.stamp.nanosec = ts.tv_nsec;

		RCSOFTCHECK(rcl_publish(publisher, &send_msg, NULL));
		printf("Thread %d sent: %d\n", id, send_msg.stamp.nanosec);	
        usleep(period_us);
    }
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	printf("Received %d from %s\n", msg->stamp.nanosec, msg->frame_id.data);
}

int main(int argc, const char * const * argv)
{
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options
	RCCHECK(rclc_support_init(&support, argc, argv, &allocator));

	// Create node
	RCCHECK(rclc_node_init_default(&node, "multithread_node", "", &support));

	// Create publishers
	for (size_t i = 0; i < PUBLISHER_NUMBER; i++)
	{
		RCCHECK(rclc_publisher_init_default(
			&publisher[i],
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
			"/microROS/multithread"));
	}

	// Create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
		"/microROS/multithread"));
	
	// Allocate the subscriber message
	char recv_msg_buffer[STRING_BUFFER_LEN];
	recv_msg.frame_id.data = recv_msg_buffer;
	recv_msg.frame_id.capacity = STRING_BUFFER_LEN;

	// Create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Start publish threads
	for (size_t i = 0; i < PUBLISHER_NUMBER; i++)
	{
		struct arg_struct *args = malloc(sizeof(struct arg_struct));
		args->publisher = &publisher[i];
		args->index = i;

		pthread_create(&pub_thr[i], NULL, publish, args);
	}

	// Set executor timeout to 100 ms to reduce thread locking time
	executor.timeout_ns = 100000000;
  	rclc_executor_spin(&executor);

	exit_flag = true;

	RCCHECK(rcl_subscription_fini(&subscriber, &node));

	for (size_t i = 0; i < PUBLISHER_NUMBER; i++)
	{
		pthread_join(pub_thr[i], NULL);
		RCCHECK(rcl_publisher_fini(&publisher[i], &node));
	}
	
	RCCHECK(rcl_node_fini(&node));
}
