#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

void * publisher_thread(void * args)
{
	rcl_publisher_t * pub = (rcl_publisher_t *) args;

	std_msgs__msg__Int32 msg = {0};
    pthread_t ptid = pthread_self();

	while (1)
	{
		RCSOFTCHECK(rcl_publish(pub, &msg, NULL));
		printf("Sent: %d (thread: %ld)\n", msg.data, ptid);
		msg.data++;
		sleep(1);
	}
}

void subscription1_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received in subscription 1: %d\n", msg->data);
}

void subscription2_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received in subscription 2: %d\n", msg->data);
}

int main()
{	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "multithreaded_node", "", &support));
	
	// create publishers
	rcl_publisher_t publishers[2];

	RCCHECK(rclc_publisher_init_default(
		&publishers[0],
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"multithreaded_topic_1"));
	
	RCCHECK(rclc_publisher_init_best_effort(
		&publishers[1],
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"multithreaded_topic_2"));

	// create subscribers
	rcl_subscription_t subscribers[2];
	RCCHECK(rclc_subscription_init_default(
		&subscribers[0],
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"multithreaded_topic_1"));
	
	RCCHECK(rclc_subscription_init_best_effort(
		&subscribers[1],
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"multithreaded_topic_2"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	
	std_msgs__msg__Int32 msg[2];
	rclc_executor_add_subscription(&executor, &subscribers[0], &msg[0], subscription1_callback, ON_NEW_DATA);
	rclc_executor_add_subscription(&executor, &subscribers[1], &msg[1], subscription2_callback, ON_NEW_DATA);

	// Create publisher threads
	pthread_t id[2];
	for (size_t i = 0; i < sizeof(id)/sizeof(pthread_t); i++)
	{
		pthread_create(&id[i], NULL, publisher_thread, &publishers[i]);
	}

	rclc_executor_spin(&executor);
}
