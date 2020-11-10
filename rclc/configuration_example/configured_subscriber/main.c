#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>

#include <rmw_uros/options.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	printf("Received: %d\n", msg->data);
}

int main(int argc, const char * const * argv)
{
	if (argc < 3 || argc > 4)
	{
		printf("Usage: configured_subscriber <IP> <port> <DomainID (default: 0)>\n");
		return 1;
	}

  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	printf("Connecting to agent %s:%d\n",argv[1],atoi(argv[2]));
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_udp_address(argv[1], argv[2], rmw_options))
	RCCHECK(rmw_uros_options_set_client_key(0xCAFEBABE, rmw_options))

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	rcl_node_options_t node_ops = rcl_node_get_default_options();
	node_ops.domain_id = (size_t)(argc == 4 ? atoi(argv[3]) : 0);
	const char * node_name = "int32_configured_subscriber_rclc";
	printf("Initializing node '%s' with ROS Domain ID %ld...\n", node_name, node_ops.domain_id);
	RCCHECK(rclc_node_init_with_options(&node, node_name, "", &support, &node_ops));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"std_msgs_msg_Int32"));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  	rclc_executor_spin(&executor);

	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

	return 0;
}
