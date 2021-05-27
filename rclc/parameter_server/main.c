#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rclc_parameter_server_t param_server;
rcl_timer_t timer;
bool publish = true;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	(void) timer;

	if (publish)
	{
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		printf("Sent: %d\n", msg.data);

		msg.data++;
	}
}

void on_parameter_changed(Parameter * param)
{
	if (strcmp(param->name.data, "publish_toogle") == 0 && param->value.type == RCLC_PARAMETER_BOOL)
	{
		publish = param->value.bool_value;
		printf("Publish %s\n", (publish) ? "ON" : "OFF");
	}
	else if (strcmp(param->name.data, "publish_rate_ms") == 0 && param->value.type == RCLC_PARAMETER_INT)
	{
		int64_t old;
		RCSOFTCHECK(rcl_timer_exchange_period(&timer, RCL_MS_TO_NS(param->value.integer_value), &old));
		printf("Publish rate %ld ms\n", param->value.integer_value);
	}
}

int main()
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"micro_ros_pub"));

  	// Create parameter service
    rclc_parameter_server_init_default(&param_server, &node);

	// create timer,
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(1000),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER + 1, &allocator));
	RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// Add parameters
    rclc_add_parameter(&param_server, "publish_toogle", RCLC_PARAMETER_BOOL);
    rclc_add_parameter(&param_server, "publish_rate_ms", RCLC_PARAMETER_INT);

    rclc_parameter_set_bool(&param_server, "publish_toogle", true);
    rclc_parameter_set_int(&param_server, "publish_rate_ms", 1000);

  	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
}
