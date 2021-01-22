#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

#include <rmw_uros/options.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// --- micro-ROS Transports ---

typedef struct {
	char * portname;
	int fd;
} custom_transport_data_t;

bool mbed_serial_open(struct uxrCustomTransport * transport){
    custom_transport_data_t * serial_port = (custom_transport_data_t*) transport->args;
    (void) unlink(serial_port->portname);

	if (0 < mkfifo(serial_port->portname, S_IRWXU | S_IRWXG | S_IRWXO))
    {
        return false;
    }
    else
    {
        serial_port->fd = open("/tmp/serial_fifo", O_RDWR | O_NONBLOCK);
        if (0 < serial_port->fd)
        {
            fcntl(serial_port->fd, F_SETFL, O_NONBLOCK);
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool mbed_serial_close(struct uxrCustomTransport * transport){
    custom_transport_data_t * serial_port = (custom_transport_data_t*) transport->args;
    (void) unlink(serial_port->portname);
	return true;
}

size_t mbed_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    custom_transport_data_t * serial_port = (custom_transport_data_t*) transport->args;
	ssize_t wrote = write(serial_port->fd, buf, len);
    return (wrote >= 0) ? wrote : 0;
}

size_t mbed_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    custom_transport_data_t * serial_port = (custom_transport_data_t*) transport->args;
	ssize_t readed = read(serial_port->fd, buf, len);
	return (readed >= 0) ? readed : 0;  
}


// --- micro-ROS App ---

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
		printf("Sent: %d\n", msg.data);
		msg.data++;
	}
}

int main(int argc, char * const argv[])
{
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_client_key(0xCAFEBABA, rmw_options))

	custom_transport_data_t custom_transport_data;
	char * portname = "/tmp/serial_fifo";
	custom_transport_data.portname = portname;

	RCCHECK(rmw_uros_options_set_custom_transport(
        true,
        (void*) &custom_transport_data,
        mbed_serial_open,
        mbed_serial_close,
        mbed_serial_write,
        mbed_serial_read,
        rmw_options
    ))

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "custom_transport_node", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"std_msgs_msg_Int32"));

	// create timer,
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	msg.data = 0;

  	rclc_executor_spin(&executor);

	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	return 0;
}
