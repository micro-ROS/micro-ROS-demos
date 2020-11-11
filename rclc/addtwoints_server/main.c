#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "example_interfaces/srv/add_two_ints.h"

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

example_interfaces__srv__AddTwoInts_Request req;
example_interfaces__srv__AddTwoInts_Response res;

typedef void (* rclc_service_callback_t)(const void *, rmw_request_id_t *, void *);

void service_callback(const void * req, rmw_request_id_t * req_id, void * res){
  example_interfaces__srv__AddTwoInts_Request * req_in = (example_interfaces__srv__AddTwoInts_Request *) req;
  example_interfaces__srv__AddTwoInts_Response * res_in = (example_interfaces__srv__AddTwoInts_Response *) res;

  printf("Service request value: %d + %d. Seq %d\n", (int) req_in->a, (int) req_in->b, (int) req_id->sequence_number);

  res_in->sum = req_in->a + req_in->b;
}


int main(int argc, const char * const * argv)
{
  RCLC_UNUSED(argc);
  RCLC_UNUSED(argv);
  rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "add_twoints_client_rclc", "", &support));

  // create service
  rcl_service_t service = rcl_get_zero_initialized_service();
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "/addtwoints"));

  // create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

	unsigned int rcl_wait_timeout = 10;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));

  rclc_executor_spin(&executor);

	RCCHECK(rcl_service_fini(&service, &node));
	RCCHECK(rcl_node_fini(&node));
}
