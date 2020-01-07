#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include "example_interfaces/srv/add_two_ints.h"

#include <stdio.h>
#include <unistd.h>

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
  rv = rcl_node_init(&node, "addtowints_server_rcl", "", &context, &node_ops);
  if (RCL_RET_OK != rv) {
    printf("Node initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  const char * service_name = "addtwoints";
  rcl_service_options_t service_op = rcl_service_get_default_options();
  rcl_service_t serv = rcl_get_zero_initialized_service();
  const rosidl_service_type_support_t * service_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  rv = rcl_service_init(
      &serv,
      &node,
      service_type_support,
      service_name,
      &service_op);

  if (RCL_RET_OK != rv) {
    printf("Server initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rv = rcl_wait_set_init(&wait_set, 0, 0, 0, 0, 1, 0, &context, rcl_get_default_allocator());
  if (RCL_RET_OK != rv) {
    printf("Wait set initialization error: %s\n", rcl_get_error_string().str);
    return 1;
  }

  do {
    rv = rcl_wait_set_clear(&wait_set);
    if (RCL_RET_OK != rv) {
      printf("Wait set clear error: %s\n", rcl_get_error_string().str);
      return 1;
    }
    
    size_t index;
    rv = rcl_wait_set_add_service(&wait_set, &serv, &index);
    if (RCL_RET_OK != rv) {
      printf("Wait set add subscription error: %s\n", rcl_get_error_string().str);
      return 1;
    }    
    
    rv = rcl_wait(&wait_set, 1000000);
    for (size_t i = 0; i < wait_set.size_of_services; i++)
    {
      if (wait_set.services[i])
        {   
            rmw_request_id_t req_id;
            example_interfaces__srv__AddTwoInts_Request req;
            example_interfaces__srv__AddTwoInts_Request__init(&req);
            rv = rcl_take_request(&serv,&req_id,&req);

            printf("Service request value: %d + %d. Seq %d\n", (int)req.a, (int)req.b, (int) req_id.sequence_number);

            example_interfaces__srv__AddTwoInts_Response res;
            example_interfaces__srv__AddTwoInts_Response__init(&res);
            
            res.sum = req.a + req.b;

            rv = rcl_send_response(&serv,&req_id,&res);
        }
    }
  } while ( true );

  rv = rcl_service_fini(&serv,&node);
  rv = rcl_node_fini(&node);

  return 0;
}

