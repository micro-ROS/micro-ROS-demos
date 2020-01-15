#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include "example_interfaces/srv/add_two_ints.h"

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) rc = fn; if((rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)rc); return 1;}
#define RCSOFTCHECK(fn) rc = fn; if((rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)rc);}

int main(int argc, const char * const * argv)
{
  rcl_ret_t rc;

  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&options, rcl_get_default_allocator()))

  rcl_context_t context = rcl_get_zero_initialized_context();
  RCCHECK(rcl_init(argc, argv, &options, &context))

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  RCCHECK(rcl_node_init(&node, "addtowints_client_rcl", "", &context, &node_ops))

  const char * client_name = "/addtwoints";
  
  rcl_client_options_t client_options = rcl_client_get_default_options();
  rcl_client_t client = rcl_get_zero_initialized_client();
  const rosidl_service_type_support_t * client_type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts);

  RCCHECK(rcl_client_init(&client, &node, client_type_support, client_name, &client_options))

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  RCCHECK(rcl_wait_set_init(&wait_set, 0, 0, 0, 1, 0, 0, &context, rcl_get_default_allocator()))

  sleep(2); // Sleep a while to ensure DDS matching before sending request

  // Send request 
  int64_t seq; 
  example_interfaces__srv__AddTwoInts_Request req;
  example_interfaces__srv__AddTwoInts_Request__init(&req);
  req.a = 24;
  req.b = 42;

  RCCHECK(rcl_send_request(&client, &req, &seq))
  printf("Send service request %d + %d. Seq %ld\n",(int)req.a, (int)req.b, (int)seq);

  // Wait for response
  bool done = false;
  do {
    RCSOFTCHECK(rcl_wait_set_clear(&wait_set))
    
    size_t index;
    RCSOFTCHECK(rcl_wait_set_add_client(&wait_set, &client, &index))
    
    RCSOFTCHECK(rcl_wait(&wait_set, RCL_MS_TO_NS(1)))

    for (size_t i = 0; i < wait_set.size_of_clients; i++) {
      if (wait_set.clients[0]) {   
        rmw_request_id_t req_id;
        example_interfaces__srv__AddTwoInts_Response res;
        example_interfaces__srv__AddTwoInts_Response__init(&res);

        rc = rcl_take_response(&client, &req_id, &res);

        if (RCL_RET_OK == rc) {
          printf("Received service response %d + %d = %d. Seq %d\n",(int)req.a, (int)req.b, (int)res.sum,req_id.sequence_number);
          done = true;
        }
      }
    }
  } while ( !done );

  RCCHECK(rcl_client_fini(&client,&node))
  RCCHECK(rcl_node_fini(&node))

  return 0;
}

