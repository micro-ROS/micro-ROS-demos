#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <math.h>
#include <time.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) return -1;

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    rclc_ret_t ret;

    ret = rclc_init(0, NULL);
    if (ret != RCL_RET_OK)
    {
        return -1;
    }

    rclc_node_t* node = NULL;
    rclc_publisher_t* publisher = NULL;

    node = rclc_create_node("rad0_altitude_sensor_c", "");
    CUSTOM_ASSERT(node);
    publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "std_msgs_msg_Float64", 1);
    CUSTOM_ASSERT(publisher);


    double A = 0;


    while (rclc_ok())
    {        
        A += 0.0001;

        // Publish new altitude  
        std_msgs__msg__Float64 msg;
        msg.data = 500 * sin(A) + 950;
        ret = rclc_publish(publisher, (const void*)&msg);

        // Spin node
        rclc_spin_node_once(node, 0);    
    }

    if (publisher) ret = rclc_destroy_publisher(publisher);
    if (node) ret = rclc_destroy_node(node);
    
    printf("altitude sendor closed.\n");
    return 0;
}