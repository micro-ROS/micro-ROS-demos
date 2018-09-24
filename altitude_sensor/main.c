#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/u_int32.h>

#include <stdio.h>
#include <math.h>
#include <time.h>


static uint32_t Offset = 0;

/**
 * @brief 
 * 
 * @param msgin 
 */
void on_power_message(const void* msgin)
{
    std_msgs__msg__UInt32* msg = (std_msgs__msg__UInt32*)msgin;

    Offset = msg->data;
}


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
    rclc_init(0, NULL);

    rclc_node_t* node = NULL;
    rclc_publisher_t* publisher = NULL;
    rclc_subscription_t* power_subscription = NULL;


    node = rclc_create_node("altitude_sensor", "");
    publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "std_msgs_msg_Float64", 1);
    //power_subscription = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "std_msgs_msg_UInt32", on_power_message, 1, false);

    double Altitude = 1000;
    double Count = 0;
    time_t t;
    srand((unsigned) time(&t));

    while (rclc_ok())
    {
        Count += 0.1;
        Altitude += (rand() % 10) * cos(Count); 


        // Publish new altitude  
        std_msgs__msg__Float64 msg;
        msg.data = Altitude + Offset;
        rclc_publish(publisher, (const void*)&msg);


        // Spin node
        rclc_spin_node_once(node, 1);
    }

    if (publisher) rclc_destroy_publisher(publisher);
    if (power_subscription) rclc_destroy_publisher(power_subscription);
    if (node) rclc_destroy_node(node);
    
    printf("altitude sendor closed.\n");
    return 0;
}