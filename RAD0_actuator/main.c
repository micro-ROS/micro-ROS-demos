#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int32.h>

#include <stdio.h>

#define ASSERT(ptr) if (ptr == NULL) return -1;

static rclc_publisher_t* engine_pub;
static uint32_t engine_power = 100;

void engine_on_message(const void* msgin)
{
    
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    
    if (msg->data + engine_power > 0)
    {
        engine_power += msg->data;
    }
    else
    {
        engine_power = 0;
    }

    // Publish new altitude    
    std_msgs__msg__UInt32  msg_out;
    msg_out.data = engine_power;
    rclc_publish(engine_pub, (const void*)&msg_out);
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    rclc_node_t* node = NULL; 
    rclc_subscription_t* engine_sub = NULL;
    


    rclc_init(0, NULL);
    node = rclc_create_node("actuator", "");
    ASSERT(node);
    engine_sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", engine_on_message, 1, false);
    ASSERT(engine_sub);
    engine_pub = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32), "std_msgs_msg_UInt32", 1);
    ASSERT(engine_pub);

    // Publish new altitude    
    std_msgs__msg__UInt32  msg_out;
    msg_out.data = engine_power;
    rclc_publish(engine_pub, (const void*)&msg_out);

    rclc_spin_node(node);

    //if (altitude_sub) rclc_destroy_subscription(altitude_sub);
    if (engine_sub) rclc_destroy_subscription(engine_sub);
    if (engine_pub) rclc_destroy_publisher(engine_pub);
    if (node) rclc_destroy_node(node);

    printf("Actuator node closed.\n");

    return 0;
}
