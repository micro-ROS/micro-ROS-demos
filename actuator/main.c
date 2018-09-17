#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float64.h>

#include <stdio.h>

static rclc_publisher_t* publisher;


void altitude_on_message(const void* msgin)
{
    const std_msgs__msg__Float64* msg = (const std_msgs__msg__Float64*)msgin;

    // Publish warning if value is less than 1000 
    if (msg->data <= 500)
    {
        std_msgs__msg__String Warning;
        Warning.data.data = "Failure!!"; 
        Warning.data.size = strlen(Warning.data.data); 
        Warning.data.capacity = strlen(Warning.data.data); 
          
        rclc_publish(publisher, (const void*)&Warning);
    }
    else if (msg->data <= 1000)
    {
        std_msgs__msg__String Warning;
        Warning.data.data = "Warning!!"; 
        Warning.data.size = strlen(Warning.data.data); 
        Warning.data.capacity = strlen(Warning.data.data); 
          
        rclc_publish(publisher, (const void*)&Warning);
    }
}


void engine_on_message(const void* msgin)
{
    const std_msgs__msg__Float64* msg = (const std_msgs__msg__Float64*)msgin;
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    rclc_init(0, NULL);
    rclc_node_t* node        = rclc_create_node("actuator", "");
    rclc_subscription_t* altitude_sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "position_sensor", altitude_on_message, 1, false);
    rclc_subscription_t* engine_sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "engine_power", engine_on_message, 1, false);
    publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "status_report", 1);

    rclc_spin_node(node);

    if (altitude_sub) rclc_destroy_subscription(altitude_sub);
    if (engine_sub) rclc_destroy_subscription(engine_sub);
    if (publisher) rclc_destroy_publisher(publisher);
    if (node) rclc_destroy_node(node);

    return 0;
}
