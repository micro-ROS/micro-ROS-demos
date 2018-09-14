#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>

static unsigned WaringCount = 0;

void on_message(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;


    printf("[%i] %s\n", WaringCount++, msg->data.data);
}


int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    rclc_init(0, NULL);
    
    rclc_node_t* node        = rclc_create_node("display", "");
    rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "Warning", on_message, 1, false);

    rclc_spin_node(node);

    if (sub) rclc_destroy_subscription(sub);
    if (node) rclc_destroy_node(node);

}