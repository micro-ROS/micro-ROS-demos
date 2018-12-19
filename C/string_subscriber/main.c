#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) return -1;

void on_message(const void* msgin)
{
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
    printf("I heard: [%s]\n", msg->data.data);
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    rclc_ret_t ret;

    ret = rclc_init(0, NULL);
    if (ret != RCL_RET_OK)
    {
        return -1;
    }

    rclc_node_t* node        = rclc_create_node("string_subscriber_c", "");
    CUSTOM_ASSERT(node);
    rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                                        "std_msgs_msg_String", on_message, 1, false);
    CUSTOM_ASSERT(sub);                                                  

    rclc_spin_node(node);

    ret = rclc_destroy_subscription(sub);
    ret = rclc_destroy_node(node);
    return 0;
}
