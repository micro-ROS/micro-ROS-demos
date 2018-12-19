#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) return -1;

void on_message(const void* msgin)
{
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
    printf("I heard: [%i]\n", msg->data);
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

    rclc_node_t* node        = rclc_create_node("int32_subscriber_c", "");
    CUSTOM_ASSERT(node);
    rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                                        "std_msgs_msg_Int32", on_message, 1, false);
    CUSTOM_ASSERT(sub);

    rclc_spin_node(node);

    ret = rclc_destroy_subscription(sub);
    ret = rclc_destroy_node(node);
    return 0;
}
