#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) return -1;

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
    rclc_node_t* node     = rclc_create_node("int32_publisher_c", "");
    CUSTOM_ASSERT(node);
    rclc_publisher_t* publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "std_msgs_msg_Int32", 1);
    CUSTOM_ASSERT(publisher);
    std_msgs__msg__Int32 msg;
    msg.data = -1;

    while (rclc_ok())
    {
        ret = rclc_publish(publisher, (const void*)&msg);
        if (ret == RCL_RET_OK){
            printf("Sending: '%i'\n", msg.data++);
        }
        rclc_spin_node_once(node, 500);
    }
    ret = rclc_destroy_publisher(publisher);
    ret = rclc_destroy_node(node);
    return 0;
}