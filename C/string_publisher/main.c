#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>

#define CUSTOM_ASSERT(ptr) if ((ptr) == NULL) return -1;

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

    rclc_node_t* node = rclc_create_node("string_publisher_c", "");
    CUSTOM_ASSERT(node);
    rclc_publisher_t* publisher =
        rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "std_msgs_msg_String", 1);
    CUSTOM_ASSERT(publisher);

    std_msgs__msg__String msg;
    char buff[128]    = {0};
    msg.data.data     = buff;
    msg.data.capacity = sizeof(buff);
    msg.data.size     = 0;

    int num = 0;
    while (rclc_ok())
    {
        msg.data.size = snprintf(msg.data.data, msg.data.capacity, "Hello World %i", num++);
        if (msg.data.size > msg.data.capacity)
            msg.data.size = 0;

        ret = rclc_publish(publisher, (const void*)&msg);
        if (ret == RCL_RET_OK){
            printf("Sending: '%s'\n", msg.data.data);
        } 

        rclc_spin_node_once(node, 500);
    }
    ret = rclc_destroy_publisher(publisher);
    ret = rclc_destroy_node(node);
    return 0;
}
