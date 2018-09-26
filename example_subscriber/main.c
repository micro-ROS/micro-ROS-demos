#include <rclc/rclc.h>
#include <example_custom_msgs/msg/nested_msg_test.h>

#include <stdio.h>

void on_message(const void* msgin)
{
    const example_custom_msgs__msg__NestedMsgTest* msg = (const example_custom_msgs__msg__NestedMsgTest*)msgin;
    printf("I heard: [%s\t%s\t%s\t%s]\n", msg->data14.data1.data, msg->data14.data2.data, msg->data14.data3.data, msg->data14.data4.data);
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    rclc_init(0, NULL);
    rclc_node_t* node        = rclc_create_node("subscription_node", "");
    rclc_subscription_t* sub = rclc_create_subscription(node, RCLC_GET_MSG_TYPE_SUPPORT(example_custom_msgs, msg, NestedMsgTest), "example_custom_msgs_msg_NestedMsgTest", on_message, 1, false);

    rclc_spin_node(node);

    rclc_destroy_subscription(sub);
    rclc_destroy_node(node);
    return 0;
}
