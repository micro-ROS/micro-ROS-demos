#include <rclc/rclc.h>
#include <example_custom_msgs/msg/nested_msg_test.h>

#include <stdio.h>

void on_message(const void* msgin)
{
    const example_custom_msgs__msg__NestedMsgTest* msg = (const example_custom_msgs__msg__NestedMsgTest*)msgin;
    printf("I heard:\n");
    printf("\tBool: %u\n", msg->data1);
    printf("\tuint8_t: %u\n", msg->data2);
    printf("\tsigned char: %u\n", msg->data3);
    printf("\tfloat: %f\n", msg->data4);
    printf("\tdouble: %lf\n", msg->data5);
    printf("\tint8_t: %i\n", msg->data6);
    printf("\tuint8_t: %u\n", msg->data7);
    printf("\tint16_t: %i\n", msg->data8);
    printf("\tuint16_t: %u\n", msg->data9);
    printf("\tint32_t: %i\n", msg->data10);
    printf("\tuint32_t: %u\n", msg->data11);
    printf("\tint64_t: %li\n", msg->data12);
    printf("\tuint64_t: %lu\n", msg->data13);

    printf("\tstring 1: %s\n", msg->data14.data1.data);
    printf("\tstring 2: %s\n", msg->data14.data2.data);
    printf("\tstring 3: %s\n", msg->data14.data3.data);
    printf("\tstring 4: %s\n", msg->data14.data4.data);
    printf("\n\n");
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
