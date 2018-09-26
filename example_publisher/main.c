#include <rclc/rclc.h>
#include <example_custom_msgs/msg/nested_msg_test.h>

#include <stdio.h>

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    rclc_init(0, NULL);
    rclc_node_t* node = rclc_create_node("publisher_node", "");
    rclc_publisher_t* publisher =
        rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(example_custom_msgs, msg, NestedMsgTest), "example_custom_msgs_msg_NestedMsgTest", 1);

    example_custom_msgs__msg__NestedMsgTest msg;
    char Buff1[30];
    msg.data14.data1.data = Buff1;
    msg.data14.data1.size = 0;
    msg.data14.data1.capacity = sizeof(Buff1);

    char Buff2[30];
    msg.data14.data2.data = Buff2;
    msg.data14.data2.size = 0;
    msg.data14.data2.capacity = sizeof(Buff1);
    
    char Buff3[30];
    msg.data14.data3.data = Buff3;
    msg.data14.data3.size = 0;
    msg.data14.data3.capacity = sizeof(Buff1);
    
    char Buff4[30];
    msg.data14.data4.data = Buff4;
    msg.data14.data4.size = 0;
    msg.data14.data4.capacity = sizeof(Buff1);
    
    

    int num = 0;
    while (rclc_ok())
    {
        msg.data14.data1.size = snprintf(msg.data14.data1.data, msg.data14.data1.capacity, "1 - %i", num++);
        msg.data14.data2.size = snprintf(msg.data14.data2.data, msg.data14.data2.capacity, "2 - %i", num++);
        msg.data14.data3.size = snprintf(msg.data14.data3.data, msg.data14.data3.capacity, "3 - %i", num++);
        msg.data14.data4.size = snprintf(msg.data14.data4.data, msg.data14.data4.capacity, "4 - %i", num++);


        printf("Sending (%i): '%s\t%s\t%s\t%s'\n", sizeof(msg), msg.data14.data1.data, msg.data14.data2.data, msg.data14.data3.data, msg.data14.data4.data);
        rclc_publish(publisher, (const void*)&msg);
        rclc_spin_node_once(node, 500);
    }
    rclc_destroy_publisher(publisher);
    rclc_destroy_node(node);
    return 0;
}
