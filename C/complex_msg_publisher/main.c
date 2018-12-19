#include <rclc/rclc.h>
#include <complex_msgs/msg/nested_msg_test.h>

#include <stdio.h>

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
    rclc_node_t* node = rclc_create_node("complex_msg_publisher_c", "");
    if (node == NULL)
    {
        return -1;
    }
    rclc_publisher_t* publisher =
        rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(complex_msgs, msg, NestedMsgTest), "complex_msgs_msg_NestedMsgTest", 1);
    if (publisher == NULL)
    {
        return -1;
    }

    complex_msgs__msg__NestedMsgTest msg;
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
        msg.data1 = (bool)(num & 0x01);
        msg.data2 = (uint8_t)num;
        msg.data3 = (signed char)num;
        msg.data4 = (float)num;
        msg.data5 = (double)num;
        msg.data6 = (int8_t)num;
        msg.data7 = (uint8_t)num;
        msg.data8 = (int16_t)num;
        msg.data9 = (uint16_t)num;
        msg.data10 = (int32_t)num;
        msg.data11 = (uint32_t)num;
        msg.data12 = (int64_t)num;
        msg.data13 = (uint64_t)num;
        msg.data14.data1.size = snprintf(msg.data14.data1.data, msg.data14.data1.capacity, "Msg A - %i", num);
        msg.data14.data2.size = snprintf(msg.data14.data2.data, msg.data14.data2.capacity, "Msg B - %i", num);
        msg.data14.data3.size = snprintf(msg.data14.data3.data, msg.data14.data3.capacity, "Msg C - %i", num);
        msg.data14.data4.size = snprintf(msg.data14.data4.data, msg.data14.data4.capacity, "Msg D - %i", num);
        num++;

        ret = rclc_publish(publisher, (const void*)&msg);
        if (ret == RCL_RET_OK){
            printf("I send:\n");
            printf("\tBool: %u\n", msg.data1);
            printf("\tuint8_t: %u\n", msg.data2);
            printf("\tsigned char: %u\n", msg.data3);
            printf("\tfloat: %f\n", msg.data4);
            printf("\tdouble: %lf\n", msg.data5);
            printf("\tint8_t: %i\n", msg.data6);
            printf("\tuint8_t: %u\n", msg.data7);
            printf("\tint16_t: %i\n", msg.data8);
            printf("\tuint16_t: %u\n", msg.data9);
            printf("\tint32_t: %i\n", msg.data10);
            printf("\tuint32_t: %u\n", msg.data11);
            printf("\tint64_t: %li\n", msg.data12);
            printf("\tuint64_t: %lu\n", msg.data13);

            printf("\tstring 1: %s\n", msg.data14.data1.data);
            printf("\tstring 2: %s\n", msg.data14.data2.data);
            printf("\tstring 3: %s\n", msg.data14.data3.data);
            printf("\tstring 4: %s\n", msg.data14.data4.data);
            printf("\n\n");
        }

        rclc_spin_node_once(node, 500);
    }
    ret = rclc_destroy_publisher(publisher);
    ret = rclc_destroy_node(node);
    return 0;
}
