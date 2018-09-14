#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>

#include <stdio.h>
#include <math.h>
#include <time.h>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    rclc_init(0, NULL);
    rclc_node_t* node     = rclc_create_node("position_sensor", "");
    rclc_publisher_t* publisher = rclc_create_publisher(node, RCLC_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "position_sensor", 1);

    std_msgs__msg__Float64 msg;
    msg.data = 0;
    double Count = 0;
    srand((unsigned) time(&t));

    while (rclc_ok())
    {
        // Emulate senoidal change position 2000 to 0
        Count += 0.1;
        msg.data = 1000* sin(Count) + 1000; 
            
        rclc_publish(publisher, (const void*)&msg);
        rclc_spin_node_once(node, (rand() % 900 + 100));
    }

    if (publisher) rclc_destroy_publisher(publisher);
    if (node) rclc_destroy_node(node);
    
    return 0;
}